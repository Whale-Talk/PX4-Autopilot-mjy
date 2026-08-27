#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
H1 飞控 (px4_fmu-v6x_seeex25d) MAVLink 串口诊断 / 参数工具。

设计要点：
  * 零第三方依赖，只用 stdlib(os/termios/struct/select)。
  * 消息线序、长度、CRC_EXTRA 全部在运行时从本仓库生成的 MAVLink 头解析，
    绝不手抄 —— 避免 fork 方言线序踩坑。
  * 打开串口时先快照 termios 原始状态再强制 raw，用于判定
    "数据乱码 / 写阻塞" 是否由主机侧行规(cooked mode, IXON, CRTSCTS)造成。

子命令:
  probe    抓包取证：termios 快照 + 速率 + 帧统计 + ASCII 提取 + 重启检测 + 写通道测试
  get      读参数
  set      写参数 (--type int|float)
  save     保存参数到存储 (COMMAND_LONG / PREFLIGHT_STORAGE param1=1)
  shell    通过 SERIAL_CONTROL 在 NuttX nsh 上执行命令
  hexdump  纯裸字节 hexdump
"""

import argparse
import glob
import os
import re
import select
import struct
import sys
import time
from collections import Counter

import termios

REPO = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
DEFAULT_MAVLINK_DIR = os.path.join(
    REPO, "build", "px4_fmu-v6x_seeex25d_default", "mavlink")

TYPE_FMT = {
    "char": "c", "uint8_t": "B", "int8_t": "b", "uint16_t": "H",
    "int16_t": "h", "uint32_t": "I", "int32_t": "i", "uint64_t": "Q",
    "int64_t": "q", "float": "f", "double": "d",
}
TYPE_SIZE = {
    "char": 1, "uint8_t": 1, "int8_t": 1, "uint16_t": 2, "int16_t": 2,
    "uint32_t": 4, "int32_t": 4, "uint64_t": 8, "int64_t": 8,
    "float": 4, "double": 8,
}

MAV_PARAM_TYPE_INT32 = 6
MAV_PARAM_TYPE_REAL32 = 9
SERIAL_CONTROL_DEV_SHELL = 10
SC_FLAG_RESPOND = 2
SC_FLAG_EXCLUSIVE = 4
SC_FLAG_MULTI = 16

_re_id = re.compile(r"#define\s+MAVLINK_MSG_ID_([A-Z0-9_]+)\s+(\d+)\s*$", re.M)
_re_len = re.compile(r"#define\s+MAVLINK_MSG_ID_([A-Z0-9_]+)_LEN\s+(\d+)")
_re_crc = re.compile(r"#define\s+MAVLINK_MSG_ID_([A-Z0-9_]+)_CRC\s+(\d+)")
_re_put = re.compile(
    r"_mav_put_([a-z0-9_]+?)(_array)?\(buf,\s*(\d+),\s*([A-Za-z0-9_]+)"
    r"(?:,\s*(\d+))?\)")


# --------------------------------------------------------------------------
# 方言加载：从生成头解析真实线序
# --------------------------------------------------------------------------
def load_dialect(mavlink_dir):
    by_id, by_name = {}, {}
    pattern = os.path.join(mavlink_dir, "**", "mavlink_msg_*.h")
    for path in glob.glob(pattern, recursive=True):
        base = os.path.basename(path)[len("mavlink_msg_"):-2].upper()
        try:
            with open(path, "r", errors="replace") as fh:
                txt = fh.read()
        except OSError:
            continue
        ids = {n: int(v) for n, v in _re_id.findall(txt)
               if not n.endswith(("_LEN", "_CRC", "_MIN_LEN"))}
        lens = {n: int(v) for n, v in _re_len.findall(txt)}
        crcs = {n: int(v) for n, v in _re_crc.findall(txt)}
        if base not in ids or base not in lens or base not in crcs:
            continue
        idx = txt.find("_pack(")
        block = txt[idx:txt.find("\n}", idx)] if idx >= 0 else ""
        fields = []
        for ftype, arr, off, fname, cnt in _re_put.findall(block):
            if ftype not in TYPE_FMT:
                continue
            fields.append((fname, ftype, int(off),
                           int(cnt) if (arr and cnt) else 0))
        msg = {"id": ids[base], "name": base, "len": lens[base],
               "crc": crcs[base], "fields": fields}
        by_id[msg["id"]] = msg
        by_name[base] = msg
    if not by_id:
        sys.exit("错误: 未能从 %s 解析出任何消息定义" % mavlink_dir)
    return by_id, by_name


def decode(msg, payload):
    buf = payload
    if len(buf) < msg["len"]:                 # v2 尾部零截断
        buf = buf + b"\x00" * (msg["len"] - len(buf))
    out = {}
    for fname, ftype, off, cnt in msg["fields"]:
        size = TYPE_SIZE[ftype]
        if cnt:
            raw = buf[off:off + size * cnt]
            if ftype == "char":
                out[fname] = raw.split(b"\x00")[0].decode("ascii", "replace")
            else:
                out[fname] = raw
        else:
            val = struct.unpack_from("<" + TYPE_FMT[ftype], buf, off)[0]
            if ftype == "char":
                val = val.decode("ascii", "replace")
            out[fname] = val
    return out


def encode(msg, vals):
    buf = bytearray(msg["len"])
    for fname, ftype, off, cnt in msg["fields"]:
        if fname not in vals:
            continue
        val = vals[fname]
        size = TYPE_SIZE[ftype]
        if cnt:
            if isinstance(val, str):
                val = val.encode("ascii", "replace")
            val = bytes(val)[:size * cnt].ljust(size * cnt, b"\x00")
            buf[off:off + size * cnt] = val
        elif isinstance(val, (bytes, bytearray)):   # 原始位模式(int 参数存 float 字段)
            buf[off:off + size] = bytes(val)[:size]
        else:
            struct.pack_into("<" + TYPE_FMT[ftype], buf, off, val)
    return bytes(buf)


# --------------------------------------------------------------------------
# CRC (MCRF4XX / 反射 X25)
# --------------------------------------------------------------------------
def crc_calc(data, extra):
    crc = 0xFFFF
    for byte in tuple(data) + (extra,):
        tmp = (byte ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


# --------------------------------------------------------------------------
# termios: 快照 + 强制 raw
# --------------------------------------------------------------------------
_IFLAGS = ["IGNBRK", "BRKINT", "IGNPAR", "PARMRK", "INPCK", "ISTRIP",
           "INLCR", "IGNCR", "ICRNL", "IUCLC", "IXON", "IXANY", "IXOFF"]
_OFLAGS = ["OPOST", "ONLCR", "OCRNL", "ONOCR", "ONLRET"]
_CFLAGS = ["CSTOPB", "CREAD", "PARENB", "PARODD", "HUPCL", "CLOCAL", "CRTSCTS"]
_LFLAGS = ["ISIG", "ICANON", "ECHO", "ECHOE", "ECHONL", "IEXTEN"]


def termios_report(attr):
    if attr is None:
        return ["  (无法读取 termios)"]
    iflag, oflag, cflag, lflag = attr[0], attr[1], attr[2], attr[3]
    lines = []
    for label, value, names in (("iflag", iflag, _IFLAGS),
                                ("oflag", oflag, _OFLAGS),
                                ("cflag", cflag, _CFLAGS),
                                ("lflag", lflag, _LFLAGS)):
        on = [n for n in names if value & getattr(termios, n, 0)]
        lines.append("  %-6s 0x%08x  %s" % (label, value,
                                            " ".join(on) if on else "(none)"))
    cc = attr[6]
    lines.append("  VMIN=%s VTIME=%s" % (cc[termios.VMIN], cc[termios.VTIME]))
    return lines


DANGEROUS = [("iflag", "IXON", "收到 0x13(XOFF) 会暂停本机输出 → 写阻塞"),
             ("iflag", "IXOFF", "本机自动发 XON/XOFF → 污染二进制流"),
             ("iflag", "ICRNL", "0x0D→0x0A 改写 → 帧体损坏"),
             ("iflag", "INLCR", "0x0A→0x0D 改写 → 帧体损坏"),
             ("iflag", "IGNCR", "丢弃 0x0D → 帧体缺字节"),
             ("iflag", "ISTRIP", "剥掉第 8 位 → 帧体损坏"),
             ("oflag", "OPOST", "输出被行规改写 → 发出的帧损坏"),
             ("cflag", "CRTSCTS", "硬件流控无 CTS → 写永久阻塞"),
             ("lflag", "ICANON", "行缓冲，read 只在换行时返回 → 速率假性骤降"),
             ("lflag", "ECHO", "回显 → 把收到的字节又发回设备")]


def diagnose_termios(attr):
    if attr is None:
        return []
    idx = {"iflag": 0, "oflag": 1, "cflag": 2, "lflag": 3}
    hits = []
    for group, name, why in DANGEROUS:
        bit = getattr(termios, name, 0)
        if bit and (attr[idx[group]] & bit):
            hits.append("  [!] %-8s 置位 → %s" % (name, why))
    return hits


def open_port(dev, baud=115200):
    fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    before = None
    try:
        before = termios.tcgetattr(fd)
    except termios.error:
        return fd, None
    attr = list(before)
    attr[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK
                 | termios.ISTRIP | termios.INLCR | termios.IGNCR
                 | termios.ICRNL | termios.IXON | termios.IXOFF
                 | termios.IXANY)
    attr[1] &= ~termios.OPOST
    attr[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON
                 | termios.ISIG | termios.IEXTEN)
    attr[2] &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB
                 | getattr(termios, "CRTSCTS", 0))
    attr[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
    speed = getattr(termios, "B%d" % baud, termios.B115200)
    attr[4] = speed
    attr[5] = speed
    cc = list(attr[6])
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    attr[6] = cc
    termios.tcsetattr(fd, termios.TCSANOW, attr)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd, before


# --------------------------------------------------------------------------
# 帧扫描
# --------------------------------------------------------------------------
def scan_frames(buf, by_id):
    """返回 (frames, junk_bytes, consumed)。frames: (ver, msgid, payload, offset)"""
    frames, junk, i, n = [], 0, 0, len(buf)
    while i < n:
        byte = buf[i]
        if byte == 0xFE:
            if i + 2 > n:
                break
            ln = buf[i + 1]
            need = 8 + ln
            if i + need > n:
                break
            msg = by_id.get(buf[i + 5])
            if msg is not None and ln <= msg["len"]:
                crc = struct.unpack_from("<H", buf, i + 6 + ln)[0]
                if crc_calc(buf[i + 1:i + 6 + ln], msg["crc"]) == crc:
                    frames.append((1, msg["id"],
                                   bytes(buf[i + 6:i + 6 + ln]), i))
                    i += need
                    continue
        elif byte == 0xFD:
            if i + 3 > n:
                break
            ln = buf[i + 1]
            signed = 13 if (buf[i + 2] & 0x01) else 0
            need = 12 + ln + signed
            if i + need > n:
                break
            mid = buf[i + 7] | (buf[i + 8] << 8) | (buf[i + 9] << 16)
            msg = by_id.get(mid)
            if msg is not None and ln <= msg["len"]:
                crc = struct.unpack_from("<H", buf, i + 10 + ln)[0]
                if crc_calc(buf[i + 1:i + 10 + ln], msg["crc"]) == crc:
                    frames.append((2, mid, bytes(buf[i + 10:i + 10 + ln]), i))
                    i += need
                    continue
        i += 1
        junk += 1
    return frames, junk, i


class Link(object):
    def __init__(self, dev, by_id, by_name, sysid=255, compid=190):
        self.fd, self.before = open_port(dev)
        self.by_id, self.by_name = by_id, by_name
        self.sysid, self.compid, self.seq = sysid, compid, 0
        self.buf = bytearray()

    def close(self):
        # 故意不恢复打开前的 termios: 原状态可能是 cooked 模式(ICRNL/IXON/
        # OPOST/ICANON)，恢复它会重新把端口弄坏。保持 raw 是正确的终态。
        os.close(self.fd)

    def drain_until_live(self, max_seconds=40.0, window=1.0, tol=2.0,
                         verbose=True):
        """排空积压直到流追上实时。

        端口未被任何进程打开时飞控遥测会在链路上堆积；新打开端口后先读到的
        是几分钟前的旧数据，此时任何请求的应答都排在积压之后，表现为"无应答"。
        用 time_boot_ms 相对壁钟的推进倍率判定是否已追上：>>1 表示还在排积压。
        """
        t_start = time.monotonic()
        while time.monotonic() - t_start < max_seconds:
            w0 = time.monotonic()
            tmin = tmax = None
            while time.monotonic() - w0 < window:
                for _ver, mid, pl, _off in self.pump(0.2):
                    msg = self.by_id[mid]
                    if any(f[0] == "time_boot_ms" for f in msg["fields"]):
                        val = decode(msg, pl)["time_boot_ms"]
                        if tmin is None:
                            tmin = val
                        tmax = val
            if tmin is None:
                return False
            wall = time.monotonic() - w0
            ratio = ((tmax - tmin) / 1000.0) / wall
            if ratio <= tol:
                if verbose:
                    sys.stderr.write(
                        "  [drain] 已追上实时 (推进 %.2fx, FC uptime %.1fs)\n"
                        % (ratio, tmax / 1000.0))
                return True
            if verbose:
                sys.stderr.write("  [drain] 排积压中 推进 %.1fx ...\n" % ratio)
        return False

    def send(self, name, vals):
        msg = self.by_name[name]
        if msg["id"] > 255:
            raise ValueError("msgid>255 需 v2 组帧: " + name)
        payload = encode(msg, vals)
        head = bytes([len(payload), self.seq & 0xFF, self.sysid,
                      self.compid, msg["id"]])
        self.seq = (self.seq + 1) & 0xFF
        crc = crc_calc(head + payload, msg["crc"])
        frame = b"\xfe" + head + payload + struct.pack("<H", crc)
        deadline = time.monotonic() + 3.0
        sent = 0
        while sent < len(frame):
            if time.monotonic() > deadline:
                raise IOError("写超时: 只写入 %d/%d 字节 (写通道阻塞)"
                              % (sent, len(frame)))
            _, wfd, _ = select.select([], [self.fd], [], 0.2)
            if not wfd:
                continue
            sent += os.write(self.fd, frame[sent:])
        return len(frame)

    def pump(self, timeout):
        """读一次并解析出帧列表"""
        rfd, _, _ = select.select([self.fd], [], [], timeout)
        if not rfd:
            return []
        try:
            chunk = os.read(self.fd, 8192)
        except BlockingIOError:
            return []
        if not chunk:
            return []
        self.buf += chunk
        frames, _, consumed = scan_frames(self.buf, self.by_id)
        del self.buf[:consumed]
        if len(self.buf) > 65536:
            del self.buf[:-4096]
        return frames

    def wait_for(self, msg_names, timeout=3.0, match=None):
        wanted = {self.by_name[n]["id"] for n in msg_names}
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for ver, mid, payload, _off in self.pump(
                    max(0.05, deadline - time.monotonic())):
                if mid not in wanted:
                    continue
                data = decode(self.by_id[mid], payload)
                if match is None or match(data):
                    return self.by_id[mid]["name"], data
        return None, None


# --------------------------------------------------------------------------
# probe
# --------------------------------------------------------------------------
def printable_runs(buf, minlen=6, top=15):
    runs, cur = [], bytearray()
    for byte in buf:
        if 0x20 <= byte <= 0x7E or byte in (0x09, 0x0A, 0x0D):
            cur.append(byte)
        else:
            if len(cur) >= minlen:
                runs.append(bytes(cur))
            cur = bytearray()
    if len(cur) >= minlen:
        runs.append(bytes(cur))
    counter = Counter(r.decode("ascii", "replace").strip() for r in runs)
    return counter.most_common(top)


def cmd_probe(args, by_id, by_name):
    print("=" * 74)
    print("H1 飞控串口取证  dev=%s  时长=%ss" % (args.dev, args.seconds))
    print("=" * 74)

    fd, before = open_port(args.dev)
    print("\n[1] 打开前 termios 快照 (判定主机侧行规是否为乱码/写阻塞元凶)")
    for line in termios_report(before):
        print(line)
    hits = diagnose_termios(before)
    if hits:
        print("  ↓ 危险标志位:")
        for line in hits:
            print(line)
    else:
        print("  → 未发现危险标志位: 端口原本已是干净的 raw 模式")

    print("\n[2] 已强制 raw 模式，开始抓包 ...")
    raw = bytearray()
    t0 = time.monotonic()
    gaps, last_rx = [], t0
    while time.monotonic() - t0 < args.seconds:
        rfd, _, _ = select.select([fd], [], [], 0.5)
        now = time.monotonic()
        if rfd:
            try:
                chunk = os.read(fd, 65536)
            except BlockingIOError:
                continue
            if chunk:
                if now - last_rx > 0.5:
                    gaps.append((round(now - t0, 2), round(now - last_rx, 2)))
                last_rx = now
                raw += chunk
    elapsed = time.monotonic() - t0

    total = len(raw)
    print("    收到 %d 字节 / %.2f s = %.0f B/s" %
          (total, elapsed, total / elapsed if elapsed else 0))
    if gaps:
        print("    数据空档(>0.5s): %s" % gaps[:10])

    if args.save_raw:
        with open(args.save_raw, "wb") as fh:
            fh.write(raw)
        print("    裸字节已存: %s" % args.save_raw)

    if not total:
        print("\n[结论] 完全无数据 —— 飞控未输出或端口错误")
        os.close(fd)
        return

    print("\n[3] 帧解析 (v1 0xFE + v2 0xFD, 全部做 CRC_EXTRA 校验)")
    frames, junk, consumed = scan_frames(raw, by_id)
    framed = total - junk - (total - consumed)
    print("    合法帧: %d 个, 占用 %d 字节 (%.1f%%)"
          % (len(frames), framed, 100.0 * framed / total))
    print("    无法归帧的散字节: %d (%.1f%%)"
          % (junk, 100.0 * junk / total))
    vers = Counter(f[0] for f in frames)
    print("    版本分布: v1=%d  v2=%d" % (vers.get(1, 0), vers.get(2, 0)))

    if frames:
        hist = Counter(by_id[f[1]]["name"] for f in frames)
        print("\n    消息直方图 (Top 20):")
        for name, cnt in hist.most_common(20):
            print("      %-32s %5d" % (name, cnt))

    print("\n[4] ASCII 文本提取 (捕捉 NuttX 启动横幅 / hardfault / nsh 提示)")
    runs = printable_runs(raw)
    if runs:
        for text, cnt in runs:
            if text:
                print("      x%-4d %r" % (cnt, text[:100]))
    else:
        print("      无可读 ASCII 串 → 流内不含控制台文本")

    print("\n[5] 重启/boot loop 判定 (time_boot_ms 单调性)")
    series = []
    for ver, mid, payload, _off in frames:
        msg = by_id[mid]
        if any(f[0] == "time_boot_ms" for f in msg["fields"]):
            data = decode(msg, payload)
            series.append((msg["name"], data["time_boot_ms"]))
    if not series:
        print("      流中无携带 time_boot_ms 的消息 → 无法判定")
    else:
        values = [v for _n, v in series]
        drops = [(series[i - 1], series[i]) for i in range(1, len(series))
                 if values[i] + 1000 < values[i - 1]]
        print("      样本 %d 个, time_boot_ms 范围 %d ms .. %d ms (跨度 %.1f s)"
              % (len(values), min(values), max(values),
                 (max(values) - min(values)) / 1000.0))
        if drops:
            print("      [!] 检测到 %d 次回退 → 飞控确实在重启:" % len(drops))
            for prev, cur in drops[:5]:
                print("          %s %d ms → %s %d ms"
                      % (prev[0], prev[1], cur[0], cur[1]))
        else:
            print("      → 单调递增，未检测到重启：排除 boot loop")

    hb = [decode(by_id[f[1]], f[2]) for f in frames
          if by_id[f[1]]["name"] == "HEARTBEAT"]
    if hb:
        last = hb[-1]
        print("\n[6] HEARTBEAT: type=%s autopilot=%s base_mode=0x%02x "
              "custom_mode=0x%08x system_status=%s"
              % (last.get("type"), last.get("autopilot"),
                 last.get("base_mode", 0), last.get("custom_mode", 0),
                 last.get("system_status")))
    os.close(fd)

    print("\n[7] 写通道测试 (raw 模式下发 PARAM_REQUEST_READ 并等应答)")
    link = Link(args.dev, by_id, by_name)
    try:
        link.drain_until_live()
        target = args.write_test_param
        t_w = time.monotonic()
        nbytes = link.send("PARAM_REQUEST_READ",
                           {"param_index": -1, "target_system": 1,
                            "target_component": 1, "param_id": target})
        print("    写入 %d 字节耗时 %.1f ms" % (nbytes,
                                          (time.monotonic() - t_w) * 1000))
        name, data = link.wait_for(
            ["PARAM_VALUE"], timeout=4.0,
            match=lambda d: d.get("param_id") == target)
        if data:
            print("    → 收到 PARAM_VALUE %s = %s (type=%s)  写通道正常"
                  % (data["param_id"], fmt_param(data), data["param_type"]))
        else:
            print("    → 4s 内无 PARAM_VALUE 应答  写通道或飞控侧异常")
    except IOError as exc:
        print("    [!] %s" % exc)
    finally:
        link.close()
    print("\n" + "=" * 74)


def fmt_param(data):
    raw = struct.pack("<f", data["param_value"])
    if data["param_type"] in (1, 2, 3, 4, 5, 6, 7, 8):
        return str(struct.unpack("<i", raw)[0])
    return repr(round(data["param_value"], 6))


# --------------------------------------------------------------------------
# 参数 / shell
# --------------------------------------------------------------------------
def cmd_get(args, by_id, by_name):
    link = Link(args.dev, by_id, by_name)
    try:
        link.drain_until_live()
        for name in args.params:
            link.send("PARAM_REQUEST_READ",
                      {"param_index": -1, "target_system": 1,
                       "target_component": 1, "param_id": name})
            _n, data = link.wait_for(
                ["PARAM_VALUE"], timeout=4.0,
                match=lambda d, want=name: d.get("param_id") == want)
            if data:
                print("%-18s = %-14s (type=%d, index=%d)"
                      % (name, fmt_param(data), data["param_type"],
                         data["param_index"]))
            else:
                print("%-18s = <无应答>" % name)
    finally:
        link.close()


def cmd_set(args, by_id, by_name):
    link = Link(args.dev, by_id, by_name)
    try:
        link.drain_until_live()
        if args.type == "int":
            bits = struct.pack("<i", int(args.value))
            ptype = MAV_PARAM_TYPE_INT32
        else:
            bits = struct.pack("<f", float(args.value))
            ptype = MAV_PARAM_TYPE_REAL32
        link.send("PARAM_SET",
                  {"param_value": bits, "target_system": 1,
                   "target_component": 1, "param_id": args.param,
                   "param_type": ptype})
        _n, data = link.wait_for(
            ["PARAM_VALUE"], timeout=4.0,
            match=lambda d: d.get("param_id") == args.param)
        if data:
            print("%s 已写入, 回读 = %s" % (args.param, fmt_param(data)))
        else:
            print("%s 写入后无 PARAM_VALUE 确认" % args.param)
    finally:
        link.close()


def cmd_save(args, by_id, by_name):
    link = Link(args.dev, by_id, by_name)
    try:
        link.drain_until_live()
        link.send("COMMAND_LONG",
                  {"param1": 1.0, "param2": 0.0, "param3": 0.0, "param4": 0.0,
                   "param5": 0.0, "param6": 0.0, "param7": 0.0,
                   "command": 245, "target_system": 1, "target_component": 1,
                   "confirmation": 0})
        _n, data = link.wait_for(["COMMAND_ACK"], timeout=5.0,
                                 match=lambda d: d.get("command") == 245)
        print("PREFLIGHT_STORAGE ack: %s"
              % (data if data else "<无应答>"))
    finally:
        link.close()


def cmd_shell(args, by_id, by_name):
    link = Link(args.dev, by_id, by_name)
    flags = SC_FLAG_RESPOND | SC_FLAG_EXCLUSIVE | SC_FLAG_MULTI
    try:
        link.drain_until_live()
        out = bytearray()
        for cmd in args.command:
            payload = (cmd + "\n").encode()
            link.send("SERIAL_CONTROL",
                      {"baudrate": 0, "timeout": 0,
                       "device": SERIAL_CONTROL_DEV_SHELL, "flags": flags,
                       "count": len(payload), "data": payload,
                       "target_system": 1, "target_component": 1})
            deadline = time.monotonic() + args.timeout
            while time.monotonic() < deadline:
                for _ver, mid, pl, _off in link.pump(0.2):
                    if by_id[mid]["name"] != "SERIAL_CONTROL":
                        continue
                    data = decode(by_id[mid], pl)
                    out += data["data"][:data["count"]]
                link.send("SERIAL_CONTROL",
                          {"baudrate": 0, "timeout": 0,
                           "device": SERIAL_CONTROL_DEV_SHELL, "flags": flags,
                           "count": 0, "data": b"", "target_system": 1,
                           "target_component": 1})
        sys.stdout.write(out.decode("ascii", "replace"))
        sys.stdout.write("\n")
    finally:
        link.close()


def cmd_hexdump(args, by_id, by_name):
    fd, _before = open_port(args.dev)
    try:
        t0 = time.monotonic()
        raw = bytearray()
        while time.monotonic() - t0 < args.seconds and len(raw) < args.nbytes:
            rfd, _, _ = select.select([fd], [], [], 0.3)
            if rfd:
                raw += os.read(fd, 4096)
        for off in range(0, min(len(raw), args.nbytes), 16):
            row = raw[off:off + 16]
            print("%08x  %-47s  |%s|" % (
                off, " ".join("%02x" % b for b in row),
                "".join(chr(b) if 0x20 <= b <= 0x7E else "." for b in row)))
    finally:
        os.close(fd)


def cmd_rcwatch(args, by_id, by_name):
    """实时监视 RC UART 字节速率 —— 边动接收机边看反馈。"""
    link = Link(args.dev, by_id, by_name)
    flags = SC_FLAG_RESPOND | SC_FLAG_EXCLUSIVE | SC_FLAG_MULTI

    def nsh(cmd, wait):
        out = bytearray()
        payload = (cmd + "\n").encode()
        link.send("SERIAL_CONTROL",
                  {"baudrate": 0, "timeout": 0,
                   "device": SERIAL_CONTROL_DEV_SHELL, "flags": flags,
                   "count": len(payload), "data": payload,
                   "target_system": 1, "target_component": 1})
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline:
            for _v, mid, pl, _o in link.pump(0.1):
                if by_id[mid]["name"] == "SERIAL_CONTROL":
                    data = decode(by_id[mid], pl)
                    out += data["data"][:data["count"]]
            link.send("SERIAL_CONTROL",
                      {"baudrate": 0, "timeout": 0,
                       "device": SERIAL_CONTROL_DEV_SHELL, "flags": flags,
                       "count": 0, "data": b"", "target_system": 1,
                       "target_component": 1})
        return out.decode("ascii", "replace")

    try:
        link.drain_until_live()
        print("监视 /dev/ttyS5 的 RC 字节速率。正常 SBUS ≈ 1700-3600 B/s；"
              "< 10 B/s = 线上无信号")
        print("%-7s %11s %9s  %s" % ("t(s)", "RX bytes", "B/s", "RC state"))
        prev = prev_t = None
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.seconds:
            text = nsh("rc_input status", args.interval)
            now = time.monotonic()
            m_b = re.search(r"UART RX bytes:\s*(\d+)", text)
            m_s = re.search(r"RC state:\s*(.+)", text)
            if not m_b:
                print("%-7.1f %11s %9s  %s"
                      % (now - t0, "?", "?", "(无 nsh 应答)"))
                continue
            cur = int(m_b.group(1))
            rate = ((cur - prev) / (now - prev_t)) if prev is not None else 0.0
            print("%-7.1f %11d %9.1f  %s"
                  % (now - t0, cur, rate,
                     m_s.group(1).strip() if m_s else "?"))
            prev, prev_t = cur, now
    except KeyboardInterrupt:
        pass
    finally:
        link.close()


def main():
    ap = argparse.ArgumentParser(description="H1 飞控 MAVLink 诊断工具")
    ap.add_argument("--dev", default="/dev/ttyACM0")
    ap.add_argument("--mavlink-dir", default=DEFAULT_MAVLINK_DIR)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("probe", help="抓包取证")
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--save-raw", default=None)
    p.add_argument("--write-test-param", default="SYS_AUTOSTART")
    p.set_defaults(func=cmd_probe)

    p = sub.add_parser("get", help="读参数")
    p.add_argument("params", nargs="+")
    p.set_defaults(func=cmd_get)

    p = sub.add_parser("set", help="写参数")
    p.add_argument("param")
    p.add_argument("value")
    p.add_argument("--type", choices=["int", "float"], default="int")
    p.set_defaults(func=cmd_set)

    p = sub.add_parser("save", help="保存参数")
    p.set_defaults(func=cmd_save)

    p = sub.add_parser("rcwatch", help="实时监视 RC UART 字节速率")
    p.add_argument("--seconds", type=float, default=120.0)
    p.add_argument("--interval", type=float, default=2.5)
    p.set_defaults(func=cmd_rcwatch)

    p = sub.add_parser("shell", help="nsh 命令")
    p.add_argument("command", nargs="+")
    p.add_argument("--timeout", type=float, default=4.0)
    p.set_defaults(func=cmd_shell)

    p = sub.add_parser("hexdump", help="裸字节")
    p.add_argument("--seconds", type=float, default=3.0)
    p.add_argument("--nbytes", type=int, default=512)
    p.set_defaults(func=cmd_hexdump)

    args = ap.parse_args()
    by_id, by_name = load_dialect(args.mavlink_dir)
    args.func(args, by_id, by_name)


if __name__ == "__main__":
    main()
