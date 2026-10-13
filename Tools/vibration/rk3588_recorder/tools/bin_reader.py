#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bin_reader.py — 原始震动数据（格式 v3）解析/校验/统计工具

真源文档：../02_原始震动采集器/02_采集软件/数据格式规范.md
机器可读定义：../02_原始震动采集器/02_采集软件/src/vib_format.h

职责：
  1) 结构校验：magic / version / 尺寸 / 文件头与每个块的 CRC
  2) 完整性判定：block_index 缺号（主机丢块）、FIFO_LOST_PKT 增长（器件丢包）、
     包头 flag（FIFO 满 / 短读 / SPI 读错）、削顶与无效样本比例
  3) 时间基准：tmst 16-bit raw ticks 回绕展开、×32/30 换算物理 µs、
     「总 ticks/总时间」实测 ODR（不用 1e6/median）
  4) 物理量：按 ±16 g / ±2000 dps 换算（手册 p.11/p.12），给出各轴 RMS/峰值
  5) 导出：--csv 写逐样本物理量（流式，内存恒定）

退出码：0=无异常；2=检出异常（CRC/缺号/丢包/越窗/削顶超阈）；1=用法或 IO 错误。
"""

from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import zlib
from collections import Counter

# ---------------------------------------------------------------- 常量（与 vib_format.h 对齐）
MAGIC = b"VIB2"
FORMAT_VERSION = 3
FILE_HEADER_SIZE = 256
BLOCK_HEADER_SIZE = 32
RECORD_SIZE = 16
SAMPLES_PER_BLOCK = 32

BLOCK_TYPE_DATA = 0x0001
BLOCK_TYPE_END = 0xFFFF

CLIP_THRESHOLD = 32760          # |raw| >= 32760 判削顶
INVALID_SENTINEL = -32766       # 器件「无效样本」哨兵
TMST_WRAP = 65536               # 16-bit raw ticks 回绕
TMST_SCALE_NUM = 32             # raw ticks → µs 分子（Rev 1.6 §12.7）
TMST_SCALE_DEN = 30             # raw ticks → µs 分母
TMST_US_NOMINAL = 125           # 8 kHz 名义物理 Δ（µs，换算后）
TMST_RAW_LO = 117               # 8 kHz 相邻 raw Δ 窗口下界
TMST_RAW_HI = 118               # 8 kHz 相邻 raw Δ 窗口上界

LSB_PER_G = 2048.0              # ±16 g（手册 p.11）
LSB_PER_DPS = 16.4              # ±2000 dps（手册 p.12）

FILE_HDR_FMT = "<4sHHIIIIHHHHQQBB6sQHHBBBBI176sI"
FILE_HDR_SIZE_CHECK = struct.calcsize(FILE_HDR_FMT)
BLOCK_HDR_FMT = "<HH IQIHHII".replace(" ", "")
BLOCK_HDR_SIZE_CHECK = struct.calcsize(BLOCK_HDR_FMT)
RECORD_FMT = "<3h3hBH B".replace(" ", "")
RECORD_SIZE_CHECK = struct.calcsize(RECORD_FMT)

REC_FLAG_INVALID = 0x01
REC_FLAG_ODR_ACCEL_CHG = 0x02
REC_FLAG_ODR_GYRO_CHG = 0x04
REC_FLAG_HEADER_MSG = 0x08

BLOCK_FLAG_FIFO_FULL = 0x0001
BLOCK_FLAG_SHORT_READ = 0x0002
BLOCK_FLAG_WRITE_RETRY = 0x0004
BLOCK_FLAG_SPI_ERR = 0x0008

AXES = ("ax", "ay", "az", "gx", "gy", "gz")

assert FILE_HDR_SIZE_CHECK == FILE_HEADER_SIZE, FILE_HDR_SIZE_CHECK
assert BLOCK_HDR_SIZE_CHECK == BLOCK_HEADER_SIZE, BLOCK_HDR_SIZE_CHECK
assert RECORD_SIZE_CHECK == RECORD_SIZE, RECORD_SIZE_CHECK


class VibFormatError(Exception):
    pass


class RunningStat:
    """Welford 在线均值/方差 → 恒定内存"""

    __slots__ = ("n", "mean", "m2", "vmin", "vmax")

    def __init__(self) -> None:
        self.n = 0
        self.mean = 0.0
        self.m2 = 0.0
        self.vmin = None
        self.vmax = None

    def push(self, x: float) -> None:
        self.n += 1
        d = x - self.mean
        self.mean += d / self.n
        self.m2 += d * (x - self.mean)
        if self.vmin is None or x < self.vmin:
            self.vmin = x
        if self.vmax is None or x > self.vmax:
            self.vmax = x

    def rms(self) -> float:
        if self.n == 0:
            return 0.0
        var = self.m2 / self.n
        return (var + self.mean * self.mean) ** 0.5

    def as_dict(self) -> dict:
        return {
            "n": self.n,
            "mean": round(self.mean, 4),
            "rms": round(self.rms(), 4),
            "peak": round(max(abs(self.vmin or 0.0), abs(self.vmax or 0.0)), 4),
            "min": round(self.vmin, 4) if self.vmin is not None else None,
            "max": round(self.vmax, 4) if self.vmax is not None else None,
        }


class BinReader:
    def __init__(self, path: str, max_records: int = 0, csv_path: str = None) -> None:
        self.path = path
        self.max_records = max_records
        self.csv_path = csv_path
        self.fh = {}
        self.errors = []          # 硬错误（结构性）
        self.anomalies = []       # 需要人工判断的异常
        self.stats = {}
        self._reset_counters()

    # ------------------------------------------------------------------ 计数器
    def _reset_counters(self) -> None:
        self.blocks = 0
        self.records = 0
        self.samples_total = 0
        self.clip_header_sum = 0
        self.clip_actual = 0
        self.clip_mismatch_blocks = 0
        self.invalid_flag_records = 0
        self.odr_change_records = 0
        self.bad_records = 0            # 长度不是 16 的倍数 / 结构异常
        self.block_index_gaps = 0
        self.missing_blocks = 0
        self.prev_block_index = None
        self.lost_pkt_first = None
        self.lost_pkt_last = None
        self.lost_pkt_steps = 0
        self.fifo_full_blocks = 0
        self.short_blocks = 0
        self.spi_err_blocks = 0
        self.crc_block_errors = 0
        self.tmst_hist = Counter()
        self.tmst_big = 0
        self.tmst_out_of_window = 0
        self.tmst_min = None
        self.tmst_max = None
        self.prev_tmst = None
        self.tmst_raw_total = 0        # Σ 展开后的 raw Δ（整数，规范真源）
        self.tmst_n_deltas = 0         # 参与速率估计的 Δ 数（d ∈ [1,4096)）
        self.tmst_abs_us = 0.0         # 展开后的物理绝对时间轴（µs，浮点，供 CSV）
        self.ticks_to_us_num = TMST_SCALE_NUM
        self.ticks_to_us_den = TMST_SCALE_DEN
        self.end_block_seen = False
        self.axes = {a: RunningStat() for a in AXES}
        self.temp_c = RunningStat()

    # ------------------------------------------------------------------ 头部
    def _read_file_header(self, f) -> None:
        raw = f.read(FILE_HEADER_SIZE)
        if len(raw) < FILE_HEADER_SIZE:
            raise VibFormatError(f"文件头不完整：只有 {len(raw)} B，应为 {FILE_HEADER_SIZE} B")
        if raw[:4] != MAGIC:
            raise VibFormatError(f"magic 不是 {MAGIC!r}，而是 {raw[:4]!r}（格式不符或文件损坏）")

        (magic, ver, hsz, flags, drv, reg_hash, odr, fs_a, fs_g, recb, _res0,
         t_mono, t_utc, who, frame, res1, samples_exp,
         tmst_scale_num, tmst_scale_den, tmst_source, tmst_res,
         count_unit, count_endian, flags2, res2, crc) = struct.unpack(FILE_HDR_FMT, raw)

        stored = zlib.crc32(raw[:252])
        if stored != crc:
            self.errors.append(f"文件头 CRC 不符：文件里 0x{crc:08X}，实算 0x{stored:08X}")
        if hsz != FILE_HEADER_SIZE:
            self.errors.append(f"header_size={hsz}，期望 {FILE_HEADER_SIZE}")
        if recb != RECORD_SIZE:
            self.errors.append(f"record_bytes={recb}，期望 {RECORD_SIZE}")
        if ver != FORMAT_VERSION:
            self.anomalies.append(f"format_version={ver}，本工具面向 v{FORMAT_VERSION}")
        if any(res2):
            self.anomalies.append("文件头 reserved2 非全 0（写入实现与文档不一致）")

        # ---- 时间戳换算：v3 带显式 scale；legacy v2 把 raw ticks 当 µs（旧假设） ----
        if ver >= 3 and tmst_scale_num > 0 and tmst_scale_den > 0:
            self.ticks_to_us_num = tmst_scale_num
            self.ticks_to_us_den = tmst_scale_den
        else:
            self.ticks_to_us_num = 1
            self.ticks_to_us_den = 1
            self.anomalies.append(
                "时间戳换算 scale 缺失/为 0：按 raw=µs 处理。这对应 legacy v2 的旧假设，"
                "物理时间轴不可信（Rev 1.6 §12.7 要求 µs = raw*32/30）")

        self.fh = {
            "magic": magic.decode("ascii", "replace"),
            "format_version": ver,
            "header_size": hsz,
            "flags": flags,
            "driver_version": f"{drv >> 16}.{(drv >> 8) & 0xFF}.{drv & 0xFF}",
            "reg_config_hash": reg_hash,
            "odr_hz_nominal": odr,
            "fs_accel_g": fs_a,
            "fs_gyro_dps": fs_g,
            "record_bytes": recb,
            "t_start_monotonic_ns": t_mono,
            "t_start_utc_ns": t_utc,
            "who_am_i": f"0x{who:02X}",
            "frame": frame,
            "sample_count_expected": samples_exp,
            "tmst_scale_num": tmst_scale_num,
            "tmst_scale_den": tmst_scale_den,
            "tmst_source": tmst_source,
            "tmst_res": tmst_res,
            "count_unit": count_unit,
            "count_endian": count_endian,
            "crc32_ok": stored == crc,
        }
        if who != 0x47:
            self.anomalies.append(f"文件头 who_am_i=0x{who:02X}，期望 0x47")

    # ------------------------------------------------------------------ 主循环
    def run(self) -> dict:
        if os.path.getsize(self.path) < FILE_HEADER_SIZE:
            raise VibFormatError("文件比文件头还短")

        csv_f = None
        if self.csv_path:
            csv_f = open(self.csv_path, "w", encoding="utf-8", newline="")
            csv_f.write("index,t_us," + ",".join(AXES) + ",temp_c,rec_flags\n")

        truncated = False
        with open(self.path, "rb") as f:
            self._read_file_header(f)
            idx = 0
            while True:
                if self.max_records and self.records >= self.max_records:
                    break
                braw = f.read(BLOCK_HEADER_SIZE)
                if not braw:
                    break
                if len(braw) < BLOCK_HEADER_SIZE:
                    self.errors.append(f"末尾块头不完整：只有 {len(braw)} B")
                    truncated = True
                    break

                (btype, n_samples, bindex, t_read, lost, bflags, _bres,
                 clip_hdr, crc_stored) = struct.unpack(BLOCK_HDR_FMT, braw)

                nbytes = n_samples * RECORD_SIZE
                payload = f.read(nbytes) if nbytes else b""
                if len(payload) != nbytes:
                    self.errors.append(
                        f"块 {bindex} payload 不完整：{len(payload)}/{nbytes} B（掉电？）")
                    truncated = True
                    break

                crc_calc = zlib.crc32(payload, zlib.crc32(braw[:28]))
                if crc_calc != crc_stored:
                    self.crc_block_errors += 1
                    self.errors.append(
                        f"块 {bindex} CRC 不符：文件 0x{crc_stored:08X} 实算 0x{crc_calc:08X}")

                # ---- block_index 连续性（主机侧丢块证据）----
                if self.prev_block_index is not None:
                    step = bindex - self.prev_block_index
                    if step != 1:
                        self.block_index_gaps += 1
                        if step > 1:
                            self.missing_blocks += step - 1
                        else:
                            self.anomalies.append(
                                f"块号回退/重复：{self.prev_block_index} → {bindex}")
                self.prev_block_index = bindex

                # ---- FIFO_LOST_PKT（器件侧丢包）----
                if self.lost_pkt_first is None:
                    self.lost_pkt_first = lost
                elif lost != self.lost_pkt_last:
                    self.lost_pkt_steps += 1
                self.lost_pkt_last = lost

                if bflags & BLOCK_FLAG_FIFO_FULL:
                    self.fifo_full_blocks += 1
                if bflags & BLOCK_FLAG_SHORT_READ:
                    self.short_blocks += 1
                if bflags & BLOCK_FLAG_SPI_ERR:
                    self.spi_err_blocks += 1

                if btype == BLOCK_TYPE_END:
                    self.end_block_seen = True
                    break
                if btype != BLOCK_TYPE_DATA:
                    self.anomalies.append(f"未知块类型 0x{btype:04X}（块 {bindex}），跳过")
                    continue

                self.blocks += 1
                self.samples_total += n_samples
                self.clip_header_sum += clip_hdr
                clip_here = 0

                for off in range(0, nbytes, RECORD_SIZE):
                    ax, ay, az, gx, gy, gz, temp_raw, tmst, rflags = struct.unpack_from(
                        RECORD_FMT, payload, off)
                    vals = (ax, ay, az, gx, gy, gz)

                    # 削顶
                    clipped = any(v >= CLIP_THRESHOLD or v <= -CLIP_THRESHOLD for v in vals)
                    if clipped:
                        clip_here += 1
                    if rflags & REC_FLAG_INVALID:
                        self.invalid_flag_records += 1
                    if rflags & (REC_FLAG_ODR_ACCEL_CHG | REC_FLAG_ODR_GYRO_CHG):
                        self.odr_change_records += 1
                    if rflags & REC_FLAG_HEADER_MSG:
                        self.bad_records += 1
                        continue

                    # 时间戳展开：16-bit raw ticks 回绕；物理 µs = raw * 32/30（累加分数）
                    if self.prev_tmst is not None:
                        d = tmst - self.prev_tmst
                        if d < 0:
                            d += TMST_WRAP
                        if d <= 0:
                            self.prev_tmst = tmst
                            continue
                        if d >= 4096:
                            self.tmst_big += 1
                        else:
                            self.tmst_raw_total += d
                            self.tmst_n_deltas += 1
                            self.tmst_hist[d] += 1
                            if d < TMST_RAW_LO or d > TMST_RAW_HI:
                                self.tmst_out_of_window += 1
                            if self.tmst_min is None or d < self.tmst_min:
                                self.tmst_min = d
                            if self.tmst_max is None or d > self.tmst_max:
                                self.tmst_max = d
                            self.tmst_abs_us += d * self.ticks_to_us_num / self.ticks_to_us_den
                    self.prev_tmst = tmst

                    # 物理量
                    self.axes["ax"].push(ax / LSB_PER_G)
                    self.axes["ay"].push(ay / LSB_PER_G)
                    self.axes["az"].push(az / LSB_PER_G)
                    self.axes["gx"].push(gx / LSB_PER_DPS)
                    self.axes["gy"].push(gy / LSB_PER_DPS)
                    self.axes["gz"].push(gz / LSB_PER_DPS)
                    self.temp_c.push(temp_raw / 2.07 + 25.0)

                    if csv_f:
                        csv_f.write("%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%d\n" % (
                            idx, self.tmst_abs_us,
                            ax / LSB_PER_G, ay / LSB_PER_G, az / LSB_PER_G,
                            gx / LSB_PER_DPS, gy / LSB_PER_DPS, gz / LSB_PER_DPS,
                            temp_raw / 2.07 + 25.0, rflags))
                    idx += 1
                    self.records += 1

                self.clip_actual += clip_here
                if clip_here != clip_hdr:
                    self.clip_mismatch_blocks += 1

        if csv_f:
            csv_f.close()

        if not self.end_block_seen:
            self.anomalies.append("未读到结束块（block_type=0xFFFF）——"
                                  "采集被强杀或掉电，最后一块可能不完整")

        # 削顶期望值：>0.1% 即异常（阈值需在 00_口径与判据 里最终定稿）
        clip_ratio = (self.clip_actual / self.records) if self.records else 0.0
        if clip_ratio > 0.001:
            self.anomalies.append(f"削顶比例 {clip_ratio:.3%} > 0.1%，"
                                  "量程或安装方式需复核（见 数据格式规范.md §5）")
        if self.clip_mismatch_blocks:
            self.anomalies.append(f"{self.clip_mismatch_blocks} 个块的 clip_count "
                                  "与逐条重算不一致（写入侧统计有 bug）")
        if self.fifo_full_blocks:
            self.anomalies.append(f"{self.fifo_full_blocks} 个块记录到 FIFO 满"
                                  "（采样期间排空不及，存在不可恢复的器件丢包）")
        if self.short_blocks:
            self.anomalies.append(f"{self.short_blocks} 个块为短读（未达水位即排空）")
        if self.spi_err_blocks:
            self.anomalies.append(f"{self.spi_err_blocks} 个块 status/lost 读取失败"
                                  "（FIFO 健康未知，不能当作 0 丢包）")
        device_lost = (self.lost_pkt_last or 0) - (self.lost_pkt_first or 0)
        if device_lost > 0:
            self.anomalies.append(f"器件 FIFO_LOST_PKT 增长 {device_lost} 条"
                                  "（这是无法事后修补的数据丢失）")
        if self.block_index_gaps:
            self.anomalies.append(f"block_index 有 {self.block_index_gaps} 处跳变，"
                                  f"共缺 {self.missing_blocks} 个块（主机侧丢块）")
        if self.tmst_big:
            self.anomalies.append(f"{self.tmst_big} 处 Δtmst ≥ 4096 µs"
                                  "（记录时间戳字节序或 ODR 配置可疑）")

        self._finalize(device_lost, clip_ratio)
        return self.stats

    # ------------------------------------------------------------------ 汇总
    def _finalize(self, device_lost: int, clip_ratio: float) -> None:
        # 实测 ODR 用「总 raw ticks / 总物理时间」估计，不用 1e6/median
        # （对 117/118 抖动更稳健，且保留全程信息；与 C 端 tmst.c 一致）。
        n_delta = self.tmst_n_deltas
        median_raw = None
        if n_delta:
            acc = 0
            for d in sorted(k for k in self.tmst_hist if k >= 0):
                acc += self.tmst_hist[d]
                if acc * 2 >= n_delta:
                    median_raw = d
                    break
        total_us = self.tmst_raw_total * self.ticks_to_us_num / self.ticks_to_us_den
        measured_odr = (n_delta * 1e6 / total_us) if total_us > 0 and n_delta else None
        duration_s = total_us / 1e6 if total_us else 0.0

        self.stats = {
            "file": self.path,
            "header": self.fh,
            "integrity": {
                "end_block_seen": self.end_block_seen,
                "block_crc_errors": self.crc_block_errors,
                "blocks": self.blocks,
                "records": self.records,
                "block_index_gaps": self.block_index_gaps,
                "missing_blocks": self.missing_blocks,
                "device_lost_pkt_first": self.lost_pkt_first,
                "device_lost_pkt_last": self.lost_pkt_last,
                "device_lost_pkt_delta": device_lost,
                "fifo_full_blocks": self.fifo_full_blocks,
                "short_blocks": self.short_blocks,
                "spi_err_blocks": self.spi_err_blocks,
                "bad_record_flags": self.bad_records,
                "invalid_flag_records": self.invalid_flag_records,
                "odr_change_records": self.odr_change_records,
                "clip_records": self.clip_actual,
                "clip_ratio": round(clip_ratio, 8),
                "clip_header_sum": self.clip_header_sum,
                "clip_mismatch_blocks": self.clip_mismatch_blocks,
            },
            "timing": {
                "duration_s": round(duration_s, 6),
                "tmst_median_raw": median_raw,
                "tmst_median_us": round(median_raw * self.ticks_to_us_num / self.ticks_to_us_den, 3)
                    if median_raw else None,
                "measured_odr_hz": round(measured_odr, 3) if measured_odr else None,
                "nominal_odr_hz": self.fh.get("odr_hz_nominal"),
                "odr_error_ppm": (round((measured_odr - self.fh["odr_hz_nominal"])
                                        / self.fh["odr_hz_nominal"] * 1e6, 1)
                                  if measured_odr and self.fh.get("odr_hz_nominal") else None),
                "tmst_raw_min": self.tmst_min,
                "tmst_raw_max": self.tmst_max,
                "tmst_out_of_window": self.tmst_out_of_window,
                "tmst_out_of_window_ratio": (
                    round(self.tmst_out_of_window / n_delta, 8) if n_delta else 0.0),
                "tmst_big_delta": self.tmst_big,
            },
            "physical": {a: self.axes[a].as_dict() for a in AXES},
            "temp": self.temp_c.as_dict(),
            "errors": self.errors,
            "anomalies": self.anomalies,
            "ok": not self.errors and not self.anomalies,
        }

    # ------------------------------------------------------------------ 打印
    def report(self, out=sys.stdout) -> None:
        s = self.stats
        h, ig, tm = s["header"], s["integrity"], s["timing"]
        p = print
        p("=" * 78)
        p(f"文件      : {s['file']}")
        p(f"格式      : {h['magic']} v{h['format_version']}  驱动 {h['driver_version']}  "
          f"reg_hash=0x{h['reg_config_hash']:08X}  帧={h['frame']}")
        p(f"采集起点  : UTC {h['t_start_utc_ns']} ns  |  MONOTONIC {h['t_start_monotonic_ns']} ns")
        p(f"标称档位  : ODR {h['odr_hz_nominal']} Hz  ±{h['fs_accel_g']} g  "
          f"±{h['fs_gyro_dps']} dps  WHO_AM_I={h['who_am_i']}  头CRC={'OK' if h['crc32_ok'] else 'FAIL'}")
        p("-" * 78)
        p(f"记录数    : {ig['records']}   块数: {ig['blocks']}   "
          f"时长: {tm['duration_s']:.3f} s   实测ODR: {tm['measured_odr_hz']} Hz "
          f"(偏差 {tm['odr_error_ppm']} ppm)")
        p(f"raw Δ     : 中位 {tm['tmst_median_raw']}（≈{tm['tmst_median_us']} µs）  范围 "
          f"[{tm['tmst_raw_min']}, {tm['tmst_raw_max']}]  "
          f"越窗 {tm['tmst_out_of_window']} ({tm['tmst_out_of_window_ratio']:.4%})  "
          f"离群(≥4096) {tm['tmst_big_delta']}")
        p(f"丢包      : 器件 FIFO_LOST_PKT {ig['device_lost_pkt_first']} → "
          f"{ig['device_lost_pkt_last']} (Δ{ig['device_lost_pkt_delta']})   "
          f"主机缺块 {ig['missing_blocks']}({ig['block_index_gaps']} 处)   "
          f"FIFO满块 {ig['fifo_full_blocks']}  短读块 {ig['short_blocks']}")
        p(f"数据质量  : 削顶 {ig['clip_records']} ({ig['clip_ratio']:.4%})   "
          f"无效标志 {ig['invalid_flag_records']}   包头异常 {ig['bad_record_flags']}   "
          f"块CRC错 {ig['block_crc_errors']}   结束块 {'有' if ig['end_block_seen'] else '缺失'}")
        p("-" * 78)
        for a in AXES:
            st = s["physical"][a]
            unit = "g" if a[0] == "a" else "dps"
            p(f"  {a}: RMS {st['rms'] if st['rms'] is not None else 0:>10.4f} {unit:<4}"
              f" 峰值 {st['peak'] if st['peak'] is not None else 0:>10.4f}"
              f"  均值 {st['mean']:>10.4f}  范围 [{st['min']}, {st['max']}]")
        t = s["temp"]
        p(f"  温度: 均值 {t['mean']:.2f} ℃  范围 [{t['min']}, {t['max']}] ℃")
        p("=" * 78)
        if s["errors"]:
            p("硬错误：")
            for e in s["errors"]:
                p(f"  ✗ {e}")
        if s["anomalies"]:
            p("异常（需人工判断）：")
            for e in s["anomalies"]:
                p(f"  ! {e}")
        if s["ok"]:
            p("结论：结构与完整性校验通过 ✅（是否可用于分析仍取决于台架/飞行口径）")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="原始震动数据（格式 v2）解析/校验/统计",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+", help=".bin 文件（可多个）")
    ap.add_argument("--json", nargs="?", const="-", default=None,
                    help="输出 JSON 汇总（默认 stdout；给出路径则写文件）")
    ap.add_argument("--csv", default=None, help="导出逐样本物理量 CSV（仅单文件）")
    ap.add_argument("--max-records", type=int, default=0, help="只解析前 N 条（快速体检）")
    ap.add_argument("--quiet", action="store_true", help="只输出结论行")
    args = ap.parse_args(argv)

    if args.csv and len(args.files) > 1:
        ap.error("--csv 只支持单文件")
    if args.json and args.json != "-" and len(args.files) > 1:
        ap.error("--json 写文件时只支持单文件")

    worst = 0
    results = []
    for path in args.files:
        try:
            r = BinReader(path, max_records=args.max_records,
                          csv_path=args.csv if len(args.files) == 1 else None)
            st = r.run()
        except VibFormatError as e:
            print(f"[{path}] 格式错误：{e}", file=sys.stderr)
            return 1
        except OSError as e:
            print(f"[{path}] 读取失败：{e}", file=sys.stderr)
            return 1

        results.append(st)
        if args.quiet:
            print(f"{path}: records={st['integrity']['records']} "
                  f"odr={st['timing']['measured_odr_hz']} "
                  f"lost={st['integrity']['device_lost_pkt_delta']} "
                  f"gaps={st['integrity']['missing_blocks']} "
                  f"clip={st['integrity']['clip_records']} "
                  f"ok={st['ok']}")
        else:
            r.report()
        if st["errors"]:
            worst = max(worst, 2)
        elif st["anomalies"]:
            worst = max(worst, 2)

    if args.json:
        payload = results[0] if len(results) == 1 else results
        text = json.dumps(payload, ensure_ascii=False, indent=2)
        if args.json == "-":
            print(text)
        else:
            with open(args.json, "w", encoding="utf-8") as f:
                f.write(text + "\n")
            print(f"JSON 已写入 {args.json}", file=sys.stderr)

    return worst


if __name__ == "__main__":
    sys.exit(main())
