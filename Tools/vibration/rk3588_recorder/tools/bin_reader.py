#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bin_reader.py — 原始震动数据（格式 v3）解析/校验/统计工具

真源文档：../02_原始震动采集器/02_采集软件/数据格式规范.md
机器可读定义：../02_原始震动采集器/02_采集软件/src/vib_format.h

职责：
  1) 结构校验：magic / version / 尺寸 / 文件头与每个块的 CRC
  2) 完整性判定：block_index 缺号（主机丢块）、FIFO_LOST_PKT 增长（器件丢包）、
     包头 flag（FIFO 满 / SPI 读错）、削顶与无效样本比例。
     短读（SHORT_READ = records < watermark）在生产路径只表示「正常排空到一个
     不足水位的批」，是信息级 notice，不是数据丢失证据（真实传输失败另有
     SPI_ERR 路径；器件丢包看 FIFO_LOST_PKT，主机丢块看 block_index）。
  3) 时间基准：tmst 16-bit raw ticks 回绕展开、×32/30 换算物理 µs、
     「总 ticks/总时间」**标称时基换算速率**（芯片内部标称时基，非独立绝对实测；
     stats.timebase 显式标注未做绝对时钟校准），不用 1e6/median。**任何断裂**（重复 Δ==0、
     大间隔 Δ≥4096、单样本缺口 = Δ∈[1,4096) 但超出正常窗口 [115,120]（如 234/236）、
     FIFO header 结构跳过、无效哨兵样本）都把时间轴切开为多个连续段并标记不可信：
     - 全程 measured_odr_hz 置 None（碎段首尾强拼的 rate 不是可信值），仅留
       measured_odr_hz_diagnostic 诊断项；逐段速率见 timeline.segments[*]
     - 断裂处的 Δ 不计入任何段的速率，跨断裂的物理时间不可比（sync 只允许单段拟合）
  4) 物理量：按 ±16 g / ±2000 dps 换算（手册 p.11/p.12），给出各轴 RMS/峰值；
     温度按 p.25/66 公式 T[℃] = TEMP_DATA/2.07 + 25，TEMP_DATA 是**有符号 8 位**
     （二补码：≥128 减 256）。本工具不据温度值做任何失效判定——手册 p.25/66
     未给出可依赖的 8 位温度「无效」编码
  5) 导出：--csv 写逐样本物理量（流式，内存恒定）

退出码：0=无异常；2=检出异常（CRC/缺号/丢包/越窗/削顶超阈/时间轴断裂/短读以外的
真实 SPI 错/未结束）；1=用法或 IO 错误。短读只是一条 notice（见 stats.notices），
不单独把健康采集判成异常。
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
INVALID_SENTINEL = -32768       # 器件「无效样本」哨兵 = 0x8000（INT16_MIN）。
                                # -32766（0x8002）是合法满量程替换值，不是无效。
                                # 注：本解析器以记录内 rec_flags 的 REC_FLAG_INVALID 为准，
                                # 该标志由 C 端 icm_parse_fifo 按此哨兵设置。
TMST_WRAP = 65536               # 16-bit raw ticks 回绕
TMST_SCALE_NUM = 32             # raw ticks → µs 分子（Rev 1.6 §12.7）
TMST_SCALE_DEN = 30             # raw ticks → µs 分母
TMST_US_NOMINAL = 125           # 8 kHz 名义物理 Δ（µs，换算后）
# 8 kHz 相邻 raw Δ 的「正常窗口」——**现选用的保守诊断门限**，不是时钟精度指标：
# 名义换算得 raw ≈ 117.1875（物理 125 µs ⇒ 整数差 117/118 交替），实机相邻 raw Δ
# 目前只见 117/118。窗口 [115,120] 只是本工具**当前选定**的保守诊断门限，用于区分
# 「名义附近的量化/抖动」与「漏样本」；integer fixture 用名义附近的整数偏移
# （Δ=116、Δ=119 这类名义附近整数偏移）覆盖测试，全部落在本窗口内（见 tools/tests/assert_tmst_tolerance.py）。
# ⚠️ **不得由本窗口反推时钟容差**：绝对钟差（例如 TDK Rev1.6 表 4 给出 CLK_SEL=01 且
# gyro active 的初始容差 ±1.5%，及 p.24 §4.10 另述的 ±1%——两者条件不同）**不能**
# 直接等同**同一芯片内相邻 raw Δ 的变化**；绝对时钟误差只能靠**外部参考**（本工具的
# host_rate_hz 等独立速率）判定，本窗口不构成任何时钟精度宣称。
# 本窗口只是「名义附近抖动 vs 漏样本」的门限，**不构成无丢包保证**
# （丢包须由 FIFO_LOST_PKT / block_index / FIFO 满标志独立判定）。
# 漏 1 个样本 ⇒ Δ ≈ 232–237，仍在窗口外（上界必须远小于 232）。
# 必须与 C 端 vib_format.h 的 VIB_TMST_RAW_LO/HI 一致；按 8 kHz 标定。
TMST_RAW_LO = 115
TMST_RAW_HI = 120
# 温度（手册 p.25/p.66）：TEMP_DATA 有符号 8 位，T[℃] = TEMP_DATA/2.07 + 25。
# 盘上是 uint8 原始字节（v3 布局/CRC 不变），解析时才按二补码还原符号。
# 不定义、也不使用任何温度「无效」哨兵常量（无手册/固件级证据）。
TEMP_LSB_PER_C = 2.07
TEMP_OFFSET_C = 25.0

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


def temp_raw_to_c(temp_raw: int) -> float:
    """盘上 uint8 温度字节 → 物理 ℃（手册 p.25/p.66）。

    v3 记录的 temp 字段是**原始字节**（布局/CRC 不变）；物理温度按**有符号 8 位**
    二补码解释：≥128 减 256（如 0xFF → −1 → 24.5169 ℃，0xF9 → −7 → 21.6184 ℃），
    再套 T = TEMP_DATA/2.07 + 25。CSV 与统计共用本函数，避免两条路径口径分叉。
    """
    return ((temp_raw - 256) if temp_raw >= 128 else temp_raw) / TEMP_LSB_PER_C \
        + TEMP_OFFSET_C


class BinReader:
    def __init__(self, path: str, max_records: int = 0, csv_path: str = None,
                 t_map: tuple = None, csv_segment_id: int = None) -> None:
        self.path = path
        self.max_records = max_records
        self.csv_path = csv_path
        # t_map = (slope, intercept)：给 CSV 追加 t_px4_us = slope*t_us + intercept
        # （供 sync_bin_ulog 在有效 fit 后导出映射过的样本时刻；无 fit 时保持 None）。
        self.t_map = t_map
        self.csv_segment_id = csv_segment_id
        self.fh = {}
        self.errors = []          # 硬错误（结构性）
        self.anomalies = []       # 需要人工判断的异常
        self.notices = []         # 信息级提示（正常但值得记录，不影响 ok/退出码）
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
        self.tmst_dup = 0          # 重复时间戳次数（异常，不得静默吞掉）
        self.tmst_gap = 0          # 单样本缺口：[1,4096) 但超出正常窗口（如 234/236）
        self.tmst_skips = 0        # 结构跳过（FIFO header msg，不是样本）导致的断裂
        self.tmst_invalid_breaks = 0  # 无效哨兵记录导致的断裂（其时间戳不作为可信锚点）
        self.tmst_breaks = 0       # 时间轴断裂次数（dup / big / gap / skip / invalid）
        self.current_segment = 0
        self.segments = []         # 连续段（含逐段 raw ticks / 速率 / 断前类型）
        self.seg_first_us = 0.0
        self.seg_last_us = 0.0
        self.seg_samples = 0
        self.seg_raw_ticks = 0     # 本段内累计的 raw Δ（**不含**造成断裂的那个 Δ）
        self.seg_n_deltas = 0
        self.pending_break_kind = None  # 下一段的 break_before（gap/dup/big/skip/invalid）
        self.tmst_out_of_window = 0
        self.tmst_min = None
        self.tmst_max = None
        self.prev_tmst = None
        self.tmst_raw_total = 0        # Σ 展开后的 raw Δ（整数，规范真源）
        self.tmst_n_deltas = 0         # 参与速率估计的 Δ 数（d ∈ [1,4096)）
        self.tmst_abs_us = 0.0         # 展开后的物理绝对时间轴（µs，由整数 raw 派生，供 CSV）
        self.ticks_to_us_num = TMST_SCALE_NUM
        self.ticks_to_us_den = TMST_SCALE_DEN
        self.first_t_read_ns = None    # 首个数据块的 t_read（主机采集窗口起点）
        self.last_t_read_ns = None     # 末个数据块的 t_read（主机采集窗口终点）
        self.first_block_valid_records = None  # 首个数据块的有效记录数（主机采样率要扣除）
        self.end_block_seen = False
        self.axes = {a: RunningStat() for a in AXES}
        self.temp_c = RunningStat()

    @staticmethod
    def _segment_rate_hz(raw_ticks: int, n_deltas: int, num: int, den: int):
        """段内速率 = n_deltas / (raw_ticks × num/den × 1e-6)；无从估计时 None。

        **只在单个连续段内**用「总 raw ticks / 总时间」，不把碎段首尾强拼成
        看似可信的全程 rate（断裂处的 Δ 不计入任何段）。"""
        if n_deltas <= 0 or raw_ticks <= 0:
            return None
        total_us = raw_ticks * num / den
        if total_us <= 0:
            return None
        return round(n_deltas * 1e6 / total_us, 3)

    def _finalize_segment(self, break_kind=None):
        """关闭当前连续段，并把 break_kind 记为下一段的断前类型（不跨接）。"""
        if self.seg_samples > 0:
            self.segments.append({
                "segment_id": self.current_segment,
                "first_us": round(self.seg_first_us, 3),
                "last_us": round(self.seg_last_us, 3),
                "samples": self.seg_samples,
                "raw_ticks": self.seg_raw_ticks,
                "n_deltas": self.seg_n_deltas,
                # 段内速率仅由本段 Δ 得出；单段内的样本时刻可信
                "measured_odr_hz": self._segment_rate_hz(
                    self.seg_raw_ticks, self.seg_n_deltas,
                    self.ticks_to_us_num, self.ticks_to_us_den),
                "break_before": self.pending_break_kind,
                "trusted": True,
            })
            self.current_segment += 1
            self.seg_samples = 0
            self.seg_raw_ticks = 0
            self.seg_n_deltas = 0
            self.seg_first_us = 0.0
            self.seg_last_us = 0.0
        self.pending_break_kind = break_kind

    def _break_segment(self, kind):
        """时间轴断裂：关闭当前段、计数，并让下一段以 kind 开头。

        断裂处**绝不跨接**：造成断裂的那个 Δ（gap）只留在全程诊断时间里，不计入
        任何段的速率；skip/invalid 连 Δ 都不再计算（prev_tmst 由调用方清零）。"""
        self.tmst_breaks += 1
        self._finalize_segment(kind)

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

        # ---- 不支持的时钟源/分辨率/元数据：明确拒绝，不当可信物理时间 ----
        if ver >= 3:
            if tmst_source != 0:
                self.errors.append(
                    "文件头 tmst_source=%d 不受支持（本实现仅支持内部时钟 0；CLKIN=1 的时间"
                    "解释不同，予以拒绝）" % tmst_source)
            if tmst_res != 0:
                self.errors.append(
                    "文件头 tmst_res=%d 不受支持（本实现仅支持 RES=0 名义 1 µs；RES=1 的 "
                    "16 µs 分辨率时间解释不同，予以拒绝）" % tmst_res)
            if count_unit not in (0, 1, 16):
                self.errors.append("文件头 count_unit=%d 非法（应为 0/1/16）" % count_unit)
            if count_endian not in (0, 1):
                self.errors.append("文件头 count_endian=%d 非法（应为 0/1）" % count_endian)

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

    def _timebase_labels(self) -> tuple:
        """(source, kind)：时基元数据的**来源标签**——只认 v3 的显式元数据。

        - v3 且换算 scale 元数据齐全：tmst_source==0 → "internal"（内部标称时基），
          非 0（CLKIN）→ "clkin"（随后仍按原样被拒绝）；kind 一律 "chip_nominal_scale"。
        - legacy v2 / 换算元数据缺失：文件头 64..75 处当年是 reserved，那里的字节
          （常为 0，也可能是任意残留值）**不是**时钟源元数据。此时一律
          source="unknown"、kind="legacy_unknown"：**绝不用 tmst_source 的取值去标
          clkin/internal**（否则 v2 的 reserved 残留字节会被冒充成外部钟），也不让
          「raw=µs 旧假设」冒充 "chip_nominal_scale"（标称标定）。
        """
        fh = self.fh
        has_meta = (fh.get("format_version", 0) >= 3
                    and fh.get("tmst_scale_num", 0) > 0
                    and fh.get("tmst_scale_den", 0) > 0)
        if not has_meta or fh.get("tmst_source") is None:
            return "unknown", "legacy_unknown"
        source = "internal" if fh.get("tmst_source") == 0 else "clkin"
        return source, "chip_nominal_scale"

    # ------------------------------------------------------------------ 主循环
    def run(self) -> dict:
        if os.path.getsize(self.path) < FILE_HEADER_SIZE:
            raise VibFormatError("文件比文件头还短")

        csv_f = None
        if self.csv_path:
            csv_f = open(self.csv_path, "w", encoding="utf-8", newline="")
            if self.t_map is not None:
                csv_f.write("index,segment,t_us,t_px4_us," + ",".join(AXES) + ",temp_c,rec_flags\n")
            else:
                csv_f.write("index,segment,t_us," + ",".join(AXES) + ",temp_c,rec_flags\n")

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

                # 主机采集窗口：用数据块的 t_read（批量读完成时刻，非采样时刻）
                if btype == BLOCK_TYPE_DATA:
                    if self.first_t_read_ns is None:
                        self.first_t_read_ns = t_read
                    self.last_t_read_ns = t_read

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
                valid_here = 0

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
                        # 结构跳过：FIFO header 消息不是样本、也不是时间锚点。
                        # 显式断段并清掉链尾，绝不让 Δ 从断裂一侧算到另一侧。
                        self.bad_records += 1
                        self.tmst_skips += 1
                        self._break_segment("skip")
                        self.prev_tmst = None
                        continue

                    valid_here += 1
                    invalid_sample = bool(rflags & REC_FLAG_INVALID)

                    if invalid_sample:
                        # 无效哨兵记录（0x8000）：其时间戳不作为可信锚点 → 断段、不跨接。
                        # 口径说明：Python 侧比 C 侧保守（C 仍把该记录计入 Δ 链），
                        # 代价是带无效样本的文件被标记为断续/不可信，而不是静默健康。
                        self.tmst_invalid_breaks += 1
                        self._break_segment("invalid")
                        self.prev_tmst = None
                    elif self.prev_tmst is not None:
                        # 时间戳展开：16-bit raw ticks 回绕；物理 µs = raw * 32/30。
                        # 先整数累计 raw 再派生时间（不逐样本浮点累加，避免漂移）。
                        d = tmst - self.prev_tmst
                        if d < 0:
                            d += TMST_WRAP
                        if d == 0:
                            # 重复时间戳：异常，时间轴断裂，显式标记（不得静默吞掉）
                            self.tmst_dup += 1
                            self._break_segment("dup")
                        elif d >= 4096:
                            self.tmst_big += 1
                            self._break_segment("big")
                        else:
                            # 全程诊断时间：Δ 一律累计（含 gap），与 C 的 sum_raw_ticks 一致
                            self.tmst_raw_total += d
                            self.tmst_n_deltas += 1
                            self.tmst_hist[d] += 1
                            if self.tmst_min is None or d < self.tmst_min:
                                self.tmst_min = d
                            if self.tmst_max is None or d > self.tmst_max:
                                self.tmst_max = d
                            # 整数派生：t_us = ΣrawΔ * num/den（不是 += d*num/den 浮点累加）
                            self.tmst_abs_us = (
                                self.tmst_raw_total * self.ticks_to_us_num
                                / self.ticks_to_us_den)
                            if d < TMST_RAW_LO or d > TMST_RAW_HI:
                                # 单样本缺口（234/236 raw ticks 等）：时间轴断续。
                                # 与 C 侧 tmst.c 一致：out_of_window + gap + breaks。
                                self.tmst_out_of_window += 1
                                self.tmst_gap += 1
                                self._break_segment("gap")   # 该 Δ 不计入任何段的速率
                            else:
                                self.seg_raw_ticks += d
                                self.seg_n_deltas += 1

                    if not invalid_sample:
                        self.prev_tmst = tmst
                        if self.seg_samples == 0:
                            self.seg_first_us = self.tmst_abs_us
                        self.seg_last_us = self.tmst_abs_us
                        self.seg_samples += 1

                    # 物理量
                    self.axes["ax"].push(ax / LSB_PER_G)
                    self.axes["ay"].push(ay / LSB_PER_G)
                    self.axes["az"].push(az / LSB_PER_G)
                    self.axes["gx"].push(gx / LSB_PER_DPS)
                    self.axes["gy"].push(gy / LSB_PER_DPS)
                    self.axes["gz"].push(gz / LSB_PER_DPS)
                    temp_c = temp_raw_to_c(temp_raw)
                    self.temp_c.push(temp_c)

                    if csv_f and (self.csv_segment_id is None
                                  or self.current_segment == self.csv_segment_id):
                        if self.t_map is not None:
                            t_px4 = self.t_map[0] * self.tmst_abs_us + self.t_map[1]
                            csv_f.write("%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%d\n" % (
                                idx, self.current_segment, self.tmst_abs_us, t_px4,
                                ax / LSB_PER_G, ay / LSB_PER_G, az / LSB_PER_G,
                                gx / LSB_PER_DPS, gy / LSB_PER_DPS, gz / LSB_PER_DPS,
                                temp_c, rflags))
                        else:
                            csv_f.write("%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%d\n" % (
                                idx, self.current_segment, self.tmst_abs_us,
                                ax / LSB_PER_G, ay / LSB_PER_G, az / LSB_PER_G,
                                gx / LSB_PER_DPS, gy / LSB_PER_DPS, gz / LSB_PER_DPS,
                                temp_c, rflags))
                    idx += 1
                    self.records += 1

                if self.first_block_valid_records is None:
                    self.first_block_valid_records = valid_here
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
            # 短读（SHORT_READ）：生产 drain 只要读到 records < 水位就置该 flag，正常
            # 收尾的最后一批（以及 3 ms 兜底排空的不足水位批）都会命中，本身**不含**
            # 丢包信息（器件丢包看 FIFO_LOST_PKT，主机丢块看 block_index，传输失败看
            # SPI_ERR）。故如实保留计数，但只作信息级 notice，不单独判异常。
            self.notices.append(
                f"{self.short_blocks} 个块为短读（records < 水位；生产路径的正常不足"
                "水位排空，非数据丢失证据，另见 FIFO_LOST_PKT/缺块/SPI 错）")
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
            self.anomalies.append(f"{self.tmst_big} 处 Δtmst ≥ 4096 raw ticks"
                                  "（记录时间戳字节序或 ODR 配置可疑）")
        if self.tmst_dup:
            self.anomalies.append(f"{self.tmst_dup} 处重复时间戳（Δtmst==0）——"
                                  "时间轴断裂，不可信")
        if self.tmst_gap:
            self.anomalies.append(
                f"{self.tmst_gap} 处单样本缺口（raw Δ 在 [1,4096) 但超出正常窗口 "
                f"[{TMST_RAW_LO},{TMST_RAW_HI}]，如 234/236）——时间轴断续、不可信，"
                "绝不当全程健康")
        if self.tmst_skips:
            self.anomalies.append(
                f"{self.tmst_skips} 处 FIFO header 结构跳过（不是样本）——"
                "时间轴在此断裂，不跨接补 Δ")
        if self.tmst_invalid_breaks:
            self.anomalies.append(
                f"{self.tmst_invalid_breaks} 处无效哨兵样本（0x8000）——"
                "其时间戳不作为可信锚点，时间轴在此断裂")
        if self.tmst_breaks:
            self.anomalies.append(
                f"{self.tmst_breaks} 处时间轴断裂（重复/大间隔/单样本缺口/结构跳过/无效样本），"
                f"共 {len(self.segments) + 1 if self.seg_samples > 0 else len(self.segments)} "
                "个连续段：跨段物理时间不可比，全程 rate 不可当可信值")

        # 关闭最后一个连续段
        self._finalize_segment()

        # 空采集：绝不能报告为健康（对应启动 flush/baseline 失败后 writer_abort 的截断文件）
        if self.records == 0:
            self.errors.append("空采集：文件无任何数据记录（0 条）——中止/失败采集，不是健康采集")

        self._finalize(device_lost, clip_ratio)
        return self.stats

    # ------------------------------------------------------------------ 汇总
    def _finalize(self, device_lost: int, clip_ratio: float) -> None:
        # 时基换算速率用「总 raw ticks / 总物理时间」估计，不用 1e6/median
        # （对 117/118 抖动更稳健，且保留全程信息；与 C 端 tmst.c 一致）。
        # ⚠️ 这是**芯片内部标称时基的换算速率**（raw*32/30），不是独立绝对实测：
        # 未做绝对时钟校准，独立参考量是 host_rate_hz（数据块 t_read 窗口）。
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

        # 时间轴是否可信：任何断裂（重复/大间隔/单样本缺口/结构跳过/无效样本）即为不可信。
        # 不可信时**不**给出「全程 rate」——碎段首尾强拼出来的 rate 不是可信值；
        # 该值只作为诊断量另行命名上报，逐段速率见 timeline.segments[*].measured_odr_hz。
        tmst_untrusted = bool(self.tmst_breaks)

        # 独立主机采样率：按数据块 t_read（读时刻，非采样时刻）定义的采集窗口，
        # 不含 writer 关闭耗时；分母用有效记录（records − invalid），并扣除首批
        # （首读前已生成的样本）。边界不确定度 = FIFO 满窗 16 ms。
        host_window_s = None
        host_rate_hz = None
        records_valid = self.records - self.invalid_flag_records
        first_valid = self.first_block_valid_records or 0
        if (self.first_t_read_ns is not None and self.last_t_read_ns is not None
                and self.last_t_read_ns > self.first_t_read_ns):
            host_window_s = (self.last_t_read_ns - self.first_t_read_ns) / 1e9
            if host_window_s > 0:
                in_window = records_valid - first_valid
                if in_window < 0:
                    in_window = 0
                host_rate_hz = in_window / host_window_s

        # 时基来源标签：只由 v3 的显式元数据得出；缺元数据 → source=unknown /
        # kind=legacy_unknown（不按 tmst_source 取值去标 internal/clkin）
        tb_source, tb_kind = self._timebase_labels()

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
                "records_valid": records_valid,
                "first_block_valid_records": self.first_block_valid_records,
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
                # 仅在时间轴可信（单连续段）时给出全程 rate；否则为 None。
                "measured_odr_hz": (round(measured_odr, 3)
                                    if measured_odr and not tmst_untrusted else None),
                # 不可信时保留「碎段首尾强拼」的诊断值，但显式命名，不得当成可信 rate
                "measured_odr_hz_diagnostic": (round(measured_odr, 3)
                                               if measured_odr and tmst_untrusted else None),
                "rate_trusted": not tmst_untrusted,
                "nominal_odr_hz": self.fh.get("odr_hz_nominal"),
                "odr_error_ppm": (round((measured_odr - self.fh["odr_hz_nominal"])
                                        / self.fh["odr_hz_nominal"] * 1e6, 1)
                                  if measured_odr and not tmst_untrusted
                                  and self.fh.get("odr_hz_nominal") else None),
                "tmst_raw_min": self.tmst_min,
                "tmst_raw_max": self.tmst_max,
                "tmst_raw_total_ticks": self.tmst_raw_total,
                "tmst_out_of_window": self.tmst_out_of_window,
                "tmst_out_of_window_ratio": (
                    round(self.tmst_out_of_window / n_delta, 8) if n_delta else 0.0),
                "tmst_big_delta": self.tmst_big,
                "tmst_dup": self.tmst_dup,
                "tmst_gap": self.tmst_gap,
                "tmst_skips": self.tmst_skips,
                "tmst_invalid_breaks": self.tmst_invalid_breaks,
                "tmst_breaks": self.tmst_breaks,
                "tmst_untrusted": tmst_untrusted,
                "host_rate_hz": round(host_rate_hz, 3) if host_rate_hz else None,
                "host_window_s": round(host_window_s, 6) if host_window_s else None,
                "host_rate_boundary_uncertainty_us": 16000,
                # 机器可读时基元数据：显式声明 measured_odr_hz 的**来源与校准状态**，
                # 避免把「芯片标称时基换算」读成「独立绝对实测 ODR」。字段名保持兼容，
                # 但语义以本块为准。source/kind 只认 v3 的显式元数据：legacy v2（或换算
                # 元数据缺失）一律 source="unknown"/kind="legacy_unknown"（reserved 字节
                # 非 0 也不据此标 clkin），绝不让它冒充 internal/clkin 或
                # chip_nominal_scale（见 _timebase_labels）。
                "timebase": {
                    "kind": tb_kind,
                    "source": tb_source,
                    "metadata_present": tb_kind != "legacy_unknown",
                    "tmst_res": self.fh.get("tmst_res"),
                    "conversion": "raw_ticks*%d/%d"
                                  % (self.ticks_to_us_num, self.ticks_to_us_den),
                    "raw_delta_window": ([TMST_RAW_LO, TMST_RAW_HI]
                                         if tb_kind != "legacy_unknown" else None),
                    "raw_delta_window_note":
                        "现选用的保守诊断门限（名义附近抖动 vs 漏样本），非时钟精度"
                        "指标；绝对钟差不得由本窗口反推、须另用外部参考，且不得据此"
                        "声明绝对 8000 Hz 已校准",
                    "absolute_clock_calibrated": False,
                    "reference_independent_rate": "host_rate_hz",
                    "independent_rate_hz": (round(host_rate_hz, 3) if host_rate_hz else None),
                },
            },
            "timeline": {
                "segments": self.segments,
                "n_segments": len(self.segments),
                "tmst_breaks": self.tmst_breaks,
                "untrusted": tmst_untrusted,
            },
            "physical": {a: self.axes[a].as_dict() for a in AXES},
            "temp": self.temp_c.as_dict(),
            "errors": self.errors,
            "anomalies": self.anomalies,
            "notices": self.notices,
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
        rate_txt = (
            f"{tm['measured_odr_hz']} Hz (与标称偏差 {tm['odr_error_ppm']} ppm)"
            if tm["rate_trusted"] else
            f"全程不可信（断裂 {tm['tmst_breaks']} 处；碎段强拼诊断值 "
            f"{tm['measured_odr_hz_diagnostic']} Hz 仅供参考，看逐段 rate）")
        p(f"记录数    : {ig['records']}   块数: {ig['blocks']}   "
          f"时长: {tm['duration_s']:.3f} s   时基换算速率: {rate_txt}")
        tb = tm.get("timebase") or {}
        if tb.get("kind") == "chip_nominal_scale" and tb.get("source") == "internal":
            tb_txt = (f"芯片内部标称时基换算（{tb.get('conversion', 'raw*32/30')}，"
                      f"源=internal）")
        elif tb.get("kind") == "legacy_unknown":
            tb_txt = (f"无换算元数据/legacy（kind=legacy_unknown，source=unknown；"
                      f"换算 {tb.get('conversion')} 仅是旧假设）—— 物理时间轴不可信")
        else:
            tb_txt = (f"{tb.get('kind')}（{tb.get('conversion')}，"
                      f"源={tb.get('source')}，本实现不支持该时钟源）")
        p(f"时基口径  : {tb_txt} —— **未做绝对时钟校准**，"
          f"不得声明绝对 8000 Hz 已校准；绝对钟差须另用外部参考；独立参考速率 host_rate="
          f"{tb.get('independent_rate_hz')} Hz")
        p(f"时间轴    : {'连续' if not tm['tmst_untrusted'] else '断续/不可信'}   "
          f"连续段 {s['timeline']['n_segments']} 个   "
          f"缺口(单样本) {tm['tmst_gap']}  重复 {tm['tmst_dup']}  "
          f"大间隔 {tm['tmst_big_delta']}  结构跳过 {tm['tmst_skips']}  "
          f"无效样本断裂 {tm['tmst_invalid_breaks']}")
        if tm["tmst_untrusted"]:
            for sg in s["timeline"]["segments"]:
                p(f"    段{sg['segment_id']}: [{sg['first_us']:.0f}, {sg['last_us']:.0f}] µs  "
                  f"样本 {sg['samples']}  rawΔ {sg['raw_ticks']}  "
                  f"段内ODR {sg['measured_odr_hz']} Hz  断前 {sg['break_before']}")
        p(f"raw Δ     : 中位 {tm['tmst_median_raw']}（≈{tm['tmst_median_us']} µs）  范围 "
          f"[{tm['tmst_raw_min']}, {tm['tmst_raw_max']}]  "
          f"越窗 {tm['tmst_out_of_window']} ({tm['tmst_out_of_window_ratio']:.4%})  "
          f"离群(≥4096) {tm['tmst_big_delta']}")
        p(f"丢包      : 器件 FIFO_LOST_PKT {ig['device_lost_pkt_first']} → "
          f"{ig['device_lost_pkt_last']} (Δ{ig['device_lost_pkt_delta']})   "
          f"主机缺块 {ig['missing_blocks']}({ig['block_index_gaps']} 处)   "
          f"FIFO满块 {ig['fifo_full_blocks']}  短读块 {ig['short_blocks']}（信息级）")
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
        if s.get("notices"):
            p("提示（信息级，不影响判定）：")
            for e in s["notices"]:
                p(f"  · {e}")
        if s["ok"]:
            p("结论：结构与完整性校验通过 ✅（是否可用于分析仍取决于台架/飞行口径）")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="原始震动数据（格式 v3）解析/校验/统计",
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
            # odr= 仍是实测字段名（兼容既有脚本），但口径是**芯片标称时基换算速率**
            # （timebase.absolute_clock_calibrated=False）；host= 才是独立参考速率。
            print(f"{path}: records={st['integrity']['records']} "
                  f"odr={st['timing']['measured_odr_hz']} "
                  f"host={st['timing']['host_rate_hz']} "
                  f"lost={st['integrity']['device_lost_pkt_delta']} "
                  f"gaps={st['integrity']['missing_blocks']} "
                  f"short={st['integrity']['short_blocks']} "
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
