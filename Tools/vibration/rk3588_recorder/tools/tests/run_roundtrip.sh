#!/usr/bin/env bash
# ============================================================================
# run_roundtrip.sh — 无硬件回归：C 写入路径 ↔ Python 解析路径 交叉验证
#
# 在 Linux（或 WSL）里跑，需要 gcc + python3。
#   用法: bash _tools/tests/run_roundtrip.sh
# 产物写在临时目录，不污染工作区。
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # .../rk3588_recorder/tools/tests
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"                      # .../rk3588_recorder
VIBDIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"                 # .../Tools/vibration（父目录，含 ulog_reader.py）
SRC="$ROOT/src"
TESTS="$ROOT/tools/tests"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

echo "工作区: $ROOT"
echo "振动工具目录: $VIBDIR"
echo "临时区: $OUT"

echo "--- 1) 严格编译采集程序 ---"
gcc -O2 -Wall -Wextra -Wshadow -Wconversion -Werror -std=gnu11 \
    -o "$OUT/vibrec" "$SRC/util.c" "$SRC/spi_dev.c" "$SRC/icm42688p.c" \
    "$SRC/writer.c" "$SRC/tmst.c" "$SRC/run.c" "$SRC/main.c" -lpthread -lrt
echo "  编译通过（-Werror）"

echo "--- 2) 编译合成数据生成器（复用真实 writer 路径）---"
gcc -O2 -Wall -Wextra -Wshadow -Wconversion -Werror -std=gnu11 -I"$SRC" \
    -o "$OUT/mk_synth" "$SRC/tests/mk_synth.c" "$SRC/writer.c" "$SRC/util.c" \
    -lpthread -lrt
echo "  编译通过（-Werror）"

echo "--- 3) 正常文件往返 ---"
"$OUT/mk_synth" "$OUT/clean.bin" 2
python3 "$TESTS/assert_roundtrip.py" "$OUT/clean.bin" 2

echo "--- 4) 注入 block_index 缺号 → 必须被检出 ---"
"$OUT/mk_synth" "$OUT/gap.bin" 1 --gap
python3 "$TESTS/assert_roundtrip.py" "$OUT/gap.bin" 1 --expect-gap

echo "--- 5) 注入 payload 坏字节 → 必须被 CRC 检出 ---"
"$OUT/mk_synth" "$OUT/bad.bin" 1 --badcrc
python3 "$TESTS/assert_roundtrip.py" "$OUT/bad.bin" 1 --expect-badcrc

echo "--- 6) 解析器在正常文件上的退出码必须为 0 ---"
set +e
python3 "$ROOT/tools/bin_reader.py" "$OUT/clean.bin" --quiet
RC=$?
set -e
if [ "$RC" -ne 0 ]; then echo "退出码应为 0，实际 $RC"; exit 1; fi
echo "  退出码 0 ✓"

echo "--- 7) 长度一致性：文件大小 == 256 + Σ(32+n*16) + 32 ---"
python3 - "$OUT/clean.bin" <<'PY'
import struct, sys, os
p = sys.argv[1]
sz = os.path.getsize(p)
off = 256
total = 0
with open(p, 'rb') as f:
    f.seek(256)
    nblocks = 0
    while True:
        b = f.read(32)
        if not b: break
        bt, n = struct.unpack_from('<HH', b)
        f.seek(n * 16, 1)
        total += 32 + n * 16
        nblocks += 1
        if bt == 0xFFFF: break
assert off + total == sz, (off, total, sz)
print(f"  文件 {sz} B = 256 + {total} B（{nblocks} 个块）✓")
PY

echo "--- 8) 时间戳换算纯函数测试（raw ticks ×32/30，总 ticks/总时间）---"
gcc -O2 -Wall -Wextra -Wshadow -Wconversion -Werror -std=gnu11 -I"$SRC" \
    -o "$OUT/tmst_test" "$SRC/tests/tmst_test.c" "$SRC/tmst.c"
"$OUT/tmst_test"

echo "--- 9) mock SPI 器件模型：自检 sub-full 不溢出 / INTF_CONFIG0 / 错误分支 ---"
gcc -O2 -Wall -Wextra -Wshadow -Wconversion -Werror -std=gnu11 -I"$SRC" \
    -o "$OUT/mock_icm" "$SRC/tests/mock_icm.c" "$SRC/icm42688p.c" \
    "$SRC/spi_dev.c" "$SRC/util.c" "$SRC/tmst.c" -lpthread -lrt
"$OUT/mock_icm"

echo "--- 10) 生产启动顺序确定性 mock（实际执行 run.c 的 vib_run，非独立模型）---"
gcc -O2 -Wall -Wextra -Wshadow -Wconversion -Werror -std=gnu11 -I"$SRC" \
    -o "$OUT/startup_test" "$SRC/tests/startup_test.c" "$SRC/util.c" \
    "$SRC/spi_dev.c" "$SRC/icm42688p.c" "$SRC/writer.c" "$SRC/tmst.c" "$SRC/run.c" \
    -Wl,--wrap=writer_submit -lpthread -lrt
"$OUT/startup_test"

echo "--- 11) 外置 bin ↔ X7 native FIFO 时间映射（offset+drift+残差）---"
PYTHONPATH="$ROOT/tools:$VIBDIR" python3 "$TESTS/sync_test.py"

echo "--- 12) 生产 writer 落盘 I/O 故障注入（链接期 --wrap）：首先成功写 data，再制造失败 ---"
gcc -O2 -Wall -Wextra -Wshadow -Wconversion -Werror -std=gnu11 -I"$SRC" \
    -Wl,--wrap=write -Wl,--wrap=fdatasync -Wl,--wrap=close \
    -o "$OUT/writer_fault" "$SRC/tests/writer_fault_test.c" "$SRC/writer.c" \
    "$SRC/util.c" -lpthread -lrt
echo "  编译通过（-Werror + --wrap）"
"$OUT/writer_fault" "$OUT/wfault"
python3 - "$ROOT/tools" "$OUT/wfault/must_reject" <<'PY'
import os, sys
sys.path.insert(0, sys.argv[1])
import bin_reader as br
d = sys.argv[2]
files = sorted(f for f in os.listdir(d) if f.endswith(".bin"))
assert files, "故障注入没有产物"
bad = [n for n in files
       if br.BinReader(os.path.join(d, n)).run()["ok"]]
print("  结构必坏产物 %d 个，解析器全部判为不 OK ✓" % len(files))
if bad:
    print("  ✗ 被误判为健康：%s" % bad)
    sys.exit(1)
PY

echo "--- 13) 时间戳单样本缺口(234 tick)：断段/不可信，sync 不得跨缺口拟合 ---"
PYTHONPATH="$ROOT/tools:$VIBDIR" python3 "$TESTS/assert_tmst_gap.py"

echo "--- 14) 映射导出仅包含已拟合段；允许误差不等于实机相位证据 ---"
PYTHONPATH="$ROOT/tools:$VIBDIR" python3 "$TESTS/mapped_segment_test.py"

echo "--- 15) 名义附近整数时基偏移（现选用的保守诊断门限）不得判缺口/不可信：C 侧反例（生产 tmst.c）---"
gcc -O2 -Wall -Wextra -Wshadow -Wconversion -Werror -std=gnu11 -I"$SRC" \
    -o "$OUT/tmst_tol" "$SRC/tests/tmst_tolerance_test.c" "$SRC/tmst.c"
"$OUT/tmst_tol"

echo "--- 16) 同上，解析器侧反例（真实 VIB3 字节 fixture + 独立复算）---"
PYTHONPATH="$ROOT/tools:$VIBDIR" python3 "$TESTS/assert_tmst_tolerance.py"

echo "--- 17) legacy v2（raw=µs 旧假设）必须被同步工具在读 ULog 前拒绝 ---"
PYTHONPATH="$ROOT/tools:$VIBDIR" python3 "$TESTS/assert_legacy_refused.py"

echo "--- 18) 独立字节 fixture：温度二补码 / 短读=notice / 真实错误仍失败 / 时基元数据 ---"
PYTHONPATH="$ROOT/tools:$VIBDIR" python3 "$TESTS/assert_hw_bytes.py"

echo
echo "ROUNDTRIP-ALL-OK ✅"
