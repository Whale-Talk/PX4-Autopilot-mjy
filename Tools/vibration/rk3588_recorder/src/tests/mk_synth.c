/* ============================================================================
 * tests/mk_synth.c — 合成数据生成器（无硬件回归用）
 *
 * 复用**真实的写入路径**（writer.c），因此它验证的是：
 *   文件头布局 / 块布局 / 记录字节序 / CRC 变体 / block_index 机制 / 结束块
 * 与 Python 解析器（_tools/bin_reader.py）是否严格一致。
 *
 * 合成规则（必须与 _tools/tests/assert_roundtrip.py 中的独立复算完全一致）：
 *   i = 全局记录序号（0 起）
 *   accel[0] = 32767 当 i % clip_every == 0，否则 1000
 *   accel[1] = -2000   accel[2] = 2048
 *   gyro[0]  = 164     gyro[1]  = 0     gyro[2] = -16
 *   temp_raw = 0xF9（存盘是**原始 uint8 字节**；物理温度须按有符号 8 位二补码解释：
 *              0xF9 → −7 → 21.6184 ℃，覆盖负温度分支；正值分支见 assert_hw_bytes.py）
 *              rec_flags = 0
 *   tmst_raw = (i * 125*30/32) & 0xFFFF  （严格 8 kHz：物理 125 µs/样本 →
 *               raw ticks 117/118 交替，经历多次 16 位回绕）
 *   每 32 条一数据块；lost_pkt_total = 0
 *
 * 用法: mk_synth OUT.bin SECONDS [--gap] [--badcrc] [--clip-every N]
 *   --gap       让 block_index 跳过 1 个数（模拟主机侧丢块）
 *   --badcrc    写完后翻转第一个数据块 payload 的 1 个字节
 * ==========================================================================*/
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>

#include "vib_format.h"
#include "writer.h"
#include "util.h"

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "用法: %s OUT.bin SECONDS [--gap] [--badcrc] [--clip-every N]\n",
                argv[0]);
        return 1;
    }
    const char *out = argv[1];
    const int seconds = atoi(argv[2]);
    int inject_gap = 0, inject_badcrc = 0, clip_every = 1000;
    for (int i = 3; i < argc; i++) {
        if (!strcmp(argv[i], "--gap"))            inject_gap = 1;
        else if (!strcmp(argv[i], "--badcrc"))    inject_badcrc = 1;
        else if (!strcmp(argv[i], "--clip-every")) clip_every = atoi(argv[++i]);
        else { fprintf(stderr, "未知参数 %s\n", argv[i]); return 1; }
    }
    if (seconds <= 0 || clip_every <= 0) {
        fprintf(stderr, "参数非法\n");
        return 1;
    }

    const uint32_t odr = 8000;
    const uint64_t nrec = (uint64_t)seconds * odr;

    vib_file_header_t fh;
    memset(&fh, 0, sizeof(fh));
    fh.t_start_monotonic_ns  = 123456789ull;
    fh.t_start_utc_ns        = 1750000000000000000ull;   /* 固定值 → 可复现 */
    fh.who_am_i              = 0x47;
    fh.frame                 = 0;
    fh.reg_config_hash       = 0xDEADBEEFu;
    fh.odr_hz                = odr;
    fh.fs_accel_g            = 16;
    fh.fs_gyro_dps           = 2000;
    fh.sample_count_expected = nrec;
    fh.tmst_scale_num        = VIB_TMST_SCALE_NUM;
    fh.tmst_scale_den        = VIB_TMST_SCALE_DEN;
    fh.tmst_source           = VIB_TMST_SOURCE_INTERNAL;
    fh.tmst_res              = VIB_TMST_RES_1US;
    fh.count_unit            = 1;      /* COUNT_REC=1（按记录） */
    fh.count_endian          = 1;      /* 大端 */

    writer_t w;
    if (writer_start(&w, out, &fh, 0, 0) != 0)
        return 1;

    uint64_t i = 0;
    uint32_t block_index = 0;
    int clipped_blocks = 0;

    while (i < nrec) {
        uint32_t n = (uint32_t)((nrec - i) < VIB_SAMPLES_PER_BLOCK
                                ? (nrec - i) : VIB_SAMPLES_PER_BLOCK);
        vib_record_t rec[VIB_SAMPLES_PER_BLOCK];
        uint32_t clip = 0;

        for (uint32_t k = 0; k < n; k++, i++) {
            vib_record_t r;
            memset(&r, 0, sizeof(r));
            r.accel[0] = (int16_t)((i % (uint64_t)clip_every == 0) ? 32767 : 1000);
            r.accel[1] = -2000;
            r.accel[2] = 2048;
            r.gyro[0]  = 164;
            r.gyro[1]  = 0;
            r.gyro[2]  = -16;
            r.temp_raw = 0xF9;         /* 原样存 uint8 字节；二补码 ⇒ −7 → 21.6184 ℃ */
            r.tmst_raw = (uint16_t)(((uint64_t)i * (uint64_t)VIB_TMST_US_NOMINAL
                                      * (uint64_t)VIB_TMST_SCALE_DEN
                                      / (uint64_t)VIB_TMST_SCALE_NUM) & 0xFFFFull);
            r.rec_flags = 0;
            for (int a = 0; a < 3; a++) {
                if (r.accel[a] >= VIB_CLIP_THRESHOLD || r.accel[a] <= -VIB_CLIP_THRESHOLD) {
                    clip++;
                    break;
                }
            }
            rec[k] = r;
        }
        if (clip)
            clipped_blocks++;

        vib_block_header_t bh;
        memset(&bh, 0, sizeof(bh));
        bh.block_type          = VIB_BLOCK_TYPE_DATA;
        bh.n_samples           = (uint16_t)n;
        bh.block_index         = block_index;
        bh.t_read_monotonic_ns = 123456789ull + i * 125000ull;
        bh.lost_pkt_total      = 0;
        bh.flags               = 0;
        bh.clip_count          = clip;

        if (writer_submit(&w, &bh, rec, (size_t)n * VIB_RECORD_SIZE) != 0) {
            fprintf(stderr, "submit 失败\n");
            return 1;
        }

        block_index++;
        if (inject_gap && block_index == 3)      /* 制造一次缺号 */
            block_index++;
    }

    writer_finish(&w, 0);
    if (writer_error(&w) != 0)
        return 1;

    if (inject_badcrc) {
        int fd = open(out, O_RDWR);
        if (fd < 0) {
            fprintf(stderr, "打开 %s 失败: %s\n", out, strerror(errno));
            return 1;
        }
        /* 第一个数据块 payload 的第 3 个字节（块头 32 B 之后） */
        off_t off = VIB_HEADER_SIZE + VIB_BLOCK_HEADER_SIZE + 2;
        uint8_t b = 0;
        if (pread(fd, &b, 1, off) != 1) {
            fprintf(stderr, "pread 失败\n");
            close(fd);
            return 1;
        }
        b ^= 0xFF;
        if (pwrite(fd, &b, 1, off) != 1) {
            fprintf(stderr, "pwrite 失败\n");
            close(fd);
            return 1;
        }
        close(fd);
    }

    printf("mk_synth: %s  records=%llu blocks=%u clipped_blocks=%d%s%s\n",
           out, (unsigned long long)nrec, block_index, clipped_blocks,
           inject_gap ? " [gap]" : "", inject_badcrc ? " [badcrc]" : "");
    return 0;
}
