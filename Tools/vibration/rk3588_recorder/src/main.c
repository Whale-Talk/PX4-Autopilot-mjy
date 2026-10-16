/* ============================================================================
 * main.c — 采集程序入口（香橙派 5 Pro + ICM-42688-P）
 *
 * 只做命令行解析，真正的采集流程在 run.c 的 vib_run()（与单元测试共用同一
 * 生产代码路径）。见 run.h / run.c。
 * ==========================================================================*/
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <getopt.h>
#include <time.h>
#include <errno.h>
#include <sys/stat.h>

#include "run.h"
#include "vib_format.h"

static void usage(const char *p)
{
    fprintf(stderr,
        "用法: %s [选项]\n"
        "  -d, --dev PATH        spidev 节点（默认 /dev/spidev4.0）\n"
        "  -s, --spi-hz HZ       SPI 时钟（默认 24000000，IMU 上限，见手册 p.1）\n"
        "      --spi-mode 0|3    SPI 模式（默认 0；手册 p.52/p.63 只允许 0 或 3）\n"
        "  -o, --out DIR         输出目录（默认 .）\n"
        "  -n, --name NAME       输出文件名前缀（默认 vib_<UTC 时间戳>）\n"
        "  -t, --seconds S       采集时长秒；0=直到 Ctrl-C（默认 0）\n"
        "  -c, --cpu N           采集线程绑核（默认 4；-1 不绑）\n"
        "  -p, --prio N          SCHED_FIFO 优先级（默认 80；0 不用 RT）\n"
        "      --fsync-ms N      fdatasync 周期（默认 5000；0 不主动 fsync）\n"
        "      --frame 0|1       0=SENSOR, 1=FRD 写入文件头（默认 0）\n"
        "      --no-selftest     跳过自检（不建议；批次须标注未过 G1）\n"
        "      --selftest-only   只自检不采集\n"
        "  -h, --help\n", p);
}

static const struct option long_opts[] = {
    { "dev",           required_argument, NULL, 'd' },
    { "spi-hz",        required_argument, NULL, 's' },
    { "spi-mode",      required_argument, NULL, 1000 },
    { "out",           required_argument, NULL, 'o' },
    { "name",          required_argument, NULL, 'n' },
    { "seconds",       required_argument, NULL, 't' },
    { "cpu",           required_argument, NULL, 'c' },
    { "prio",          required_argument, NULL, 'p' },
    { "fsync-ms",      required_argument, NULL, 1001 },
    { "frame",         required_argument, NULL, 1002 },
    { "no-selftest",   no_argument,       NULL, 1003 },
    { "selftest-only", no_argument,       NULL, 1004 },
    { "help",          no_argument,       NULL, 'h' },
    { NULL, 0, NULL, 0 }
};

static int ensure_dir(const char *d)
{
    struct stat sb;
    if (stat(d, &sb) == 0)
        return S_ISDIR(sb.st_mode) ? 0 : -1;
    if (mkdir(d, 0755) == 0)
        return 0;
    return (errno == EEXIST) ? 0 : -1;
}

int main(int argc, char **argv)
{
    vib_run_args_t rc;
    memset(&rc, 0, sizeof(rc));
    rc.dev         = "/dev/spidev4.0";
    rc.spi_hz      = 24000000;
    rc.spi_mode    = 0;
    rc.outdir      = ".";
    rc.seconds     = 0.0;
    rc.cpu         = 4;
    rc.prio        = 80;
    rc.fsync_ms    = 5000;
    rc.frame       = 0;
    rc.do_selftest = 1;

    int c;
    while ((c = getopt_long(argc, argv, "d:s:o:n:t:c:p:h", long_opts, NULL)) != -1) {
        switch (c) {
        case 'd': rc.dev = optarg; break;
        case 's': rc.spi_hz = (uint32_t)strtoul(optarg, NULL, 0); break;
        case 'o': rc.outdir = optarg; break;
        case 'n': snprintf(rc.name, sizeof(rc.name), "%s", optarg); break;
        case 't': rc.seconds = strtod(optarg, NULL); break;
        case 'c': rc.cpu = atoi(optarg); break;
        case 'p': rc.prio = atoi(optarg); break;
        case 1000: rc.spi_mode = (uint8_t)atoi(optarg); break;
        case 1001: rc.fsync_ms = (uint32_t)strtoul(optarg, NULL, 0); break;
        case 1002: rc.frame = atoi(optarg); break;
        case 1003: rc.do_selftest = 0; break;
        case 1004: rc.selftest_only = 1; break;
        case 'h': usage(argv[0]); return 0;
        default:  usage(argv[0]); return 1;
        }
    }
    if (rc.spi_mode != 0 && rc.spi_mode != 3) {
        fprintf(stderr, "SPI 模式只能是 0 或 3（手册 p.52/p.63）\n");
        return 1;
    }
    if (ensure_dir(rc.outdir) != 0) {
        fprintf(stderr, "输出目录不可用: %s\n", rc.outdir);
        return 1;
    }
    if (rc.name[0] == '\0') {
        const time_t now = time(NULL);
        struct tm tmv;
        gmtime_r(&now, &tmv);
        strftime(rc.name, sizeof(rc.name), "vib_%y%m%dT%H%M%SZ", &tmv);
    }

    /* banner 版本从 VIB_DRIVER_VERSION 派生（major<<16|minor<<8|patch），不再写死，
     * 避免与 vib_format.h 的驱动版本漂移。 */
    fprintf(stderr, "=== 原始震动采集 v%u.%u.%u（数据格式 v%d）===\n",
            (VIB_DRIVER_VERSION >> 16) & 0xFFu, (VIB_DRIVER_VERSION >> 8) & 0xFFu,
            VIB_DRIVER_VERSION & 0xFFu, VIB_FORMAT_VERSION);
    fprintf(stderr, "设备=%s  SPI=%u Hz  mode=%u  输出=%s/%s.bin\n",
            rc.dev, rc.spi_hz, rc.spi_mode, rc.outdir, rc.name);

    return vib_run(&rc);
}
