/* ============================================================================
 * util.h — 通用工具（时钟/延时/CRC/实时化）
 * 事实来源：无（纯工程实现）。CRC 变体见 vib_format.h 注释。
 * ==========================================================================*/
#ifndef VIB_UTIL_H
#define VIB_UTIL_H

#include <stdint.h>
#include <stddef.h>

/* 单调时钟 CLOCK_MONOTONIC，单位 ns。
 * 用途：SPI 读打点、读延迟/丢包校验。
 * 严禁当作采样时刻——采样时刻取记录内 tmst（见 数据格式规范.md §0）。 */
uint64_t mono_ns(void);

/* UTC 纳秒（CLOCK_REALTIME），仅用于跨设备粗对齐（文件头 t_start_utc_ns） */
uint64_t utc_ns(void);

/* 微秒级延时：<50 µs 忙等，否则 nanosleep */
void sleep_us(unsigned int us);

/* CRC-32/ISO-HDLC（= zlib.crc32 = Python zlib.crc32）
 * poly 0xEDB88320（反射），init 0xFFFFFFFF，refin/refout=true，xorout 0xFFFFFFFF。
 * 与 Python 侧校验脚本必须一致——解析器交叉验证靠这条。
 *
 * 增量接口：块头 CRC 需要覆盖「header[0:28) + payload」两段不连续内存，
 * 因此提供 begin/update/end；vib_crc32() 是单段便捷封装。 */
uint32_t vib_crc32_begin(void);
uint32_t vib_crc32_update(uint32_t crc, const void *buf, size_t len);
uint32_t vib_crc32_end(uint32_t crc);
uint32_t vib_crc32(const void *buf, size_t len);

/* 尽力实时化：mlockall + SCHED_FIFO + 绑核。失败不致命，返回 0 成功、-1 有失败项。
 * cpu < 0 表示不绑核。 */
int rt_setup(int cpu, int prio, int verbose);

#endif /* VIB_UTIL_H */
