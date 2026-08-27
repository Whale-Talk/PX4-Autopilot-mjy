# _h1_diag — H1 飞控 MAVLink 串口诊断工具

零第三方依赖（仅 Python stdlib）。针对 H1 飞机（定制版 FMUv6x，STM32H743 + NuttX）
通过 USB CDC-ACM 的 MAVLink 链路做取证、读写参数、执行 nsh 命令。

## 为什么不用现成库

两个坑现成工具不会替你查：

1. **串口 termios 行规**。若端口处于 cooked 模式，`ICRNL` 会把 0x0D 改写成 0x0A、
   `OPOST|ONLCR` 会把发出的 0x0A 扩成 0x0D 0x0A、`IXON` 会因流中的 0x13 暂停本机输出
   （表现为"写通道阻塞"）、`ICANON` 会让 read 只在换行时返回（表现为"速率骤降"）。
   本工具**打开前先快照 termios 并报告危险标志位**，然后强制 raw，且**关闭时不恢复**
   （原状态可能就是坏的）。

2. **链路积压**。端口未被打开时遥测会堆积；新打开后先读到的是几分钟前的旧数据，
   此时请求的应答排在积压之后，表现为"无应答"。本工具用 `time_boot_ms` 相对壁钟的
   推进倍率判定是否已追上实时（>>1 = 还在排积压），交互前先排空。

消息线序 / 长度 / CRC extra 全部在运行时从
`build/<target>/mavlink/**/mavlink_msg_*.h` 解析（249 条消息），不手抄。

## 用法

```bash
cd /home/mjy/Workspace/飞控/PX4-Autopilot-v6x_mjy

# 抓包取证：termios 快照 + 速率 + 帧统计 + ASCII 提取 + 重启检测 + 写通道测试
python3 _h1_diag/mavtool.py probe --seconds 12 --save-raw /tmp/h1.bin

# 读 / 写 / 保存参数
python3 _h1_diag/mavtool.py get COM_RC_IN_MODE RC_INPUT_PROTO RC_MAP_THROTTLE
python3 _h1_diag/mavtool.py set COM_RC_IN_MODE 0 --type int
python3 _h1_diag/mavtool.py save

# NuttX nsh 命令（可传多条）
python3 _h1_diag/mavtool.py shell "rc_input status" "ver all"

# 实时监视 RC 口字节速率 —— 插拔/对频接收机时用
python3 _h1_diag/mavtool.py rcwatch

# 裸字节
python3 _h1_diag/mavtool.py hexdump --nbytes 512
```

默认 `--dev /dev/ttyACM0`，默认方言目录
`build/px4_fmu-v6x_seeex25d_default/mavlink`（用 `--mavlink-dir` 改）。

## 判读参考

| 观察 | 含义 |
|---|---|
| `probe` 显示 100% 字节归帧、散字节 ~0 | 链路健康 |
| `time_boot_ms` **回退** | 飞控真的在重启（boot loop） |
| `time_boot_ms` 推进 >> 1.0x | 在排积压，读到的是旧数据（不是故障） |
| `rcwatch` 稳定 ~2 B/s | RC 线静默，仅 SBUS 扫描每 500ms 重配 UART 的伪影 → 接收机无输出 |
| `rcwatch` 1700–3600 B/s | 正常 SBUS 数据量 |
| `rcwatch` 高速率但 `searching for signal` | 有信号但极性/波特/校验不符 |

## WSL 前置

```powershell
usbipd list                        # 确认板子 BUSID 与 Shared 状态
usbipd attach --wsl --busid 2-6
```

排查全过程与结论见知识库：
`MD知识库/嵌入式开发/PX4/H1飞控/PX4-H1飞控-SBUS无响应排查记录.md`
