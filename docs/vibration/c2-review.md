# C2 配置工具和真实日志链路验收

2026-10-12。配置与离线验收工具位于 Tools/vibration，操作步骤见 [operation-guide.md](operation-guide.md)。它们运行在电脑上，不进入飞控固件。

## 工具验证

WSL Python 3.14 与现有 pyulog 环境运行 63 项测试：61 项通过，2 项因当前已安装 pyulog 而跳过“不存在 pyulog”的互斥环境分支。标准 pyulog 的六项读取和值交叉验证全部实跑通过；真实 H1 log_101 的结构与字段验证也已运行。

验收中修复了合成文件 topic 名称终结符、定义区与数据区顺序、PX4 字段排序及尾部 padding 约定，以及 pyulog 展开的数组字段。时间戳转换为有符号 Python 整数后再相减，防止 numpy uint64 在负延迟场景回绕。JSON 中的 numpy 数字保持数字类型。没有修改第三方 pyulog 或跳过失败用例。

工具按设备变化切段，按逐条 full/file/active 状态判断缓冲，保留当前构建的实例上限选择；校验真实 param show 输出、非法参数、外部清单替换及含歧义的输入。自动读取不会在 pyulog 解码失败后静默降级。没有数据、缺健康证据和警告均有明确输出；无 ERROR 不等同于完整验收通过。

## 真实 uORB 到 ULog 集成

新增主机专用 functional-LoggerFifoIntegration 测试，受 BUILD_TESTING 门控，不进入 X7 固件。它在独立临时目录初始化主机计时设施、发布加速度与陀螺各三个实例，启动实际 logger，按正常捕获路径记录，正常停止并等待写线程完成。ASSERT 失败路径有 stop 守卫。

本轮运行参数为 profile 768、acc mask 2、gyro mask 5。真实日志只包含 accel 实例 1、gyro 实例 0 和 2，未选中实例没有进入日志。每路 301 包、9,632 个有效样本：包含启动前的一包预热，再发布 300 批。独立 pyulog 检查核验了每包全部 96 个 x/y/z 数值、device_id、samples、dt、scale 与时间戳，以及六条 logger_status 记录。ULog 没有 dropout 标记；该次测试序列没有观察到标记跳跃。

ULog 为 207,948 B，SHA256：dad147e3ed347709fb374cd24aebc3f4e43f1ee4350eb8c6cfe6f814243ee0c3。原始日志、预期 manifest、GTest XML、逐值验收和 CLI JSON 保存在 E:/07-Workspace_codex/output/px4-x7-baseline/2026-10-12/integration/。

CLI 对该人工数据报告 0 ERROR、4 WARNING。三条时间残差警告来自人工发布间隔约 8 ms，而 samples×dt 为 4 ms，属于本测试设定；另一条是没有运行 sensor/estimator，因此缺 vehicle_imu_status。它们没有被隐藏或改为“连续无丢样通过”。本测试证明选择与数值传递，不证明实机高负载连续性。

主机 harness 在测试初始化前产生 work queue 提示，非 root 主机提高 logger 优先级也返回权限错误。测试通过初始化实际 hrt/legacy 队列实现计时捕获，logger 正常关闭并 flush，没有强制 stop。这些主机结果不验证 NuttX 的实际调度优先级、X7 SD 长尾或飞行控制时序。

## 架构结论与平台边界

C1 独立代码复核未发现需修复的生产缺陷；架构复核为 WATCH，无架构阻断。新增订阅独立于控制订阅的读取状态，但共享 CPU、队列和写卡资源。logger_status 约 1 Hz、通常下一轮记录，不能作为每个数据包的精确故障时刻。

官方 px4_fmu-v6x_default 目标从 C1 源码构建通过，FLASH 1,924,600/1,966,080 B，静态 AXI_SRAM 100,732 B。这是通用平台编译兼容性证据；原 H1 定制目标尚未移植或重新构建本补丁。

X7 实机 CPU、动态内存、写卡长尾、完整传感器链路及飞行对照仍按操作指南执行。C1 正式固件和基线固件已分别归档，见 [c1-build.json](c1-build.json) 与 [baseline-build.json](baseline-build.json)。
