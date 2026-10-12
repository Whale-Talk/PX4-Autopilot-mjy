# CUAV X7+ 原生 FIFO 振动日志实施计划

日期：2026-10-12。用户已批准第一版范围。先完成 X7+ 基线构建，再由 DSH 编码、Codex 验收；每阶段单独提交并同步组织和个人仓库。编译验收与实板、飞行验收分别记录。

## 基点和版本管理

- 官方稳定版 `v1.16.1`，源码提交 `94cb2012792b2ae89f0b147cfee53ee31ae550be`。依据：[官方发布记录](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.16.1)。
- 功能分支 `feature/cuav-x7-vibration-logging`。从官方标签建立，不以 H1 适配分支 `seeex25d_H1@f298aad` 为父提交。
- 工作区 `E:/07-Workspace_codex/px4-cuav-x7-vibration` 是原 `PX4-H1` 仓库的独立 worktree，共享提交和分支历史，原 H1 工作区的六个 Rockchip 暂存文件留在原索引。
- 组织仓库 `NexusAir-Technologies/PX4-H1`；个人仓库 `Whale-Talk/PX4-Autopilot-mjy`（经账号仓库列表和用户重新确认）。仅推送本次功能分支，不强推、合并或修改原分支。
- 初始化父仓库记录的固定子模块提交，禁止 `submodule update --remote`。初始化本身不更改 gitlink；依赖、环境和验证记录作为阶段提交内容。
- 每次只提交明确拥有的文件，不使用 `git add -A` 或 DSH 自动全量提交，不混入原六个文件。沿用现有提交身份。
- 远程凭据不进入日志、文档或 DSH 指令；使用正常 Git 连接和凭据管理。

## 第一版范围

通过原生 `sensor_accel_fifo`、`sensor_gyro_fifo` 保存指定 IMU 实例的全部已发布数据，保留正常飞行日志。普通 X7+ 是主对象，H1 的共享 logger 功能保持兼容。

不改 IMU ODR、硬件滤波、积分频率、`IMU_GYRO_RATEMAX`、控制参数、传感器选择、写卡线程优先级、FIFO 消息结构和调度策略。FFT/PSD 在电脑端运行。主分析 1–500 Hz，500–1000 Hz 作为受传感器滤波和测量链路限制的探索。

X7+ 构建目标 `cuav_x7pro_default`。目标名含 pro 不构成产品型号识别证据；实际 IMU 实例必须对应真实 `device_id`。首轮选择一颗 ICM20689 的加速度和陀螺，地面资源验收通过后，再增加 ICM42688P 加速度对照。

## 参数与订阅规则

在现有 `src/modules/logger/params.c` 添加 int32 参数 `SDLOG_ACC_MASK`、`SDLOG_GYR_MASK`，默认 **1**，范围 **0–1023**，bit n 选择该话题实例 n，标记重启 logger/设备后生效。第一版不在飞行中动态重配。

1. 对应 `SDLOG_PROFILE` FIFO 位开启才应用 mask；关闭时不新增 FIFO 或健康订阅。默认 1 保持只记录实例 0。
2. mask 0 不选择该类型实例；仍开着 FIFO profile 时沿用原有快速捕获设置，退出实验须恢复原 profile，不能只设 mask 0。
3. 负值或大于 1023：一次警告，本次启动回退 1，不改持久参数。受限构建按实际 `ORB_MULTI_MAX_INSTANCES` 裁掉不支持的高位并给一次警告；裁剪后 0 就不选择，不自动全选。
4. 每个选择的 FIFO 实例订阅间隔仍为 0。保留现有去重、订阅容量限制及失败警告。
5. `logger_topics.txt` 的原有 `ntopics > 0` 替换语义不变；mask 不向外部清单追加话题。注意该计数不等同于“至少一个有效订阅”，不顺带修复解析器。非默认 mask 遇到替换清单时适当提示，配置检查工具检出冲突。
6. 仅在无外部替换清单、FIFO profile 开启的路径追加约 1 Hz `logger_status` 两个实例。使用普通订阅，允许 logger 启动后晚发布；按消息 `type` 区分 full/mission，不仅凭实例号判断。

## 文件责任和测试

| 阶段 | 文件或产出 | 验证 |
|---|---|---|
| C0 | 官方稳定版基点、批准计划、依赖/环境/基线构建记录 | 未改功能的 X7+ 目标实际构建；原 H1 索引保留 |
| C1 | `params.c`、`logged_topics.cpp`、`LoggedTopicsTest.cpp`、logger `CMakeLists.txt` | 参数元数据；`functional-LoggedTopics` 定向功能测试；X7+ 修改版构建 |
| C2 | `Tools/vibration/` 配置示例和 ULog 验收工具，操作文档 | 正常、错误配置与合成/真实 ULog；实例、有效样本、单位、时序/缺口摘要 |
| C3 | 实际验收结果、版本和固件哈希、必要的已验证修正 | Git/diff 边界，构建与工具回归，剩余硬件验证缺口 |

测试复用 `px4_add_functional_gtest`，通过 `initialize_logged_topics()` 的实际订阅集合检查默认、不同/稀疏/零 mask、最高位、非法值及受限构建高位。不得只镜像实现的按位运算。覆盖外部清单缺失、空/注释、有效、混合/全未知、重复及容量限制，保存和恢复测试配置文件。定向入口 `make tests TESTFILTER=LoggedTopics`（CTest 正则匹配 `functional-LoggedTopics`），首次运行核实链接与 `PX4_STORAGEDIR` 路径。

如测试确需显式传入 mask，可以小幅涉及 `logged_topics.h`、`logger.cpp`、`logger.h`，说明原因，不重构 logger。H1 兼容通用构建为 `px4_fmu-v6x_default`；原 H1 定制目标只在 H1 分支，对其验证需在另一个独立工作区仅移植共享 logger 改动。

ULog 工具复用既有 pyulog/numpy。按真实驱动语义展开 `samples` 个有效值，使用 `scale`、`dt`、`timestamp_sample`，不要展开数组填充区。加速度和陀螺的实例/device_id 分别对应，不假设同一实例号必定同一颗芯片。跨包间隙和重叠需要量化容差；没有逐样本序号时，不能单凭时间异常宣称已证明具体丢样数。C2 集成验证需真正发布多实例并读取 ULog，而非仅检查位运算。

## 缓冲和实板验收

普通启动保留 `LOGGER_BUF=8 KiB` 默认；手动 CLI 默认 12 KiB 不能代替启动配置事实。实验通过 `/fs/microsd/etc/config.txt` 显式设置经验证的缓冲值，安装前保存并合并原文件，不覆盖其它用户配置，不自动安装到实机。

按实际总流量、最长已测写卡停顿及余量选择缓冲，核验连续堆空间、任务栈和分配结果。以 `logger_status.buffer_size_bytes` 确认实际大小，不能只看配置值。800 包/s 的一对 FIFO 约 0.362 MB/s，8 KiB 只覆盖约 23 ms 这部分积压；这是流量预算示例，不是实板能力保证。测量 `logger_sd_write`、`logger_sd_fsync` 长尾及多轮 `sd_bench`，不以卡的标称速度替代。

保存基线固件、参数/profile/mask、config.txt 和设备映射。持续地面采集至少覆盖预计飞行日志时长，检查 FIFO 溢出/复位、日志 dropout/message gap、有效通道连续性和资源余量。

比较同机、同工况的 CPU 周期统计、任务、堆/栈、控制运行及输出延迟。`control latency` 只覆盖传感器样本时间戳至输出模块发布时间，不能包含 ESC/电机机械响应；现有平均/最大计数不能提供 p99 或完整 deadline 证明。不得用随意设置的 CPU 百分比门槛代替实机要求。

`logger status` 查询会清零部分“自上次查询以来”的 dropout/峰值计数，必须保存每次结果和时间，并结合持续 `logger_status` 和 ULog dropout 标记。不能因查询后归零判定全程无丢失。

地面通过后再按正常飞行试验流程做短时对照，比较姿态/角速度跟踪、振荡、估计器状态。编译成功及 Flash/静态 RAM 余量不证明新增采集不会影响飞行。回退时恢复原 profile/缓冲配置并重启，必要时使用已知基线固件。
