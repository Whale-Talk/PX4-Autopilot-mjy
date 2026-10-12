# 原生 FIFO 振动采集 · 实板操作与回退指南

适用范围：CUAV X7+（`cuav_x7pro_default`）与 H1（`fmu-v6x_seeex25d`）上的原生
`sensor_accel_fifo` / `sensor_gyro_fifo` 采集。本指南**只描述操作与判读**，不改变任何
采样率、硬件滤波、积分/控制参数、调度策略、写入线程优先级或 `LOGGER_BUF` 默认值。

> 本指南中的带宽、缓冲与 Flash 数值是**预算**（由消息结构与配置推导），不是实机性能结论。
> 未连接硬件、未完成地面与飞行验收前，不得声称 CPU 余量或飞行性能已通过。

---

## 1. 不变量（任何档位都不得改动）

| 项 | 保持 |
|---|---|
| IMU ODR / 硬件滤波 | 不变（`IMU_GYRO_RATEMAX` 等仅由既有配置决定，本实验不改） |
| 积分频率、控制参数、传感器选择 | 不变 |
| 采集调度、消息结构 | 不变 |
| 写卡线程优先级、`LOGGER_BUF` 默认 | 不变 |
| 分析位置 | PSD/FFT **离线**在电脑端执行，不占用飞行 CPU |

注意一个**现有行为**（非本次引入）：一旦 `SDLOG_PROFILE` 的第 8/9 位被置位，logged_topics
的既有分支会把 logger 的任务优先级提高并把日志间隔缩短到 800 µs。因此"进入实验"和"退出实验"
都必须以 **profile 位**为准。

---

## 2. 参数与生效方式

```bash
# 1) 只有默认飞行日志（对照档，不做高频采集）
param set SDLOG_PROFILE 1
param set SDLOG_MODE 1        # 1=上电即开始记录，直到 disarm；2=上电记录到关机（地面台架建议 2）
                              # 注意：不是"解锁后才开始"，1 从上电就写

# 2) 官方 raw 采集路径（既有功能：只记录实例 0）
param set SDLOG_PROFILE 769   # 1(默认集) + 256(陀螺 FIFO) + 512(加速度 FIFO)
# 默认 SDLOG_ACC_MASK=1 / SDLOG_GYR_MASK=1 —— 与既有一致，不需要改

# 3) 指定实例（本功能 C1）
param set SDLOG_GYR_MASK 1    # 仅陀螺实例 0；bit n = 实例 n
param set SDLOG_ACC_MASK 1    # 仅加速度实例 0
param set SDLOG_GYR_MASK 3    # 0 与 1 两颗
```

* 参数**重启后生效**（logger 启动时读取）。第一版不支持飞行中动态重配。
* `mask = 0` **不会**关闭快速捕获：只要 profile 第 8/9 位还在，logger 仍按快速模式运行。
  退出实验必须**同时**恢复 profile 并重启（见 §7）。
* 非法值（<0 或 >1023）只在启动时警告一次并回退为 1；超出现有构建实例上限的高位会被裁掉，
  裁剪后为 0 时**不会自动全选**。

**持久化建议**：台架实验优先使用 SD 卡 `/fs/microsd/etc/config.txt`（保存原文件后**合并**，
不覆盖他人配置，工具不自动安装）。若用 `param set` + `param save`，请在实验结束后把
`SDLOG_PROFILE`、`SDLOG_ACC_MASK`、`SDLOG_GYR_MASK`、`SDLOG_MODE` 一并恢复。

### 提交前先跑配置检查

```bash
# 把 `param show` 输出保存成文本，再连同 SD 卡上的 logger_topics.txt（如有）一起检查
param show > params.txt
python3 Tools/vibration/px4_vibration_config_check.py --params params.txt \
        --topics-file logger_topics.txt
```

工具会报出：profile 位是否开启、mask 非法/裁剪/裁剪成 0、profile 关闭但 mask 有选择、
以及 `logger_topics.txt` 的替换冲突（该文件一旦有任何一行被解析就**整体替换** profile 清单，
包括全是未知话题名的情况；此时 mask 与健康话题都不生效）。

* `--params` 直接接受真实 `param show` 输出（含 `x` / `*` / `+` 状态标记与
  `NAME [index,used] : VALUE` 括号形式，见 `src/systemcmds/param/param.cpp:680`），
  也接受简单 `NAME VALUE` / `NAME: VALUE`。
* 取值**不做截断**：`SDLOG_*` 出现小数或无法解析时是**错误**（退出码 1），不会默认成 1、
  也不会把 `1.5` 截成 `1`，更不会因字符串参与位运算而崩溃。目标参数缺失则明确报告为"缺证据"。
* `logger_topics.txt` 只对**常规输入**保证复刻固件 `ntopics > 0` 的替换语义；行超过 79 字节
  （固件 `fgets` 会截断成两行）、数值字段带尾随字符、超出 `uint16`/`uint8` 收窄范围等情况会以
  `topics-file/ambiguous-line` 显式报出，要求人工复核，**不声称**与 `sscanf` 完全等价。

---

## 3. 四档对比矩阵（必须四档都做，不能只比"普通 vs 全量"）

| 档 | 固件 | 配置 | 采集内容 | 每档必记 |
|---|---|---|---|---|
| A 普通 | 基线固件 | `SDLOG_PROFILE=1` | 现有飞行日志（`sensor_combined` 200 Hz 等） | CPU/任务/堆栈、控制时序、写卡计数、日志总码率 |
| B 官方 raw 单颗 | **官方基线固件**（无 `SDLOG_*_MASK` 参数） | `SDLOG_PROFILE=769` | `sensor_gyro_fifo`/`sensor_accel_fifo` **仅实例 0** | 同上 + FIFO 溢出/复位计数、ULog dropout、有效样本数 |
| C 新 raw 单颗 | **新固件** | `SDLOG_PROFILE=769` + mask 选**与 B 完全相同的实例** | 同一 `device_id`、同一通道数 | 同上 + 期望实例与实际实例一致性 |
| D 多颗 | 新固件 | `SDLOG_PROFILE=769` + 两位以上 mask（例如 `GYR_MASK=3`、`ACC_MASK=3`） | 多实例原始流 | 同上 + 带宽/缓冲压力、dropout 与 message_gaps 变化 |

> **B 与 C 必须同实例、同 `device_id`、同通道数**（C 只换固件），否则"传感器/通道变化"会与
> "代码变化"混在一起，无法归因。D 才是有意增加通道，用来观察资源变化。
> 只有对照 A→B→C→D 才能把"新增通道数"与"资源变化"分开；只看 A 与 D 无法归因。

> **构建前提**：FMU-v6x 的通用目标编译通过只说明**编译兼容性**；H1 原始 custom 适配目标
> 属另一套 board 配置，尚未包含本次补丁，需单独构建验证后才能用于 B/C/D。

---

## 4. 采集流程（地面 → 悬停 → 飞行）

1. **地面准备**：保存参数快照（`param show`）、profile/mask、`config.txt`、设备与 IMU→`device_id` 映射。
2. **地面录制**：螺旋桨断开或系留，`SDLOG_MODE=2`，录满预计飞行时长（至少等长）。
   检查：FIFO 溢出/复位、ULog dropout、消息缺口、有效通道连续性、资源余量。
3. **悬停/飞行对照**：按 A→B→C→D 顺序各做同工况短时架次，保持同一机体与同一 SD 卡。
4. **每次改动后**：`logger status` 的查询会**清零**"自上次查询以来"的 dropout/峰值计数，
   因此必须**逐次保存输出与时间**，并结合持续 `logger_status` 与 ULog 内 dropout 标记，
   不能因为查询后归零就判定"全程无丢失"。

---

## 5. 数据验收（离线）

### 5.1 主机侧链路验收（真实 uORB → logger → 写卡链路）

先用固件侧功能测试确认"选中的实例确实被写进 ULog"，再做分析：

```bash
export PX4_VIBRATION_INTEGRATION_DIR="$(mktemp -d)"    # 输出临时父目录
make tests TESTFILTER=LoggerFifoIntegration            # 构建并运行 functional-LoggerFifoIntegration
# 该目录下会生成 logger_fifo_expected.json（含 ulog 路径、profile 768、acc mask 2、gyr mask 5、32 samples/batch 等）
python3 Tools/vibration/px4_ulog_vibration_check.py \
        "$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["ulog"])' \
           "$PX4_VIBRATION_INTEGRATION_DIR/logger_fifo_expected.json")" \
        --expect-acc-mask 2 --expect-gyr-mask 5
```

覆盖范围：**主机侧合成 publisher + 真实 logger 写卡链路**，`SDLOG_PROFILE=768`、
`SDLOG_ACC_MASK=2`、`SDLOG_GYR_MASK=5`、每包 32 个样本。它验证选择/标定/序列化通路，
**不代表** X7+ 的 CPU 余量、SD 卡带宽或飞行表现。

### 5.2 实机日志分析

```bash
python3 Tools/vibration/px4_ulog_vibration_check.py log_XX.ulg \
        --expect-gyr-mask 3 --expect-acc-mask 3 --gap-tolerance-us 2000 \
        --orb-multi-max 10 --json report.json
```

* `--orb-multi-max` 必须与实机构建的实例上限一致（普通构建 10，受限构建 4）。该值**由用户指定**，
  日志元数据无法证明它；它只影响 mask 的解释。
* `--gap-tolerance-us` / `--latency-limit-us` / `--full-scale-fraction` 必须是有限合理值，
  传 `nan`/`inf`/负数会直接以用法错误退出，不会静默遮住异常。

报告包含（每项都指明数据来源读取器）：FIFO 流（实例、包数、有效样本总数、端点采样率、
`dt`/`scale` 统计、`device_id` 分段、跨包残差、发布延迟、原始饱和/近满量程）、健康证据
（ULog dropout、`logger_status`、`vehicle_imu_status`）与"已检查项"清单。

### 判读规则

* **随转速漂移** → 桨叶通过频率（BPF = RPM × 桨叶数 / 60）及其谐波；
  **固定不动** → 结构弯曲/扭转模态；**非整数倍且与转速无关** → 装配/松动等异步成分。
* ~90–100 Hz 附近 BPF 二次谐波与一阶弯曲模态会重叠，必须结合转速同步测量区分。
* `samples` 之后数组尾部是填充区，不参与分析；`timestamp_sample` 是该批**最后一个**样本的名义锚点。
* 跨包残差 `T[k]-T[k-1]-N[k]*dt[k]` 超容差只说明**时间一致性异常**，不能据此宣称精确丢样数；
  残差与端点速率**按 `device_id` 分段计算**（跨设备求全局 span 会得到伪速率），报告逐段列出
  `device_id` / 包数 / 端点速率 / 残差。`device_id = 0` 视为**驱动映射缺失**的证据，
  该实例与芯片的对应关系不可信。
* `timestamp - timestamp_sample` 是**驱动发布时刻**相对该批末端锚点的延迟，**不是**写卡耗时。
* 原始值达到 int16 数值极限（±32767 / −32768）只能说明**接近数值极限**，不等于物理饱和或 clipping；
  近极限但未达极限的样本单独统计。**真正 clipping 必须用驱动计数**
  （`vehicle_imu_status.accel_clipping` / `gyro_clipping`）确认。
* ICM42688P 的 `dt` 是该批最后的芯片时间增量（量化约 17.0667 µs 且批内可能变化），
  ICM20689 的加速度相位可能与陀螺相差约 125 µs，因此**不要**硬性要求 `dt=125` 或残差为 0，
  也不要宣称 acc/gyr 严格同步。
* 健康判定**逐行**进行：只有 `logger_status` 中 `type=FULL` 且 `is_logging` 且 backend 含 file 的行
  才作为验收证据，缓冲占用率只取自这些行（不跨行拼接）；若整份日志没有这样的行，报告为
  **缺验收证据**；inactive mission（`buffer_size_bytes = 0`）不算故障。
* 健康结论必须来自证据：无 `logger_status` / `vehicle_imu_status` 时报告"**缺证据**"，
  不能默认健康；`message_gaps` 是 generation 跳跃事件次数，**不是丢样个数**。

### 频段计划

* 主分析 **1–500 Hz**：由所选 IMU 的实际 ODR 决定上限（用 `vehicle_imu_status` 的
  `*_raw_rate_hz` 与检查工具的端点速率核实，不要凭标称值）。
* **500–1000 Hz** 作为探索频段，受传感器/链路与记录带宽限制，结论必须标注为探索性。
* PSD/FFT 一律离线执行，不进入飞行 CPU 预算。

---

## 6. 资源与带宽预算（预算，不是实测结论）

* 单条 `sensor_*_fifo` 记录固定约 **226 B**（`msg_id` + 结构字段），与包内有效样本数无关。
* **每条话题** 800 包/s：陀螺 + 加速度两条 = 1600 包/s ≈ **0.3616 MB/s**。
  若两条合计只有 800 包/s（即每条 400 包/s），则 ≈ **0.1808 MB/s**。
* 8 KiB 缓冲在上述两种情形下分别只能覆盖约 **22.7 ms** 与 **45 ms** 的积压。
* 实际发布率必须**实测**（端点速率），它既不等于样本 ODR，也不等于采集唤醒率。
* 实际写卡能力必须以本机 `logger_sd_write`/`logger_sd_fsync` 长尾与多轮 `sd_bench` 为准，
  不得用 SD 卡标称速度替代；Flash 占用与缓冲配置同样**不是**性能结论。

---

## 7. 回退（必须一次做全）

1. `/fs/microsd/etc/config.txt`：删除或还原本次合并的 `SDLOG_*` 行（保留其它用户配置）。
2. 恢复参数：`param set SDLOG_PROFILE 1`（或原值）、`SDLOG_ACC_MASK 1`、`SDLOG_GYR_MASK 1`、
   `SDLOG_MODE 0`，必要时 `param save`。
3. **重启飞控**（参数仅在 logger 启动时读取）。只把 mask 设为 0 **不会**退出快速捕获模式。
4. 确认已退出：`logger status` 不应再显示 800 µs 日志间隔与提高后的优先级；
   下一次日志应只有普通话题、无 `sensor_*_fifo`。
5. 若资源或控制时序出现异常，回退到已保存的**基线固件**并记录差异；回退后重新做一次 A 档对照。

---

## 8. 未完成项（不得当成已完成）

| 项 | 状态 |
|---|---|
| 本机 CPU/任务/堆栈/控制时序对照 | **未做**（需要实机与四档数据） |
| `sd_bench` 与写卡长尾实测 | **未做** |
| 主机侧 uORB→logger→ULog 链路（多实例 + acc FIFO） | 由固件侧 `functional-LoggerFifoIntegration` 覆盖（见 §5.1）；**不代表** X7+ CPU 或飞行 |
| 实机 ULog 验收（多实例 mask、H1 custom 适配目标） | **未做**；通用 FMU-v6x 编译通过不等于 H1 original custom 目标已含补丁 |
| pyulog 交叉验证与真实 `log_101` 最终验收 | 由仓库环境（含 `Tools/setup/requirements.txt`）负责执行 |
| 飞行性能与振动抑制效果 | **未做**；编译通过与 Flash 余量不构成结论 |
