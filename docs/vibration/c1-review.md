# C1：FIFO 实例选择与日志健康订阅验收

2026-10-12。基点为官方 PX4 v1.16.1（94cb2012）。DSH 在“飞控震动问题处理”原会话编码，Codex 独立读取差异、构建并运行测试。

## 修改边界

仅修改 logger 的订阅配置：两个默认值为 1 的实例 mask 参数，在对应 raw FIFO profile 开启时选择加速度和陀螺实例，并以原间隔 0 订阅。在该路径增加两个约 1 Hz 的 logger_status 订阅；消费者必须按 type 区分 full/mission。外部 logger_topics.txt 的原替换规则、容量、去重及优先级保持不变。

涉及 params.c、logged_topics.cpp/.h、logger.cpp/.h、CMakeLists.txt，以及新增 LoggedTopicsTest.cpp。参数通过 logger 的既有参数包装读取，再显式传入订阅配置；没有修改 IMU 驱动、消息结构、采样率、滤波、积分、估计器或控制器。

uORB 订阅者各自维护读取代数。logger 读取不会消耗控制订阅者的数据，但共享 CPU、队列、内存和存储仍会带来资源竞争。raw profile 原本就改变 logger 的捕获周期和优先级；本改动沿用该行为，mask 为 0 不会取消它。

## 已运行验证

- 普通 host 功能测试：17 项中 16 项通过，1 项跳过。10 实例构建不存在“合法 mask 超出构建实例数”的裁剪情况，因此该项按设计跳过。
- 独立 CONSTRAINED_MEMORY 构建实际采用 4 实例：17/17 通过，包括裁剪后为零、默认/非法/稀疏/最高位、外部清单替换及容量用例。
- 新参数生成元数据确认：INT32，默认 1，范围 0–1023，reboot_required=true。
- X7 候选构建通过：FLASH 1,915,480/1,966,080 B，比基线增加 720 B，剩余 50,600 B。静态 AXI_SRAM 44,712 B、SRAM4 2,048 B，与基线一致。
- 使用仓库指定 AStyle 2.06 检查并修正新增测试的格式；不使用不兼容的 AStyle 3.6 返回码作为通过证据。

首次测试链接暴露 C++14 类静态常量被 GoogleTest 引用造成的 ODR-use，已在测试中先转换成局部 int 值修正，没有给生产类增加存储定义。格式修正后重跑两种配置的定向测试。

## 证据边界

上述功能测试检查实际订阅集合，不代表 logger 完整生命周期或 uORB→ULog 集成已验证。容量用例检查结果上限与截断前缀，不捕获警告文本，也不能仅凭 count 等于容量证明发生了额外添加失败。

候选固件由未提交补丁构建，其版本头仍显示基点身份，不能作为最终交付固件。提交后重新检出实际提交、构建及归档，另行记录源码 SHA 和产物哈希。

没有连接实机或烧写。运行 CPU、堆/栈、SD 长尾、FIFO/日志丢失、控制周期及飞行性能尚未验证；Flash 与静态 RAM 数值不替代这些检查。

原始验证日志在 E:/07-Workspace_codex/output/px4-x7-baseline/2026-10-12/：c1-functional-tests-repaired.log、c1-constrained-functional-tests.log、c1-candidate-x7-build.log。
