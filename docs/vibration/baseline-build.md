# X7+ 官方 v1.16.1 基线构建记录

日期：2026-10-12。结果：`cuav_x7pro_default` 全量构建通过（1289 个构建步骤，退出码 0），生成 `.elf`、`.bin`、`.px4`。本阶段没有修改飞控功能源码、板级默认、IMU采样率、滤波或控制参数，没有烧写实机。

## 源码与环境

- 官方 `v1.16.1^{commit}`：`94cb2012792b2ae89f0b147cfee53ee31ae550be`。固件生成头确认该 SHA 和 `PX4_GIT_TAG_STR="v1.16.1"`。
- 功能分支从该官方标签建立；H1分支的 `f298aad` 不是本次功能分支的基点。原H1工作区及六个Rockchip暂存文件保留。
- 固定依赖见 [baseline-dependencies.json](baseline-dependencies.json)。所有已启用硬件依赖按父仓库gitlink检出，MAVLink嵌套pymavlink也已补齐，未更新到远程最新提交。
- DDS构建额外取得官方 Micro-CDR `v2.0.1`，实际提交 `3d1b17703c7cf4f22def2910bc845bdb5152d7b5`，由已固定的Micro-XRCE-DDS-Client构建脚本指定。
- WSL2 / Ubuntu26.04，Make4.4.1、CMake4.2.3、Ninja1.13.2、Python3.14.4、empy3.3.4。
- 本次独立工具链：Arm GNU Toolchain13.2.rel1，GCC13.2.1 20231009。安装目录 `/home/mjy/px4-builds/toolchains/arm-gnu-13.2.rel1`，仅对本次构建设置PATH，原系统GCC14未替换。
- 官方工具链包SHA256：`6cd1bbc1d9ae57312bcd169ae283153a9572bd6a8e4eeae2fedfbc33b115fdbb`，与Arm官方校验文件一致。

原GCC14.2在DDS依赖的无网络板UDP编译路径中，将`getaddrinfo/freeaddrinfo`的隐式声明诊断升级为错误。改用版本文档示例的GCC13.2后通过；未启用板级网络、删模块、降低全局错误检查或修改第三方源码。基线保留第三方既有警告，不能将“构建通过”解读为没有警告。

## NuttX版本元数据

实际检出的NuttX源码始终为 `886acbbdb4f061e5c0ce1a76afbcfa7cb7df9849`，apps为 `e37940d8535f603a16b8f6f21c21edaf584218aa`。

旧版本生成器在`describe`无法提供版本时会取最高标签。浅克隆无标签会使PX4版本头生成失败；远端后来增加的12.12标签又会将这份历史源码错误声明成12.12。本次构建隔离较新标签，采用官方11.0.0标签作为**历史版本元数据参照**，重新生成版本信息。

11标签不是固定维护提交的祖先，不代表检出了11发布源码。两条线共同基点为`dd718e78f70f9350ac648067509672c5051841b9`；维护线与发布标签线有实际分叉。真实源码身份以固定SHA为准，不移动官方标签，不检出另一份NuttX代码。该区别在构建记录中保留。

## 固件与资源

板型包`board_id=1010`。已解压`.px4`映像，确认与`.bin`逐字节一致，长度等于包声明的`image_size`，且未超过应用Flash区域。构建目标名称含pro，不代替普通X7+实机型号、bootloader和IMU device_id核验。

| 链接区域 | 使用量 | 区域容量 | 说明 |
|---|---:|---:|---|
| FLASH | 1,914,760 B | 1,966,080 B（1920 KiB） | 97.39%，剩余51,320 B，后续逐阶段检查 |
| AXI_SRAM | 44,712 B | 512 KiB | 8.53%，静态链接占用 |
| SRAM4 | 2,048 B | 64 KiB | 3.12%，静态链接占用 |

`.px4`包为1,777,983 B，`.elf`为46,544,836 B；这两个文件大小都不能当作Flash占用。参数缓冲、logger缓冲、运行堆/栈和任务动态内存不由以上静态数字证明。

固件SHA256、实际文件大小、板型和源码身份详见 [baseline-build.json](baseline-build.json)。原始产物保存在 `E:/07-Workspace_codex/output/px4-x7-baseline/2026-10-12/baseline/`，成功构建日志为上级目录的`build-attempt-04.log`，环境日志为`environment-attempt-04.log`。

## 复现与后续验收

使用固定版本依赖、历史标签元数据和上述工具链，在新的WSL本地构建目录执行：

```sh
PATH=/home/mjy/px4-builds/toolchains/arm-gnu-13.2.rel1/bin:$PATH \
CMAKE_BUILD_PARALLEL_LEVEL=4 make cuav_x7pro_default -j4
```

本次成功构建目录 `/home/mjy/px4-builds/cuav-x7-v1161-gcc13`。Windows磁盘上的首次配置耗时约566秒，同一固定源码的WSL本地目录配置约10秒；后续构建和测试继续使用WSL本地目录，源码编辑仍在独立工作区中完成。

后续仅按批准范围添加logger实例选择与诊断。Flash剩余约50 KiB，不为本功能擅自删除板级模块。CPU开销、动态RAM、SD写入长尾、FIFO/日志丢失、控制时序以及飞行性能仍需同机地面和飞行对照；本阶段未完成这些硬件验证。
