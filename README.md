# MatekF405-UAV：RC9 云台单步点动实验（ArduCopter 4.5.7）

> 当前分支：`feature/gimbal-rc9-single-step`
>
> 父分支：`feature/matekf405-uav-target`
>
> 上游基线：ArduCopter 4.5.7，提交 `2a3dc4b7`

这是基于正式 `MatekF405-UAV` Target创建的云台功能实验分支。它保留已经完成实机验证的 **RC9 → M5/M6 → RZ7889 → 空心杯电机** 硬件链路，并把“长按连续密度调速”改为“每次离开中位只运行一个固定小步”。

This branch tests a one-shot RC9 gimbal step while preserving the hardware-tested MatekF405-UAV/RZ7889 PWM path and fail-safe behavior.

> **当前验证边界：** 原有云台硬件链路已有实机验证；本分支的单步状态机仅通过源码检查和 `MatekF405-UAV` 编译验证，尚未完成拆桨台架实测，不能标记为实机验证通过。

## 1. 分支用途

该分支只用于验证RC9单步点动控制，主要目标是：

- RC9从中位进入方向区时只触发一次；
- 对应方向以1 kHz载波、15%硬件PWM连续运行180 ms；
- 动作结束后自动停止，持续偏杆不得重复运行；
- RC9回到`1420～1580`后才重新允许下一次触发；
- RC失效立即终止动作，并要求恢复后先回中；
- 目标是约9次覆盖90°机械行程，实际次数等待台架标定。

该分支不是正式Target基线，也不是最新ArduPilot `master`。实机验证通过前不得合并回`feature/matekf405-uav-target`。

## 2. 已验证的云台链路

跨项目测试记录确认了以下链路：

```text
FS-i6（RC9，自回中通道）
        ↓ iBUS
MatekF405 / ArduCopter 4.5.7
        ↓
M5 / PA15 / TIM2_CH1 ──→ RZ7889 A1
M6 / PA8  / TIM1_CH1 ──→ RZ7889 B1
        ↓
双向空心杯电机
```

实机已经验证：

- RC9 范围约为 `1000～2000`；
- `1420～1580` 死区内电机停止；
- RC9 两侧可分别控制两个方向；
- 回到死区后停止，没有上电自转、抖动或停不住；
- 15% 硬件 PWM 能通过机构全行程的最大负载位置；
- M5/M6 由代码严格互斥，不允许两路同时驱动。

本分支保存的点动参数为：

```text
硬件载波频率：1000 Hz
单步占空比：15%
单步运行时间：180 ms
重复触发条件：必须先回到RC9中位死区
自动回中：关闭
```

1 kHz PWM仍由STM32硬件定时器产生；180 ms只是单步动作窗口，不是GPIO软件PWM。180 ms由原有15% PWM、10/50 ms密度方案约10秒覆盖全行程推算而来，属于首轮开环标定值，不能保证每步严格10°。

## 3. 失效安全行为

当前实现包含以下保护：

- RC 输入无效或超时：M5、M6 均停止；
- RC9 回到死区：两路停止；
- PWM 模式、频率或通道检查失败：禁用两路输出；
- M5/M6 功能已被其他功能占用：初始化失败并保持停止；
- 两个方向输出在同一周期共同发布，避免换向时短暂同时导通；
- 自动回中代码保留但默认关闭，避免无角度反馈时撞击机械限位。

> 云台输出在飞控未解锁时也可工作。拆桨维护时仍应断开云台电机电源，不能把“飞控未解锁”当作云台安全开关。

## 4. 硬件与接口

| 功能 | 接口/引脚 | 说明 |
|---|---|---|
| 主电机 1～4 | PWM1～PWM4 | 标准 Quad X 混控 |
| 云台方向 M5 | PA15 / TIM2_CH1 / PWM5 | 接 RZ7889 A1 |
| 云台方向 M6 | PA8 / TIM1_CH1 / PWM6 | 接 RZ7889 B1 |
| 接收机 | UART2 / Serial5 | FS-A8S iBUS，需 `BRD_ALT_CONFIG=1` |
| 光流/测距 | UART5 / Serial4 | MAVLink OpticalFlow / DistanceSensor |
| 外置罗盘 | I2C1 | 当前识别为 HMC5883，地址 `0x1E` |
| 电池电压 | PC5 / ADC1_CH15 | 分压倍率仍需万用表校准 |

## 5. 主要定制文件

| 文件 | 作用 |
|---|---|
| `ArduCopter/UserCode.cpp` | RC9、M5/M6、1 kHz硬件PWM、180 ms单步状态机和失效保护 |
| `ArduCopter/APM_Config.h` | 只对 `MatekF405-UAV` 启用所需UserHook |
| `libraries/AP_HAL_ChibiOS/hwdef/MatekF405-UAV/hwdef.dat` | 继承MatekF405并覆盖GPIO、传感器后端和功能裁剪 |
| `libraries/AP_HAL_ChibiOS/hwdef/MatekF405-UAV/hwdef-bl.dat` | 继承标准bootloader布局，保留Board ID 125 |
| `libraries/AP_HAL_ChibiOS/hwdef/MatekF405-UAV/defaults.parm` | 每次启动加载、由已保存参数优先覆盖的硬件默认值 |
| `libraries/AP_Compass/AP_Compass_QMC5883L.cpp` | 恢复上游QMC5883L身份检查；本Target不编译QMC后端 |
| `BUILD_MatekF405_WSL2.md` | WSL2构建环境和命令 |
| `MATEKF405_*_ANALYSIS.md` | 云台、光流、起飞门限等设计分析 |

通用 `MatekF405` 已恢复为ArduCopter 4.5.7定义。本机代码由 `HAL_MATEKF405_UAV` 隔离；构建标准 `MatekF405` 时不会启用云台UserHook。

### 默认参数说明

`defaults.parm`会在每次启动时加载。飞控参数存储中**已经保存的参数**优先于这些默认值；但从未保存、一直沿用旧固件默认值的参数，升级后可能改用本Target的默认值。因此刷写前应导出参数，刷写后仍需在Mission Planner逐项比较串口、RC、罗盘方向、M5/M6和日志设置。

已加入的硬件默认值包括：

- `UART2 / Serial5`：iBUS接收机，`BRD_ALT_CONFIG=1`；
- `UART5 / Serial4`：MAVLink1光流与测距；
- 外置HMC5883：I2C地址`0x1E`，安装方向暂按`Yaw270`保存；
- M5/M6：`SERVO5_FUNCTION=94`、`SERVO6_FUNCTION=95`；
- `ARMING_CHECK=1`，保留SD日志。

没有写入罗盘DevID、罗盘偏置、电池倍率或实验性EKF/Loiter参数，因为这些数值必须在对应实机上校准。

## 6. 新手构建步骤

在 WSL2 Ubuntu 22.04 中：

```bash
cd $HOME/ardupilot
mkdir -p /tmp/ardupilot-ccache /tmp/ardupilot-ccache-tmp

env PATH=$HOME/ardupilot/venv/bin:/usr/lib/ccache:/usr/bin:/bin \
  CCACHE_DIR=/tmp/ardupilot-ccache \
  CCACHE_TEMPDIR=/tmp/ardupilot-ccache-tmp \
  venv/bin/python waf configure --board MatekF405-UAV

env PATH=$HOME/ardupilot/venv/bin:/usr/lib/ccache:/usr/bin:/bin \
  CCACHE_DIR=/tmp/ardupilot-ccache \
  CCACHE_TEMPDIR=/tmp/ardupilot-ccache-tmp \
  venv/bin/python waf copter -j4
```

生成文件：

```text
build/MatekF405-UAV/bin/arducopter.apj
build/MatekF405-UAV/bin/arducopter.bin
```

Mission Planner选择`arducopter.apj`。第一阶段继续使用飞控中已经验证的标准MatekF405 bootloader；Target保留相同的`APJ_BOARD_ID=125`，但应用固件不内嵌bootloader。**不要在当前阶段刷写`AP_Bootloader.bin`。**

刷写前应先导出当前参数。刷写后拆桨逐项确认接收机、光流、测距、罗盘、电池和云台，不能直接带桨飞行。

## 7. 回滚方法

查看稳定标签：

```bash
git show baseline/matekf405-gimbal-sd-20260730
```

需要从稳定状态重新开发时，应创建新分支，不要在标签上直接工作：

```bash
git switch -c feature/my-change baseline/matekf405-gimbal-sd-20260730
```

## 8. 分支地图

| 分支 | 状态 | 作用 |
|---|---|---|
| `project/matekf405-gimbal-ap457` | 稳定基线 | 已验证云台及当前MatekF405功能 |
| `test/matekf405-no-sd-fatfs` | 台架实验 | 关闭SPI SD/FATFS，分析启动延迟 |
| `feature/matekf405-uav-target` | 已编译、待实机 | 独立 `MatekF405-UAV` 板级Target |
| `feature/gimbal-rc9-single-step` | 已编译、待台架 | RC9每次离中触发15%/180 ms单步 |

## 9. 已完成的构建验证

以下结果来自本分支实际构建，不是预计值：

| 构建 | 结果 | Flash使用/剩余 |
|---|---|---|
| `MatekF405-UAV` ArduCopter（单步180 ms） | 通过 | 899,707 B / 83,332 B |
| 标准 `MatekF405` ArduCopter | 通过 | 897,948 B / 85,092 B |
| `MatekF405-UAV` bootloader | 通过 | 14,440 B / 18,328 B |

同时确认：

- APJ Board ID仍为125；
- SD/FATFS、EKF3、光流、MAVLink测距和日志仍编译；
- HMC5883后端保留，未使用的QMC5883L后端在本Target关闭；
- 通用MatekF405仍可独立构建；
- 构建产物不提交Git，只保留源码、README和构建方法。

## 10. 累计测试记录与下一步

### 2026-08-13：单步180 ms首版

- 分支：`feature/gimbal-rc9-single-step`；
- 参数：1 kHz、15%、180 ms，自动回中关闭；
- 编译：`MatekF405-UAV`通过，Board ID 125；
- 当前状态：**待拆桨台架验证**，没有实机通过结论；
- 待测：上电不动作、一次离中只运行一步、持续偏杆不重复、回中后可重触发、RC失效立即停止、M5/M6严格互斥；
- 标定：分别记录上到下、下到上覆盖90°所需次数；机械到达端点后禁止继续点动。

台架反馈后只更新本节累计记录，不为每次参数调整新建重复文档。若实测不是9步，应在本功能分支内调整180 ms并形成新提交。

回滚到正式Target基线：

```bash
git switch feature/matekf405-uav-target
```

## 11. 安全与许可

- 这是特定硬件上的研究/实验固件，不代表 ArduPilot 官方支持；
- 首次刷写或参数变化后必须拆桨台架验证；
- 不删除 Arming、EKF、RC、电池、Crash、Thrust Loss 等安全检查；
- 新Target尚未实机刷写，不能把“编译通过”等同于“飞行验证通过”；
- ArduPilot 及本派生工作继续遵循 GNU GPL v3，完整许可见 `COPYING.txt`；
- 原始 ArduPilot 项目及贡献者信息保留在下方。

---

# Upstream ArduPilot Project

<a href="https://ardupilot.org/discord"><img src="https://img.shields.io/discord/674039678562861068.svg" alt="Discord">

[![Test Copter](https://github.com/ArduPilot/ardupilot/workflows/test%20copter/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_copter.yml) [![Test Plane](https://github.com/ArduPilot/ardupilot/workflows/test%20plane/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_plane.yml) [![Test Rover](https://github.com/ArduPilot/ardupilot/workflows/test%20rover/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_rover.yml) [![Test Sub](https://github.com/ArduPilot/ardupilot/workflows/test%20sub/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_sub.yml) [![Test Tracker](https://github.com/ArduPilot/ardupilot/workflows/test%20tracker/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_tracker.yml)

[![Test AP_Periph](https://github.com/ArduPilot/ardupilot/workflows/test%20ap_periph/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_periph.yml) [![Test Chibios](https://github.com/ArduPilot/ardupilot/workflows/test%20chibios/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_chibios.yml) [![Test Linux SBC](https://github.com/ArduPilot/ardupilot/workflows/test%20Linux%20SBC/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_linux_sbc.yml) [![Test Replay](https://github.com/ArduPilot/ardupilot/workflows/test%20replay/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_replay.yml)

[![Test Unit Tests](https://github.com/ArduPilot/ardupilot/workflows/test%20unit%20tests/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_unit_tests.yml) [![test size](https://github.com/ArduPilot/ardupilot/actions/workflows/test_size.yml/badge.svg)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_size.yml)

[![Test Environment Setup](https://github.com/ArduPilot/ardupilot/actions/workflows/test_environment.yml/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_environment.yml)

[![Cygwin Build](https://github.com/ArduPilot/ardupilot/actions/workflows/cygwin_build.yml/badge.svg)](https://github.com/ArduPilot/ardupilot/actions/workflows/cygwin_build.yml) [![Macos Build](https://github.com/ArduPilot/ardupilot/actions/workflows/macos_build.yml/badge.svg)](https://github.com/ArduPilot/ardupilot/actions/workflows/macos_build.yml)

[![Coverity Scan Build Status](https://scan.coverity.com/projects/5331/badge.svg)](https://scan.coverity.com/projects/ardupilot-ardupilot)

[![Test Coverage](https://github.com/ArduPilot/ardupilot/actions/workflows/test_coverage.yml/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_coverage.yml)

[![Autotest Status](https://autotest.ardupilot.org/autotest-badge.svg)](https://autotest.ardupilot.org/)

ArduPilot is the most advanced, full-featured, and reliable open source autopilot software available.
It has been under development since 2010 by a diverse team of professional engineers, computer scientists, and community contributors.
Our autopilot software is capable of controlling almost any vehicle system imaginable, from conventional airplanes, quad planes, multi-rotors, and helicopters to rovers, boats, balance bots, and even submarines.
It is continually being expanded to provide support for new emerging vehicle types.

## The ArduPilot project is made up of: ##

- ArduCopter: [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter), [wiki](https://ardupilot.org/copter/index.html)

- ArduPlane: [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane), [wiki](https://ardupilot.org/plane/index.html)

- Rover: [code](https://github.com/ArduPilot/ardupilot/tree/master/Rover), [wiki](https://ardupilot.org/rover/index.html)

- ArduSub : [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduSub), [wiki](http://ardusub.com/)

- Antenna Tracker : [code](https://github.com/ArduPilot/ardupilot/tree/master/AntennaTracker), [wiki](https://ardupilot.org/antennatracker/index.html)

## User Support & Discussion Forums ##

- Support Forum: <https://discuss.ardupilot.org/>

- Community Site: <https://ardupilot.org>

## Developer Information ##

- Github repository: <https://github.com/ArduPilot/ardupilot>

- Main developer wiki: <https://ardupilot.org/dev/>

- Developer discussion: <https://discuss.ardupilot.org>

- Developer chat: <https://discord.com/channels/ardupilot>

## Top Contributors ##

- [Flight code contributors](https://github.com/ArduPilot/ardupilot/graphs/contributors)
- [Wiki contributors](https://github.com/ArduPilot/ardupilot_wiki/graphs/contributors)
- [Most active support forum users](https://discuss.ardupilot.org/u?order=post_count&period=quarterly)
- [Partners who contribute financially](https://ardupilot.org/about/Partners)

## How To Get Involved ##

- The ArduPilot project is open source and we encourage participation and code contributions: [guidelines for contributors to the ardupilot codebase](https://ardupilot.org/dev/docs/contributing.html)

- We have an active group of Beta Testers to help us improve our code: [release procedures](https://ardupilot.org/dev/docs/release-procedures.html)

- Desired Enhancements and Bugs can be posted to the [issues list](https://github.com/ArduPilot/ardupilot/issues).

- Help other users with log analysis in the [support forums](https://discuss.ardupilot.org/)

- Improve the wiki and chat with other [wiki editors on Discord #documentation](https://discord.com/channels/ardupilot)

- Contact the developers on one of the [communication channels](https://ardupilot.org/copter/docs/common-contact-us.html)

## License ##

The ArduPilot project is licensed under the GNU General Public
License, version 3.

- [Overview of license](https://ardupilot.org/dev/docs/license-gplv3.html)

- [Full Text](https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt)

## Maintainers ##

ArduPilot is comprised of several parts, vehicles and boards. The list below
contains the people that regularly contribute to the project and are responsible
for reviewing patches on their specific area.

- [Andrew Tridgell](https://github.com/tridge):
  - ***Vehicle***: Plane, AntennaTracker
  - ***Board***: Pixhawk, Pixhawk2, PixRacer
- [Francisco Ferreira](https://github.com/oxinarf):
  - ***Bug Master***
- [Grant Morphett](https://github.com/gmorph):
  - ***Vehicle***: Rover
- [Willian Galvani](https://github.com/williangalvani):
  - ***Vehicle***: Sub
- [Lucas De Marchi](https://github.com/lucasdemarchi):
  - ***Subsystem***: Linux
- [Michael du Breuil](https://github.com/WickedShell):
  - ***Subsystem***: Batteries
  - ***Subsystem***: GPS
  - ***Subsystem***: Scripting
- [Peter Barker](https://github.com/peterbarker):
  - ***Subsystem***: DataFlash, Tools
- [Randy Mackay](https://github.com/rmackay9):
  - ***Vehicle***: Copter, Rover, AntennaTracker
- [Siddharth Purohit](https://github.com/bugobliterator):
  - ***Subsystem***: CAN, Compass
  - ***Board***: Cube*
- [Tom Pittenger](https://github.com/magicrub):
  - ***Vehicle***: Plane
- [Bill Geyer](https://github.com/bnsgeyer):
  - ***Vehicle***: TradHeli
- [Emile Castelnuovo](https://github.com/emilecastelnuovo):
  - ***Board***: VRBrain
- [Georgii Staroselskii](https://github.com/staroselskii):
  - ***Board***: NavIO
- [Gustavo José de Sousa](https://github.com/guludo):
  - ***Subsystem***: Build system
- [Julien Beraud](https://github.com/jberaud):
  - ***Board***: Bebop & Bebop 2
- [Leonard Hall](https://github.com/lthall):
  - ***Subsystem***: Copter attitude control and navigation
- [Matt Lawrence](https://github.com/Pedals2Paddles):
  - ***Vehicle***: 3DR Solo & Solo based vehicles
- [Matthias Badaire](https://github.com/badzz):
  - ***Subsystem***: FRSky
- [Mirko Denecke](https://github.com/mirkix):
  - ***Board***: BBBmini, BeagleBone Blue, PocketPilot
- [Paul Riseborough](https://github.com/priseborough):
  - ***Subsystem***: AP_NavEKF2
  - ***Subsystem***: AP_NavEKF3
- [Víctor Mayoral Vilches](https://github.com/vmayoral):
  - ***Board***: PXF, Erle-Brain 2, PXFmini
- [Amilcar Lucas](https://github.com/amilcarlucas):
  - ***Subsystem***: Marvelmind
- [Samuel Tabor](https://github.com/samuelctabor):
  - ***Subsystem***: Soaring/Gliding
- [Henry Wurzburg](https://github.com/Hwurzburg):
  - ***Subsystem***: OSD
  - ***Site***: Wiki
- [Peter Hall](https://github.com/IamPete1):
  - ***Vehicle***: Tailsitters
  - ***Vehicle***: Sailboat
  - ***Subsystem***: Scripting
- [Andy Piper](https://github.com/andyp1per):
  - ***Subsystem***: Crossfire
  - ***Subsystem***: ESC
  - ***Subsystem***: OSD
  - ***Subsystem***: SmartAudio
- [Alessandro Apostoli ](https://github.com/yaapu):
  - ***Subsystem***: Telemetry
  - ***Subsystem***: OSD
- [Rishabh Singh ](https://github.com/rishabsingh3003):
  - ***Subsystem***: Avoidance/Proximity
- [David Bussenschutt ](https://github.com/davidbuzz):
  - ***Subsystem***: ESP32,AP_HAL_ESP32
- [Charles Villard ](https://github.com/Silvanosky):
  - ***Subsystem***: ESP32,AP_HAL_ESP32
