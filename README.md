# MatekF405 UAV：云台稳定基线（ArduCopter 4.5.7）

> 当前分支：`project/matekf405-gimbal-ap457`
>
> 稳定标签：`baseline/matekf405-gimbal-sd-20260730`
>
> 上游基线：ArduCopter 4.5.7，提交 `2a3dc4b7`

这是基于 ArduPilot 4.5.7 的 MatekF405 四旋翼定制固件。该分支保留了已经在“无人机云台电机模块”项目中完成实机验证的 **RC9 → M5/M6 → RZ7889 → 空心杯电机** 云台控制链路，同时保留光流、测距、GPS、EKF3、日志和必要安全功能。

This branch is a hardware-tested MatekF405/ArduCopter 4.5.7 baseline with RC9-controlled bidirectional brushed-gimbal output through an RZ7889 driver.

## 1. 分支用途

该分支是后续开发的稳定起点，主要用于：

- 保存已验证云台控制方案；
- 保存 MatekF405 的当前编译裁剪和传感器支持；
- 为独立 `MatekF405-UAV` 板级 Target 提供可回滚基线；
- 与无 SD 实验、罗盘修复和飞行算法实验隔离。

该分支不是最新 ArduPilot `master`，也不应直接与最新上游固件混刷。

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

当前保存的速度方案为：

```text
硬件载波频率：1000 Hz
开启阶段占空比：15%
密度开启：10 ms
密度停止：50 ms
自动回中：关闭
```

1 kHz PWM 由 STM32 硬件定时器产生；10/50 ms 只负责低速密度包络，不是用 GPIO 模拟 1 kHz PWM。

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
| `ArduCopter/UserCode.cpp` | RC9、M5/M6、1 kHz硬件PWM、密度调速和失效保护 |
| `ArduCopter/APM_Config.h` | 启用所需 UserHook |
| `ArduCopter/config.h` | 小容量F405功能选择 |
| `libraries/AP_HAL_ChibiOS/hwdef/MatekF405/hwdef.dat` | 当前板级GPIO、PWM、传感器及功能裁剪 |
| `BUILD_MatekF405_WSL2.md` | WSL2构建环境和命令 |
| `MATEKF405_*_ANALYSIS.md` | 云台、光流、起飞门限等设计分析 |

长期结构将把本机差异迁移到独立 `MatekF405-UAV` Target；在该迁移完成前，本分支作为原始可回滚基线保留，不继续重构。

## 6. 新手构建步骤

在 WSL2 Ubuntu 22.04 中：

```bash
cd /home/xk/ardupilot
mkdir -p /tmp/ardupilot-ccache /tmp/ardupilot-ccache-tmp

env PATH=/home/xk/ardupilot/venv/bin:/usr/lib/ccache:/usr/bin:/bin \
  CCACHE_DIR=/tmp/ardupilot-ccache \
  CCACHE_TEMPDIR=/tmp/ardupilot-ccache-tmp \
  venv/bin/python waf configure --board MatekF405

env PATH=/home/xk/ardupilot/venv/bin:/usr/lib/ccache:/usr/bin:/bin \
  CCACHE_DIR=/tmp/ardupilot-ccache \
  CCACHE_TEMPDIR=/tmp/ardupilot-ccache-tmp \
  venv/bin/python waf copter -j4
```

生成文件：

```text
build/MatekF405/bin/arducopter.apj
build/MatekF405/bin/arducopter.bin
build/MatekF405/bin/arducopter_with_bl.hex
```

Mission Planner 一般选择 `arducopter.apj`。刷写前应先导出当前参数，刷写后逐项确认接收机、光流、测距、罗盘、电池和云台，不能直接带桨飞行。

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
| `feature/matekf405-uav-target` | 开发/验证 | 独立 `MatekF405-UAV` 板级Target |

## 9. 安全与许可

- 这是特定硬件上的研究/实验固件，不代表 ArduPilot 官方支持；
- 首次刷写或参数变化后必须拆桨台架验证；
- 不删除 Arming、EKF、RC、电池、Crash、Thrust Loss 等安全检查；
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
