# MatekF405 无SD/FATFS启动实验（ArduCopter 4.5.7）

> 当前分支：`test/matekf405-no-sd-fatfs`
>
> 实验标签：`test/matekf405-no-sd-fatfs-20260730`
>
> 父分支：`project/matekf405-gimbal-ap457`

这是一个公开的、可回滚的 **台架实验分支**。它在保留RC9云台控制、光流、测距、EKF3和其他当前功能的基础上，仅关闭 MatekF405 的 SPI SD 设备和 FATFS，用于定位“不插SD卡时启动仍然很慢”的原因。

> 该分支不是当前飞行稳定基线。关闭SD/FATFS后不会生成SD卡飞行日志，不应在完成替代日志方案前作为常规飞行固件。

## 1. 为什么建立这个分支

原始现象：MatekF405 在没有插入 SD 卡时，`Initialising ArduPilot` 持续数分钟。源码分析发现 ChibiOS BoardConfig 会反复尝试挂载 SPI SD/FATFS。

为避免把启动实验混入已验证云台基线，单独建立了本分支，只改变SD/FATFS相关板级定义。

## 2. 本分支改了什么

文件：

```text
libraries/AP_HAL_ChibiOS/hwdef/MatekF405/hwdef.dat
```

改动：

```text
不再注册名为 sdcard 的 SPI3 设备
不再定义 HAL_OS_FATFS_IO=1
生成阶段应得到 USE_FATFS=no
```

本分支没有删除：

- 云台RC9控制；
- M5/M6硬件PWM；
- 光流和测距；
- EKF3；
- MAVLink；
- 罗盘、气压计和IMU；
- Arming与飞行安全检查。

## 3. 已有测试结论

台架启动测试显示：

```text
原启动时间：约280秒
关闭SD/FATFS后：约155秒
缩短：约125秒
```

这证明SD/FATFS重试是启动延迟的一部分，但不是全部。

剩余约150秒已经进一步对应到 MAX7456 模拟OSD字体NVM Busy超时：

```text
MAX_NVM_WAIT = 10000
每次等待 = 15 ms
理论最大等待 = 150秒
```

因此本分支的结论是：

```text
SD/FATFS解释约125秒
MAX7456 OSD异常解释剩余约150秒
GPS、EKF和云台不是这段启动阻塞的根因
```

## 4. 云台功能是否还在

在。该实验分支继承已验证的云台基线：

```text
RC9 1420～1580：停止
RC9 > 1580：M5方向
RC9 < 1420：M6方向
硬件PWM：1000 Hz / 15%
密度调速：10 ms开启 / 50 ms停止
自动回中：关闭
RC失效：两路停止
```

SD/FATFS实验没有修改这条控制链路。

## 5. 新手构建步骤

该工作树示例路径为：

```bash
cd $HOME/ardupilot-worktrees/no-sd-fatfs
```

先做源码检查：

```bash
python3 Tools/scripts/verify_matekf405_nosd.py --source-only
```

再配置和编译：

```bash
mkdir -p /tmp/ardupilot-ccache /tmp/ardupilot-ccache-tmp

env PATH=$HOME/ardupilot/venv/bin:/usr/lib/ccache:/usr/bin:/bin \
  CCACHE_DIR=/tmp/ardupilot-ccache \
  CCACHE_TEMPDIR=/tmp/ardupilot-ccache-tmp \
  $HOME/ardupilot/venv/bin/python waf configure --board MatekF405

env PATH=$HOME/ardupilot/venv/bin:/usr/lib/ccache:/usr/bin:/bin \
  CCACHE_DIR=/tmp/ardupilot-ccache \
  CCACHE_TEMPDIR=/tmp/ardupilot-ccache-tmp \
  $HOME/ardupilot/venv/bin/python waf copter -j4
```

构建后验证生成配置：

```bash
python3 Tools/scripts/verify_matekf405_nosd.py
```

验证脚本会检查：

- 源码中没有活动的`SPIDEV sdcard`；
- 没有活动的`HAL_OS_FATFS_IO=1`；
- 生成的`hwdef.h`没有MMC-SPI/FATFS宏；
- 构建环境包含`USE_FATFS=no`。

## 6. 如何刷写和识别

生成固件：

```text
build/MatekF405/bin/arducopter.apj
build/MatekF405/bin/arducopter.bin
```

刷写后应重点确认：

1. 启动时间是否显著缩短；
2. Mission Planner是否不再显示文件日志能力；
3. RC9云台控制是否与稳定分支一致；
4. 光流、测距、罗盘和接收机是否仍正常；
5. `OSD_TYPE=1`时是否仍出现约150秒延迟。

## 7. 风险和已知限制

- 没有SD卡文件日志，事故后诊断能力明显下降；
- 模拟OSD异常仍可能造成约150秒启动等待；
- 该分支只做过台架启动验证，没有替代稳定分支；
- 不应同时修改GPS、EKF、OSD和SD后再比较启动时间，否则无法定位变量；
- 如果恢复SD功能，应回到稳定分支，不要手工拼接多组板级改动。

## 8. 回滚

返回稳定云台分支：

```bash
git switch project/matekf405-gimbal-ap457
```

或者从稳定标签重新建立分支：

```bash
git switch -c recovery/matekf405-gimbal baseline/matekf405-gimbal-sd-20260730
```

## 9. 分支关系

```text
ArduCopter 4.5.7 / 2a3dc4b7
        ↓
project/matekf405-gimbal-ap457   已验证云台稳定基线
        ↓
test/matekf405-no-sd-fatfs      仅关闭SD/FATFS的台架实验
```

## 10. 安全与许可

- 首次刷写必须拆桨测试；
- 无日志能力的实验固件不建议用于常规飞行；
- 本分支没有关闭ArduPilot飞行安全检查；
- ArduPilot及本派生工作遵循GNU GPL v3，完整许可见`COPYING.txt`。

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
