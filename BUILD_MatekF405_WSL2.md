# WSL2 Ubuntu 22.04 编译 MatekF405 固件指南

本文记录当前仓库在 WSL2 Ubuntu 22.04 中编译 ArduPilot Copter 的 MatekF405 固件所需的最小环境、日常编译命令和常见问题处理。

适用场景：

- 当前源码目录：`$HOME/ardupilot`
- 系统：WSL2 Ubuntu 22.04
- 目标飞控板：`MatekF405`
- 目标固件：ArduCopter
- 使用仓库内 Python 虚拟环境：`venv/`

## 一、核心原则

为了避免影响 ROS、系统 Python 或 Windows PATH，建议遵守下面几点：

1. 不运行 ArduPilot 的全量环境安装脚本，除非明确需要 SITL、图形界面、MAVProxy 模块等完整开发环境。
2. 日常编译时显式使用仓库内的 `venv`。
3. 日常编译时显式设置 `PATH`，让 Linux 工具链和 `venv` 优先。
4. `ccache` 缓存放到 `/tmp/ardupilot-ccache`，不依赖 `/run/user/1000`。
5. 不用 `sudo` 运行 `waf`。
6. 不随意清理或还原源码改动，尤其是 `ArduCopter/` 和 `libraries/AP_HAL_ChibiOS/hwdef/MatekF405/` 下的本地修改。

## 二、已验证的环境

本次验证结果：

```bash
arm-none-eabi-gcc --version
```

版本：

```text
arm-none-eabi-gcc 10.3.1 20210621
```

```bash
ccache --version
```

版本：

```text
ccache 4.5.1
```

Python 虚拟环境：

```bash
$HOME/ardupilot/venv/bin/python
```

已验证 `venv` 中这些关键 Python 包可用：

```text
em
pymavlink
serial
pexpect
future
lxml
intelhex
```

## 三、首次配置最小依赖

如果是新 WSL2 环境，先安装最小依赖。

注意：这一步需要 `sudo`，只安装固件编译需要的包，不安装 ROS/SITL/GUI 扩展环境。

```bash
sudo apt-get update
```

```bash
sudo apt-get install -y \
  build-essential g++ gawk git make wget screen \
  python3-dev python3-pip python3-setuptools python3-venv python-is-python3 \
  libtool libtool-bin libxml2-dev libxslt1-dev \
  ccache \
  gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi
```

安装后可以清理 apt 下载缓存，释放空间：

```bash
sudo apt-get clean
```

说明：

- `gcc-arm-none-eabi` 是 STM32 固件交叉编译器。
- `binutils-arm-none-eabi` 提供 `objcopy`、`size` 等工具。
- `libnewlib-arm-none-eabi` 提供嵌入式 C 库。
- `ccache` 用于加速重复编译。
- `libxslt1-dev`、`libxml2-dev` 和 Python 包用于 MAVLink、脚本生成等构建步骤。

## 四、检查虚拟环境

进入源码目录：

```bash
cd $HOME/ardupilot
```

检查 `venv`：

```bash
venv/bin/python --version
```

检查关键包：

```bash
venv/bin/python -c "import em, pymavlink, serial, pexpect, future, lxml, intelhex; print('venv ok')"
```

如果输出：

```text
venv ok
```

说明当前 `venv` 可以继续使用。

## 五、首次重新配置 MatekF405

如果迁移过源码、换过系统、清理过 `build/`，建议重新配置一次：

```bash
cd $HOME/ardupilot
mkdir -p /tmp/ardupilot-ccache /tmp/ardupilot-ccache-tmp
```

```bash
env PATH=$HOME/ardupilot/venv/bin:/usr/lib/ccache:/usr/bin:/bin \
  CCACHE_DIR=/tmp/ardupilot-ccache \
  CCACHE_TEMPDIR=/tmp/ardupilot-ccache-tmp \
  venv/bin/python waf configure --board MatekF405
```

配置成功时会看到类似信息：

```text
Setting board to                         : MatekF405
Using toolchain                          : arm-none-eabi
Checking for 'g++' (C++ compiler)        : /usr/lib/ccache/arm-none-eabi-g++
Checking for 'gcc' (C compiler)          : /usr/lib/ccache/arm-none-eabi-gcc
CXX Compiler                             : g++ 10.3.1
'configure' finished successfully
```

## 六、日常编译命令

推荐以后都使用下面这条命令编译 Copter 固件：

```bash
cd $HOME/ardupilot
mkdir -p /tmp/ardupilot-ccache /tmp/ardupilot-ccache-tmp
env PATH=$HOME/ardupilot/venv/bin:/usr/lib/ccache:/usr/bin:/bin \
  CCACHE_DIR=/tmp/ardupilot-ccache \
  CCACHE_TEMPDIR=/tmp/ardupilot-ccache-tmp \
  venv/bin/python waf copter -j4
```

解释：

- `PATH=$HOME/ardupilot/venv/bin:...` 确保构建脚本调用的是 `venv` 里的 Python。
- `/usr/lib/ccache` 放在前面，让编译器通过 ccache 加速。
- `CCACHE_DIR=/tmp/ardupilot-ccache` 把缓存放到 `/tmp`，避免 WSL2 或沙箱环境中 `/run/user/1000` 只读导致失败。
- `-j4` 表示 4 线程编译，适合当前 4 核 WSL 环境。

## 七、固件输出位置

编译成功后，固件在：

```text
build/MatekF405/bin/
```

常用文件：

```text
build/MatekF405/bin/arducopter.apj
build/MatekF405/bin/arducopter.bin
build/MatekF405/bin/arducopter_with_bl.hex
```

一般用途：

- `arducopter.apj`：常用于 Mission Planner、MAVProxy 等方式刷写。
- `arducopter.bin`：裸固件二进制。
- `arducopter_with_bl.hex`：包含 bootloader 的 hex 文件。

本次成功编译的大小约为：

```text
arducopter.apj          841K
arducopter.bin          933K
arducopter_with_bl.hex  2.7M
```

本次构建摘要：

```text
Total Flash Used: 955123 B
Free Flash:        27908 B
```

## 八、清理构建缓存

如果只是普通修改代码，不建议频繁清理。直接重新编译即可，waf 会增量构建。

如果迁移环境后需要清理 MatekF405 的旧产物，可以只清理 MatekF405：

```bash
cd $HOME/ardupilot
rm -rf build/MatekF405 build/c4che/MatekF405_cache.py
```

然后重新执行：

```bash
env PATH=$HOME/ardupilot/venv/bin:/usr/lib/ccache:/usr/bin:/bin \
  CCACHE_DIR=/tmp/ardupilot-ccache \
  CCACHE_TEMPDIR=/tmp/ardupilot-ccache-tmp \
  venv/bin/python waf configure --board MatekF405
```

再编译：

```bash
env PATH=$HOME/ardupilot/venv/bin:/usr/lib/ccache:/usr/bin:/bin \
  CCACHE_DIR=/tmp/ardupilot-ccache \
  CCACHE_TEMPDIR=/tmp/ardupilot-ccache-tmp \
  venv/bin/python waf copter -j4
```

## 九、常见问题

### 1. ccache 报 `/run/user/1000/ccache-tmp` 只读

错误示例：

```text
ccache: error: Failed to create directory /run/user/1000/ccache-tmp: Read-only file system
```

原因：

`ccache` 默认临时目录在 `/run/user/1000`，某些 WSL2、沙箱或权限环境下该路径不可写。

解决：

编译时显式指定：

```bash
CCACHE_DIR=/tmp/ardupilot-ccache
CCACHE_TEMPDIR=/tmp/ardupilot-ccache-tmp
```

本文推荐命令已经包含这两个变量。

### 2. 最后生成 `arducopter_with_bl.hex` 时报 `No module named 'intelhex'`

错误示例：

```text
ModuleNotFoundError: No module named 'intelhex'
```

原因：

某些脚本通过 `/usr/bin/env python` 调用 Python。如果 `PATH` 没把 `venv/bin` 放在最前面，就可能调用系统 Python，而系统 Python 没有 `intelhex`。

解决：

确保编译命令中 `PATH` 以 `venv/bin` 开头：

```bash
PATH=$HOME/ardupilot/venv/bin:/usr/lib/ccache:/usr/bin:/bin
```

本文推荐命令已经包含这个设置。

### 3. 不要用 sudo 编译

不要这样做：

```bash
sudo ./waf copter
```

原因：

用 `sudo` 编译会让 `build/` 里的文件变成 root 所有，后续普通用户编译可能出现权限问题，也会污染 Python、PATH 和缓存环境。

### 4. Windows PATH 干扰

WSL2 默认可能把 Windows 的 PATH 加进来，例如 `/mnt/c/...`。这可能导致错误调用 Windows 程序。

解决：

日常编译使用本文推荐的 `env PATH=...` 命令，显式限制 PATH：

```bash
PATH=$HOME/ardupilot/venv/bin:/usr/lib/ccache:/usr/bin:/bin
```

### 5. ROS 环境隔离

本流程不会写入 `.bashrc`、`.profile`，也不会修改 ROS 环境变量。

如果你的终端已经 source 过 ROS，例如：

```bash
source /opt/ros/humble/setup.bash
```

也建议仍然使用本文推荐的 `env PATH=...` 编译命令，以减少干扰。

## 十、检查当前源码改动

编译前后可以看一下工作区状态：

```bash
git status --short
```

本次验证时存在本地改动，例如：

```text
M ArduCopter/APM_Config.h
M ArduCopter/UserCode.cpp
M ArduCopter/config.h
M libraries/AP_Compass/AP_Compass_QMC5883L.cpp
M libraries/AP_HAL_ChibiOS/hwdef/MatekF405/hwdef-bl.dat
M libraries/AP_HAL_ChibiOS/hwdef/MatekF405/hwdef.dat
?? ArduCopter/.cuav-lgc.cfg
?? libraries/GCS_MAVLink/include/
```

这些是源码工作区内容，编译环境配置不会自动修改或还原它们。

## 十一、完整推荐流程

最常用的流程如下。

进入目录：

```bash
cd $HOME/ardupilot
```

第一次或清理后配置：

```bash
mkdir -p /tmp/ardupilot-ccache /tmp/ardupilot-ccache-tmp
env PATH=$HOME/ardupilot/venv/bin:/usr/lib/ccache:/usr/bin:/bin \
  CCACHE_DIR=/tmp/ardupilot-ccache \
  CCACHE_TEMPDIR=/tmp/ardupilot-ccache-tmp \
  venv/bin/python waf configure --board MatekF405
```

编译：

```bash
env PATH=$HOME/ardupilot/venv/bin:/usr/lib/ccache:/usr/bin:/bin \
  CCACHE_DIR=/tmp/ardupilot-ccache \
  CCACHE_TEMPDIR=/tmp/ardupilot-ccache-tmp \
  venv/bin/python waf copter -j4
```

查看结果：

```bash
ls -lh build/MatekF405/bin/
```

看到以下文件即可：

```text
arducopter
arducopter.apj
arducopter.bin
arducopter_with_bl.hex
```

  更省事的日常写法
  你可以进入一次环境，然后后面就像以前一样用 ./waf：

  cd $HOME/ardupilot
  source venv/bin/activate
  mkdir -p /tmp/ardupilot-ccache /tmp/ardupilot-ccache-tmp
  export PATH="$VIRTUAL_ENV/bin:/usr/lib/ccache:/usr/bin:/bin"
  export CCACHE_DIR=/tmp/ardupilot-ccache
  export CCACHE_TEMPDIR=/tmp/ardupilot-ccache-tmp

  然后日常就可以：

  ./waf configure --board MatekF405
  ./waf copter -j4

  换板就是：

  ./waf configure --board Pixhawk6C
  ./waf copter -j4

  换机型就是：

  ./waf configure --board MatekF405-Wing
  ./waf plane -j4


# 以下是Pixhawk6X 硬件定义:
SERIAL_ORDER OTG1 UART7 UART5 USART1 UART8 USART2 UART4 USART3 OTG2

Mission Planner 参数	ArduPilot 串口	Pixhawk V6X 物理接口
SERIAL0	OTG1	USB
SERIAL1	UART7	TELEM1
SERIAL2	UART5	TELEM2
SERIAL3	USART1	GPS1
SERIAL4	UART8	GPS2
SERIAL5	USART2	TELEM3
SERIAL6	UART4	UART4
SERIAL7	USART3	Debug Console
SERIAL8	OTG2	第二 USB OTG，是否外露取决于硬件

不好意思，我刚才还没说完，以下是我的想法：
1、我的环境是WSL2，已经安装了ROS2-humble，和ign-gazebo（但是source进行了重命名为ign-gazebo-safe，需要你检查）
2、我主机没有显卡，但是有个A卡，不知道配置怎么样，如果能用起来加速仿真，使其不那么卡就更好了，这点需要你检查
3、我的目标是前期接入ROS，然后做做导航和目标检测，为后续真实无人机项目做准备，我目前研一，9月份研二要开题，方向还没定，后续老师可能会让我做“无人机空地协同”，我想着先做做仿真，了解一些
4、我的学习方式是先跑通一些demo，了解一些流程，然后再分模块去详细学习，感觉整体的框架懂了再去学习会好一些
5、我之前在VMware里面跑过虚拟机的仿真，但是仅仅限于接入ros用python控制一下常规的运动，还没有加入过传感器，比如搭载视觉相机进行巡检追踪和加载雷达进行导航之类的，我觉得这才有用吧
6、我有以下疑惑需要你解答：1、我记得ROS2对于无人机的通信不只是Mavlink或者说是Mavros，还有新的什么通信方式，我在纠结学习哪个，我还是觉得Mavros是否更经典一些，容易上手，开源资料也多，先了解一些熟悉后，后面再学习新型的。2、我需要你帮我制定一个学习方案和路线，也可以分模块，而不是上来就直接操作。3、如果后面在当前对话聊得多了，满了之后该如何移植到新的对话呢，或者是压缩上下文，当前这是后面担心的事情，我只是未雨绸缪

本次回答先只做分析与方案确定，后面再基于方案一个模块一个模块学习，如果你有好的思路或者是我没说到的，你也可以进行补充与发表你的意见