# MatekF405 Copter 4.5.7 电机 5 路控制云台小电机分析

本文基于当前目录内这份已修改的 ArduPilot 源码分析，目标是说明：

- 现在四个主电机是如何被 Copter 混控和输出的。
- 板子上看到的 `5/6` 输出到底是什么。
- 如果想用物理输出 5 控制一个云台空心杯电机，如何先用参数和遥控器映射实现正反转。
- 哪些情况必须改源码，哪些情况不需要改源码。

本文件只做分析和流程说明，不修改飞控源码。

## 1. 先区分三个概念

在 ArduPilot 里，容易把下面几个“5”混在一起：

1. **物理 PWM 输出 5**
   - 这是板子上的第 5 路输出引脚。
   - 在当前 `hwdef.dat` 中定义为 `PA15 TIM2_CH1 TIM2 PWM(5) GPIO(54)`。
   - 它对应参数名前缀通常是 `SERVO5_...`。

2. **Motor5 功能**
   - 这是 `SERVOx_FUNCTION=37`，代表“多旋翼第 5 个升力电机”。
   - 它不是“第 5 路输出随便拿来驱动小电机”的意思。
   - 四轴机不应该把云台小电机配置成 `Motor5`。

3. **遥控器输入通道 5**
   - 这是接收机传给飞控的第 5 个 RC 输入。
   - Copter 默认 `FLTMODE_CH=5`，也就是遥控器 5 通道默认用于切换飞行模式。
   - 因此建议不要直接用 RC5 控制云台，除非你明确改了飞行模式通道。

推荐表达方式：

- “我要用物理输出 5” = `SERVO5_...`
- “我要用遥控器 6 通道控制它” = `SERVO5_FUNCTION=RCIN6` 或 `RCIN6Scaled`

## 2. 当前 MatekF405 的 PWM 输出

当前板级文件：

```text
libraries/AP_HAL_ChibiOS/hwdef/MatekF405/hwdef.dat
```

关键输出定义：

```text
PC6  TIM3_CH1 TIM3 PWM(1) GPIO(50)
PC7  TIM8_CH2 TIM8 PWM(2) GPIO(51)
PC8  TIM8_CH3 TIM8 PWM(3) GPIO(52)
PC9  TIM8_CH4 TIM8 PWM(4) GPIO(53)
PA15 TIM2_CH1 TIM2 PWM(5) GPIO(54)
PA8  TIM1_CH1 TIM1 PWM(6) GPIO(55)
```

也就是说，硬件层面现在一共有 6 路 PWM 输出：

| ArduPilot 输出 | MCU 引脚 | 常见参数前缀 | 当前用途建议 |
|---|---|---|---|
| PWM(1) | PC6 | SERVO1 | 主电机 1 |
| PWM(2) | PC7 | SERVO2 | 主电机 2 |
| PWM(3) | PC8 | SERVO3 | 主电机 3 |
| PWM(4) | PC9 | SERVO4 | 主电机 4 |
| PWM(5) | PA15 | SERVO5 | 可作为云台/辅助输出 |
| PWM(6) | PA8 | SERVO6 | 可作为备用辅助输出 |

## 3. 四个主电机当前是怎么工作的

### 3.1 Copter 初始化输出

`ArduCopter/radio.cpp` 的 `Copter::init_rc_out()` 会初始化电机库：

```cpp
motors->init((AP_Motors::motor_frame_class)g2.frame_class.get(),
             (AP_Motors::motor_frame_type)g.frame_type.get());
SRV_Channels::enable_aux_servos();
motors->set_update_rate(g.rc_speed);
SRV_Channels::update_aux_servo_function();
```

这表示：

- 电机布局由 `FRAME_CLASS` 和 `FRAME_TYPE` 决定。
- 对于普通四轴，默认是 Quad + X。
- `SRV_Channels` 负责把 `SERVOx_FUNCTION` 映射到真实输出通道。

### 3.2 Quad X 只注册 4 个电机

`libraries/AP_Motors/AP_MotorsMatrix.cpp` 的 Quad X 分支只添加 4 个电机：

```cpp
case MOTOR_FRAME_TYPE_X: {
    static const AP_MotorsMatrix::MotorDef motors[] {
        {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
        { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
        {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
        {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
    };
    add_motors(motors, ARRAY_SIZE(motors));
    break;
}
```

这就是当前四个主电机能正常工作的核心原因。四轴混控只计算并输出 Motor1 到 Motor4。

### 3.3 Motor1 到 Motor4 的功能编号

`libraries/SRV_Channel/SRV_Channel.h` 中定义：

```cpp
k_motor1 = 33,
k_motor2 = 34,
k_motor3 = 35,
k_motor4 = 36,
k_motor5 = 37,
k_motor6 = 38,
```

所以：

- `SERVO1_FUNCTION=33` 表示这个物理输出承载 Motor1。
- `SERVO2_FUNCTION=34` 表示这个物理输出承载 Motor2。
- `SERVO5_FUNCTION=37` 表示这个物理输出承载 Motor5。

注意：`Motor5` 是“多旋翼混控中的第 5 个升力电机”，通常用于六轴/八轴，不是普通辅助电机。

### 3.4 每个控制周期如何输出

`ArduCopter/motors.cpp` 中 `Copter::motors_output()` 的关键顺序：

```cpp
SRV_Channels::calc_pwm();
SRV_Channels::cork();
SRV_Channels::output_ch_all();
...
motors->output();
```

简化理解：

1. `SRV_Channels::calc_pwm()` 根据各个 `SERVOx_FUNCTION` 算出输出 PWM。
2. `SRV_Channels::output_ch_all()` 处理辅助输出、遥控直通等。
3. 电机库根据姿态控制结果写主电机 PWM。

对于主电机，`AP_Motors::rc_write()` 会根据内部电机编号找到 `k_motor1/k_motor2/...` 功能，再写入对应 `SERVOx` 输出。

## 4. 为什么不建议用 `Motor5` 控制云台

如果把物理输出 5 设置为：

```text
SERVO5_FUNCTION = 37
```

含义是：输出 5 是多旋翼第 5 个升力电机。

这会带来几个问题：

- 当前 Quad X 混控只启用 4 个电机，不会把云台小电机当独立遥控器通道控制。
- `Motor5` 会被系统视为危险动力输出，进入电机停转、急停、安全开关等逻辑。
- 如果以后把机架类型改成 Hexa，Motor5 会变成飞行姿态控制的一部分，云台电机会干扰飞行。
- 它不提供“摇杆中位停止、上方正转、下方反转”的云台控制语义。

因此，云台小电机不要使用 `Motor5/Motor6` 功能编号，而应使用辅助输出功能，例如 `RCIN6` 或 `RCIN6Scaled`。

## 5. 当前源码中云台模块状态

当前 `hwdef.dat` 里有：

```text
define HAL_MOUNT_ENABLED 0
define HAL_CAMERA_ENABLED 0
```

这表示你这份小容量 F405 固件为了省空间已经关闭了 ArduPilot 原生 Mount/Camera 模块。

所以本轮目标不适合走 `MNT1_TYPE`、`MNT1_RC_RATE`、`SERVO5_FUNCTION=MountPitch/MountYaw` 这套完整云台框架。

更现实的路线是：

- 先把云台小电机当成“一个由遥控器通道直接控制的辅助 PWM 输出”。
- 如果后续需要限位、闭环姿态、跟随模式、按飞行状态联动，再考虑源码级功能。

## 6. 推荐的无源码修改方案

### 6.1 硬件前提

飞控 `SERVO5` 输出脚只是信号脚，不能直接给空心杯电机供电。

要实现正反转，外部驱动必须支持以下之一：

1. **一线式双向电调/双向 brushed ESC**
   - 输入一个舵机 PWM 信号。
   - 典型语义：`1500us` 停止，`1000us` 一个方向，`2000us` 另一个方向。
   - 这是最适合用 `SERVO5` 单路输出控制的硬件。

2. **带舵机 PWM 输入的 H 桥驱动**
   - 和双向电调类似，能直接理解 1000/1500/2000us。

3. **普通 H 桥：PWM + DIR 或 IN1 + IN2**
   - 这种通常需要两个飞控输出或一个 PWM 加一个 GPIO 方向脚。
   - 单独一个 `SERVO5` PWM 不够优雅，后续大概率需要改源码。

如果你现在只是把空心杯电机直接接到飞控输出脚，这是不行的。飞控输出脚没有驱动电机的电流能力，也不负责换向。

### 6.2 参数方案 A：物理输出 5 跟随遥控器 6 通道

假设：

- 云台电调信号线接飞控物理输出 5，也就是 `SERVO5`。
- 遥控器旋钮/三段开关/摇杆映射到接收机 `RC6`。
- 外部电调支持 `1000/1500/2000us` 双向控制。

建议参数：

```text
SERVO5_FUNCTION = 56     # RCIN6，物理输出5直接跟随遥控器输入6
SERVO5_MIN      = 1000
SERVO5_TRIM     = 1500
SERVO5_MAX      = 2000
SERVO5_REVERSED = 0      # 方向反了再改成 1
SERVO_RATE      = 50     # 普通舵机/很多双向电调用 50Hz；按你的电调说明调整

RC6_MIN         = 实测校准值
RC6_TRIM        = 1500
RC6_MAX         = 实测校准值
RC6_DZ          = 20~50  # 给中位附近留一点死区，具体看电调和遥控器抖动
```

对应功能编号来自源码：

```cpp
k_rcin6 = 56
```

这种方式最直接。`SRV_Channel::output_ch()` 遇到 `k_rcin1 ... k_rcin16` 时，会取对应 RC 输入的 `radio_in`，写到输出 PWM。

控制效果：

| 遥控器 RC6 输入 | SERVO5 输出 | 双向电调理解 |
|---|---|---|
| 约 1500us | 约 1500us | 停止 |
| 大于 1500us | 大于 1500us | 一个方向转 |
| 小于 1500us | 小于 1500us | 反方向转 |

方向反了的处理优先级：

1. 先改 `SERVO5_REVERSED=1`。
2. 或者在遥控器里反向 RC6。
3. 对 brushed 电机，交换电机两根线也会改变正反定义。

### 6.3 参数方案 B：使用 `RCIN6Scaled`

也可以用：

```text
SERVO5_FUNCTION = 145    # RCIN6Scaled
SERVO5_MIN      = 1000
SERVO5_TRIM     = 1500
SERVO5_MAX      = 2000
SERVO5_REVERSED = 0
SERVO_RATE      = 50
```

对应功能编号：

```cpp
k_rcin6_mapped = 145
```

`RCIN6Scaled` 会根据 RC 输入归一化后再映射到输出范围。相比 `RCIN6`，它更依赖 RC 通道类型、校准和输出 min/trim/max 的设置。

如果只是要“遥控器输入多少 PWM，输出就跟着多少 PWM”，优先用 `RCIN6`。

如果想让输出严格按 `SERVO5_MIN/TRIM/MAX` 重新映射，或者想利用 scaled passthrough 的 RC failsafe mask，再考虑 `RCIN6Scaled`。

### 6.4 RC failsafe 时输出怎么办

源码里 `SERVO_RC_FS_MSK` 只针对 `RCINxScaled` 这类 scaled passthrough：

```text
SERVO_RC_FS_MSK bit 5 = RCIN6Scaled
```

如果用 `SERVO5_FUNCTION=145`，并希望 RC failsafe 时输出回中位，可以设置：

```text
SERVO_RC_FS_MSK = 32
```

因为 RC6 对应 bit 5，十进制是 `1 << 5 = 32`。

如果用 `SERVO5_FUNCTION=56` 原始直通，建议同时在接收机 failsafe 中把 RC6 设置为 1500us，避免失控时保持最后一次非中位输出。

## 7. 如果你坚持用遥控器 5 通道

物理输出 5 跟随遥控器 5 通道的参数是：

```text
SERVO5_FUNCTION = 55     # RCIN5
```

但 Copter 默认：

```text
FLTMODE_CH = 5
```

这意味着 RC5 默认用于切换飞行模式。把同一个 RC5 又拿来控制云台，容易出现两个问题：

- 你转云台时同时切换飞行模式。
- ArduPilot 可能报 `FLTMODE_CH` 与 `RCx_OPTION` 或其他用途冲突。

更建议：

- 保留 RC5 做飞行模式。
- 用 RC6、RC7、RC8 作为云台控制输入。

如果确实要用 RC5，需要先重新规划飞行模式通道，例如把 `FLTMODE_CH` 改到 RC7/RC8，或者关闭飞行模式通道并使用其他方式切模式。这个改动涉及飞行安全，不建议作为第一步。

## 8. 调试流程

### 8.1 上电前

1. 拆桨。
2. 云台电机独立限流供电，先不要让它带负载高速转。
3. 确认电调或 H 桥输入信号地和飞控地共地。
4. 确认外部驱动支持你选择的信号协议。

### 8.2 在地面站看输入输出

1. 看 RC Input 页面：
   - 摇动遥控器云台控制通道。
   - 确认 `RC6` 或你选择的通道在 1000/1500/2000 附近变化。

2. 看 Servo Output 页面：
   - 设置 `SERVO5_FUNCTION=56` 后重启。
   - 摇动 RC6。
   - 确认 `SERVO5` 输出跟着变化。

3. 再接云台驱动：
   - 中位应该停止。
   - 一侧正转，另一侧反转。
   - 如果方向相反，改 `SERVO5_REVERSED`。

### 8.3 不建议用 Motor Test 测这个输出

Mission Planner 的 Motor Test 面向 `Motor1/Motor2/...` 主电机功能。你把 `SERVO5` 配成 `RCIN6` 后，它已经不是 Motor5，应该通过 RC 输入或 Servo Output 观察。

## 9. 什么时候需要改源码

下面这些需求，单靠参数可能不够：

1. **普通 H 桥需要两个控制脚**
   - 例如一个 PWM 控速度，一个 GPIO 控方向。
   - 或者 IN1/IN2 两路 PWM。
   - 这时需要在源码里读取 RC 输入，并分别控制两个输出。

2. **需要按解锁状态限制云台电机**
   - 例如未解锁禁止动、飞行中允许动、failsafe 强制停止。
   - 参数直通不会自动包含你自定义的业务逻辑。

3. **需要软启动、限速、死区、刹车、超时保护**
   - 可以在 `UserCode.cpp` 的 50Hz hook 中做。
   - 当前 `APM_Config.h` 已启用 `USERHOOK_50HZLOOP`，所以后续可以走这个入口。

4. **需要真正的云台姿态控制**
   - 例如跟随机体姿态补偿、角度闭环、MAVLink 云台命令。
   - 当前固件关闭了 `HAL_MOUNT_ENABLED`，需要重新启用 Mount 模块并评估 F405 flash 空间。

## 10. 第一版建议

最小可行方案：

```text
SERVO5_FUNCTION = 56
SERVO5_MIN      = 1000
SERVO5_TRIM     = 1500
SERVO5_MAX      = 2000
SERVO5_REVERSED = 0
SERVO_RATE      = 50
```

遥控器端：

- 用一个回中摇杆或带中位的旋钮映射到 RC6。
- 中位校准到 1500us。
- 接收机 failsafe 把 RC6 固定到 1500us。

硬件端：

- 使用支持舵机 PWM 输入的双向空心杯电机驱动或双向 brushed ESC。
- 飞控 `SERVO5` 只接信号线，电机电源走外部驱动。

这条路线不需要改源码，适合先验证“物理输出 5 能否由遥控器稳定控制正反转”。验证通过后，再决定是否需要做源码级的保护和高级逻辑。
