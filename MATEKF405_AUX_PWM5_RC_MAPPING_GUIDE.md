# MatekF405 使用辅助 PWM5 映射遥控器控制空心杯电机

本文针对当前需求做明确说明：使用 MatekF405 板子上的**物理第 5 路 PWM 输出**控制一个云台空心杯电机，并通过遥控器实现停止、正转、反转。

这里的目标不是把它加入多旋翼飞行电机混控，而是把 `PWM5/SERVO5` 当作一个普通辅助输出。

## 1. 先纠正关键概念

`SERVO5` 不是天然等于“多旋翼第 5 个电机”。

更准确地说：

- `SERVO5` 表示**物理第 5 路输出**。
- `SERVO5_FUNCTION` 决定这一路输出执行什么功能。
- 如果 `SERVO5_FUNCTION=37`，它才会变成多旋翼 `Motor5`。
- 如果 `SERVO5_FUNCTION=56`，它就是“跟随遥控器 RC6 输入”的普通辅助 PWM 输出。

所以我们当前要做的是：

```text
物理输出 5/SERVO5
    不配置成 Motor5
    配置成 RCIN6 或其他 RCINx
    让它跟随遥控器某个通道
```

## 2. 当前板子的 PWM5 是哪个引脚

当前源码文件：

```text
libraries/AP_HAL_ChibiOS/hwdef/MatekF405/hwdef.dat
```

里面定义：

```text
PA15 TIM2_CH1 TIM2 PWM(5) GPIO(54)
```

这说明：

- 物理 PWM5 对应 MCU 引脚 `PA15`。
- 在 ArduPilot 参数里对应 `SERVO5_...`。
- 这一路可以输出普通 PWM 信号，例如 1000us、1500us、2000us。

注意：这个引脚只是控制信号输出，不是电机电源输出。

## 3. 硬件连接方式

空心杯电机不能直接接飞控 PWM 输出脚。

正确结构应该是：

```text
遥控器
  -> 接收机
  -> 飞控 RC 输入
  -> ArduPilot 参数映射
  -> 飞控 SERVO5 信号输出
  -> 双向空心杯电机驱动/双向 brushed ESC
  -> 空心杯电机
```

推荐使用支持舵机 PWM 输入的双向电机驱动，例如：

```text
1000us  = 反转最大
1500us  = 停止
2000us  = 正转最大
```

不同电调方向可能相反，这是正常的，可以通过参数反向或交换电机线处理。

## 4. 推荐遥控器映射方案

建议使用遥控器 `RC6` 来控制云台电机。

原因：

- Copter 默认 `RC5` 是飞行模式通道，对应参数 `FLTMODE_CH=5`。
- 如果用 RC5 控制云台，可能会边控制电机边切换飞行模式。
- RC6、RC7、RC8 通常更适合作为辅助控制通道。

本文以 `RC6 -> SERVO5` 为例。

控制链路是：

```text
遥控器第 6 通道
  -> 接收机输出 RC6
  -> 飞控识别为 RC6
  -> SERVO5_FUNCTION = RCIN6
  -> 物理 PWM5 输出同样的 PWM
  -> 双向电调控制空心杯电机正反转
```

## 5. 需要设置的参数

### 5.1 最小推荐参数

```text
SERVO5_FUNCTION = 56
SERVO5_MIN      = 1000
SERVO5_TRIM     = 1500
SERVO5_MAX      = 2000
SERVO5_REVERSED = 0
SERVO_RATE      = 50
```

含义：

| 参数 | 含义 |
|---|---|
| `SERVO5_FUNCTION=56` | 物理输出 5 跟随遥控器 RC6 |
| `SERVO5_MIN=1000` | 输出下限 |
| `SERVO5_TRIM=1500` | 中位停止 |
| `SERVO5_MAX=2000` | 输出上限 |
| `SERVO5_REVERSED=0` | 输出方向不反向 |
| `SERVO_RATE=50` | 普通 PWM 输出频率 50Hz |

`56` 来自源码中的功能枚举：

```cpp
k_rcin6 = 56
```

对应关系：

| 遥控器输入 | `SERVO5` 输出 | 电机动作 |
|---|---|---|
| RC6 约 1500us | SERVO5 约 1500us | 停止 |
| RC6 高于 1500us | SERVO5 高于 1500us | 一个方向转 |
| RC6 低于 1500us | SERVO5 低于 1500us | 另一个方向转 |

### 5.2 RC6 校准参数

遥控器校准后，一般会得到类似：

```text
RC6_MIN  = 1000 左右
RC6_TRIM = 1500 左右
RC6_MAX  = 2000 左右
```

建议额外设置一点死区：

```text
RC6_DZ = 30
```

作用是避免遥控器中位轻微抖动导致电机慢慢转。

如果你的遥控器通道中位不是 1500us，要先在遥控器或地面站校准，让中位尽量接近 1500us。

## 6. Mission Planner 里怎么设置

### 6.1 遥控器端

1. 在遥控器里找一个旋钮、拨杆或回中摇杆。
2. 把它映射到接收机第 6 通道。
3. 在 Mission Planner 的 Radio Calibration 页面确认：
   - 摇动这个控件时，`Radio 6` 有变化。
   - 中位接近 1500。
   - 两端接近 1000 和 2000。

### 6.2 飞控参数端

在 Full Parameter List 或 Full Parameter Tree 中设置：

```text
SERVO5_FUNCTION = 56
SERVO5_MIN      = 1000
SERVO5_TRIM     = 1500
SERVO5_MAX      = 2000
SERVO5_REVERSED = 0
SERVO_RATE      = 50
RC6_DZ          = 30
```

写入参数后建议重启飞控。

### 6.3 检查输出

在 Servo Output 页面观察：

- 动遥控器 RC6。
- 看 `Servo 5` 是否跟着变化。
- 中位应该在 1500 附近。
- 两端应该接近 1000 和 2000。

确认输出正常后，再连接电机驱动。

## 7. 正反转方向不对怎么办

如果你希望“遥控器往上是正转”，但实际变成反转，可以用以下任一方式处理。

优先推荐：

```text
SERVO5_REVERSED = 1
```

其他方式：

- 在遥控器里反向 RC6 通道。
- 对普通有刷空心杯电机，交换电机两根线。

建议优先用 `SERVO5_REVERSED`，因为改动集中在飞控参数里，后续更容易记录和复现。

## 8. 为什么不用 `SERVO5_FUNCTION=37`

不要设置：

```text
SERVO5_FUNCTION = 37
```

因为 `37` 是：

```cpp
k_motor5 = 37
```

这表示多旋翼第 5 个飞行电机。

对四轴 Copter 来说，我们当前飞行混控只需要 Motor1 到 Motor4。把云台电机设置成 Motor5 会造成语义错误，也可能在以后改机架类型或调参时引入风险。

当前需求应该使用：

```text
SERVO5_FUNCTION = 56
```

也就是 `RCIN6`。

## 9. 如果想换成遥控器其他通道

只需要改 `SERVO5_FUNCTION`。

常用映射如下：

| 想用的遥控器输入 | `SERVO5_FUNCTION` | 含义 |
|---|---:|---|
| RC5 | 55 | 输出 5 跟随 RC5 |
| RC6 | 56 | 输出 5 跟随 RC6 |
| RC7 | 57 | 输出 5 跟随 RC7 |
| RC8 | 58 | 输出 5 跟随 RC8 |
| RC9 | 59 | 输出 5 跟随 RC9 |

不推荐用 RC5，除非你已经处理好 `FLTMODE_CH=5` 的飞行模式占用问题。

如果改用 RC7，例如：

```text
SERVO5_FUNCTION = 57
RC7_DZ          = 30
```

其余 `SERVO5_MIN/TRIM/MAX/REVERSED` 不变。

## 10. 安全和 failsafe 建议

### 10.1 接收机 failsafe

如果使用 `SERVO5_FUNCTION=56` 这种原始 RC 直通，建议在接收机里设置 failsafe：

```text
RC6 failsafe = 1500us
```

这样遥控器失联时，云台电机回到停止。

### 10.2 飞控急停

`RCIN6` 直通输出不是飞行主电机，不能完全等同于 Copter 主电机急停逻辑。

如果你要求“飞控锁定、急停、failsafe 时云台电机一定停止”，第一版参数直通可能不够严谨，后续应该做源码级保护逻辑。

### 10.3 初次测试

第一次测试建议：

1. 拆桨。
2. 空心杯电机单独限流供电。
3. 先不接电机，只用示波器、逻辑分析仪或地面站 Servo Output 看 `SERVO5` 输出。
4. 确认中位停止后再接电机。

## 11. 当前最推荐的实施步骤

第一步，在遥控器里把一个回中控件映射到 RC6。

第二步，在 Mission Planner 里确认 RC6 输入正常：

```text
低位约 1000
中位约 1500
高位约 2000
```

第三步，设置飞控参数：

```text
SERVO5_FUNCTION = 56
SERVO5_MIN      = 1000
SERVO5_TRIM     = 1500
SERVO5_MAX      = 2000
SERVO5_REVERSED = 0
SERVO_RATE      = 50
RC6_DZ          = 30
```

第四步，重启飞控。

第五步，在 Servo Output 页面确认 `Servo 5` 跟随 `RC6`。

第六步，接入双向空心杯电机驱动，测试：

```text
RC6 中位：停止
RC6 一侧：正转
RC6 另一侧：反转
```

第七步，如果方向反了：

```text
SERVO5_REVERSED = 1
```

## 12. 什么时候再考虑改源码

当前需求如果只是“遥控器控制空心杯电机正反转”，不需要改源码。

后续如果需要以下能力，再考虑源码方案：

- 飞控未解锁时强制云台电机不动。
- 遥控器失联时由飞控强制输出 1500。
- 云台电机软启动、限速、刹车。
- 用两个输出控制普通 H 桥。
- 根据飞行模式自动启停。
- 接入真正的云台姿态闭环控制。

当前最稳妥的第一版路线是参数映射：

```text
SERVO5_FUNCTION = 56
```

也就是：物理 PWM5 作为辅助输出，跟随遥控器 RC6。
