# MatekF405 ArduCopter 4.5.7 起飞门控逻辑分析

本文档用于交给另一个 Codex 或开发者继续分析当前 ArduPilot/ArduCopter 源码中“已解锁但未起飞”的油门/起飞门控逻辑。

当前源码版本：

```text
Copter-4.5.7-dirty
```

当前关注对象：

```text
Board: MatekF405
Vehicle: ArduCopter
Use case: 250g 小型多旋翼，自回中油门杆，RC7 解锁
```

本文只做源码分析和方案设计，没有修改源码。

## 需求目标

目标行为分两个阶段：

### 阶段 1：已解锁但未起飞

- 电机只怠速转。
- 即使遥控器油门杆是自回中，油门通道处于中位，也不能直接起飞。
- 需要明确起飞动作后，才允许进入高度控制/升力输出。

### 阶段 2：起飞后

- 油门中位保持高度。
- 上推升高。
- 下拉降低。

当前已知现象：

- Stabilize 模式下解锁且油门在中位，会直接飞起来。
- 这是因为 Stabilize 是手动油门模式，中位油门会直接转成较高油门输出。
- 重点问题是 AltHold / Loiter 等高度控制模式是否默认已有 landed 状态下的起飞门控。

## 总体结论

ArduCopter 4.5.7 的 AltHold / Loiter 默认已经有“landed 状态下中位油门不触发起飞”的逻辑。

在 `ap.land_complete == true` 时：

- 油门中位会通过 `THR_DZ` 转换为 `target_climb_rate = 0`。
- `target_climb_rate = 0` 不满足起飞触发条件。
- 因此 AltHold / Loiter 不会进入 `AltHold_Takeoff`。
- 不会启动 `takeoff.start()`。
- 不会产生正爬升率目标。

但是，默认逻辑不是严格 DJI 式“必须二次确认后才允许起飞”的门控。

在 AltHold / Loiter 已解锁且 landed 时，如果油门中位或略高于最低但没有触发起飞，状态机会进入 `AltHold_Landed_Pre_Takeoff`，并将电机期望 spool 状态设为：

```cpp
AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED
```

这意味着电机会从 `GROUND_IDLE` 逐步预转到可起飞准备状态。正常情况下位置控制器会 `relax_z_controller(0.0f)`，不应产生起飞升力目标；但在小飞机上，如果 `MOT_SPIN_MIN` 设置过高，仍可能产生接近离地的推力。

所以：

- 如果只要求“中位油门不直接起飞”，AltHold / Loiter 默认逻辑基本满足。
- 如果要求“未明确触发前始终只怠速，禁止进入预起飞 spool”，默认逻辑不足，需要源码层最小改动。

## 关键源码证据

### 1. AltHold 模式逻辑

文件：

```text
ArduCopter/mode_althold.cpp
```

关键位置：

```text
ArduCopter/mode_althold.cpp:42
```

AltHold 每轮先读取油门并转成目标爬升率：

```cpp
float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
```

关键位置：

```text
ArduCopter/mode_althold.cpp:45
```

进入公共高度保持状态机：

```cpp
AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);
```

关键位置：

```text
ArduCopter/mode_althold.cpp:66
```

只有状态为 `AltHold_Takeoff` 时才启动起飞：

```cpp
case AltHold_Takeoff:
    if (!takeoff.running()) {
        takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
    }
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
    takeoff.do_pilot_takeoff(target_climb_rate);
    break;
```

关键位置：

```text
ArduCopter/mode_althold.cpp:61
```

落地/预起飞状态只重置积分并放松 Z 控制器：

```cpp
case AltHold_Landed_Pre_Takeoff:
    attitude_control->reset_rate_controller_I_terms_smoothly();
    pos_control->relax_z_controller(0.0f);
    break;
```

这说明 landed 且未触发 takeoff 时，不会给高度控制器正爬升目标。

### 2. Loiter 模式逻辑

文件：

```text
ArduCopter/mode_loiter.cpp
```

关键位置：

```text
ArduCopter/mode_loiter.cpp:108
```

Loiter 同样从油门输入得到目标爬升率：

```cpp
target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
```

关键位置：

```text
ArduCopter/mode_loiter.cpp:120
```

Loiter 也调用同一个公共状态机：

```cpp
AltHoldModeState loiter_state = get_alt_hold_state(target_climb_rate);
```

关键位置：

```text
ArduCopter/mode_loiter.cpp:145
```

只有状态为 `AltHold_Takeoff` 才进入起飞：

```cpp
case AltHold_Takeoff:
    if (!takeoff.running()) {
        takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
    }
    target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);
    takeoff.do_pilot_takeoff(target_climb_rate);
    loiter_nav->update();
    attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
    break;
```

关键位置：

```text
ArduCopter/mode_loiter.cpp:138
```

落地/预起飞状态同样放松 Z 控制器：

```cpp
case AltHold_Landed_Pre_Takeoff:
    attitude_control->reset_rate_controller_I_terms_smoothly();
    loiter_nav->init_target();
    attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);
    pos_control->relax_z_controller(0.0f);
    break;
```

### 3. 公共 AltHold 状态机

文件：

```text
ArduCopter/mode.cpp
```

关键位置：

```text
ArduCopter/mode.cpp:959
```

函数：

```cpp
Mode::AltHoldModeState Mode::get_alt_hold_state(float target_climb_rate_cms)
```

核心逻辑：

```cpp
if (!motors->armed()) {
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
    ...

} else if (takeoff.running() || takeoff.triggered(target_climb_rate_cms)) {
    return AltHold_Takeoff;

} else if (!copter.ap.auto_armed || copter.ap.land_complete) {
    if (target_climb_rate_cms < 0.0f && !copter.ap.using_interlock) {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    } else {
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    }

    if (motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
        return AltHold_Landed_Ground_Idle;
    } else {
        return AltHold_Landed_Pre_Takeoff;
    }

} else {
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    return AltHold_Flying;
}
```

重要含义：

- `takeoff.triggered(target_climb_rate_cms)` 为 true 时才进入 `AltHold_Takeoff`。
- landed 且没有触发起飞时，根据目标爬升率决定 `GROUND_IDLE` 或 `THROTTLE_UNLIMITED`。
- 如果 `target_climb_rate_cms < 0`，进入或保持 ground idle。
- 如果 `target_climb_rate_cms >= 0`，设定 `THROTTLE_UNLIMITED`，进入预起飞状态。
- 中位油门一般是 `target_climb_rate_cms == 0`，因此不会起飞，但会准备起飞。

### 4. 起飞触发条件

文件：

```text
ArduCopter/mode.cpp
```

关键位置：

```text
ArduCopter/mode.cpp:524
```

函数：

```cpp
bool Mode::_TakeOff::triggered(const float target_climb_rate) const
```

逻辑：

```cpp
if (!copter.ap.land_complete) {
    return false;
}
if (target_climb_rate <= 0.0f) {
    return false;
}
if (copter.motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
    return false;
}
return true;
```

起飞触发必须同时满足：

1. `ap.land_complete == true`
2. `target_climb_rate > 0`
3. `motors->get_spool_state() == THROTTLE_UNLIMITED`

因此油门中位不会触发起飞，因为中位油门在死区内会得到 `target_climb_rate = 0`。

### 5. 油门输入如何转为爬升率

文件：

```text
ArduCopter/Attitude.cpp
```

关键位置：

```text
ArduCopter/Attitude.cpp:60
```

函数：

```cpp
float Copter::get_pilot_desired_climb_rate(float throttle_control)
```

核心逻辑：

```cpp
throttle_control = constrain_float(throttle_control,0.0f,1000.0f);
g.throttle_deadzone.set(constrain_int16(g.throttle_deadzone, 0, 400));

const float mid_stick = get_throttle_mid();
const float deadband_top = mid_stick + g.throttle_deadzone;
const float deadband_bottom = mid_stick - g.throttle_deadzone;

if (throttle_control < deadband_bottom) {
    desired_rate = get_pilot_speed_dn() * (throttle_control-deadband_bottom) / deadband_bottom;
} else if (throttle_control > deadband_top) {
    desired_rate = g.pilot_speed_up * (throttle_control-deadband_top) / (1000.0f-deadband_top);
} else {
    desired_rate = 0.0f;
}
```

含义：

- 油门低于中位死区：负爬升率，下降。
- 油门在中位死区内：`0`，保持高度或地面待命。
- 油门高于中位死区：正爬升率，可触发起飞。

`THR_DZ` 不只是飞行后的高度保持死区，也参与地面起飞触发，因为起飞触发依赖 `target_climb_rate > 0`。

### 6. Stabilize 为什么中位油门会飞

文件：

```text
ArduCopter/mode_stabilize.cpp
```

关键位置：

```text
ArduCopter/mode_stabilize.cpp:21
```

Stabilize 是手动油门模式：

```cpp
if (!motors->armed()) {
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
} else if (copter.ap.throttle_zero || ...) {
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
} else {
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
}

float pilot_desired_throttle = get_pilot_desired_throttle();
...
attitude_control->set_throttle_out(pilot_desired_throttle, true, g.throttle_filt);
```

只要油门不是 0，Stabilize 会进入 `THROTTLE_UNLIMITED` 并直接输出手动油门。

文件：

```text
ArduCopter/mode.cpp
```

关键位置：

```text
ArduCopter/mode.cpp:915
```

函数：

```cpp
float Mode::get_pilot_desired_throttle() const
```

该函数把 RC 油门映射成 0 到 1 的油门输出，中位油门大约对应较高输出。对于 250g 小飞机，这可能直接超过悬停油门，因此 Stabilize 中位油门会直接起飞。

结论：

- 自回中油门杆不适合用 Stabilize 解锁待机。
- 起飞待机应使用 AltHold / Loiter / FlowHold 等非手动油门模式。

## 起飞辅助参数分析

### PILOT_TKOFF_ALT

定义位置：

```text
ArduCopter/Parameters.cpp:62
ArduCopter/config.h:338
```

说明：

```cpp
// Altitude that altitude control modes will climb to when a takeoff is triggered with the throttle stick.
GSCALAR(pilot_takeoff_alt, "PILOT_TKOFF_ALT", PILOT_TKOFF_ALT_DEFAULT)
```

默认：

```cpp
#define PILOT_TKOFF_ALT_DEFAULT 0
```

作用：

- 在 AltHold / Loiter / PosHold / Sport / FlowHold 等 user takeoff 模式中，当油门上推触发起飞时，`takeoff.start()` 使用该高度作为目标起飞高度。
- 单位是 cm。
- 代码中约束为 `0.0f` 到 `1000.0f`。

在 AltHold：

```text
ArduCopter/mode_althold.cpp:69
takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
```

在 Loiter：

```text
ArduCopter/mode_loiter.cpp:148
takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
```

结论：

- `PILOT_TKOFF_ALT=0` 时，油门中位不会触发起飞。
- `PILOT_TKOFF_ALT=0` 时，油门上推超过死区仍会触发起飞流程，但目标起飞高度增量为 0，主要依靠 pilot climb rate 推动起飞流程。
- `PILOT_TKOFF_ALT=100` 时，油门上推触发起飞后，会进入 user takeoff，目标爬升到约 100 cm。
- 对 250g 小飞机建议初始设置为 `50` 到 `100` cm。

### PILOT_THR_BHV

定义位置：

```text
ArduCopter/Parameters.cpp:71
ArduCopter/defines.h:149
```

参数说明：

```cpp
// Bitmask containing various throttle stick options.
// TX with sprung throttle can set PILOT_THR_BHV to "1" so motor feedback when landed starts from mid-stick instead of bottom of stick.
```

位定义：

```cpp
#define THR_BEHAVE_FEEDBACK_FROM_MID_STICK (1<<0)
#define THR_BEHAVE_HIGH_THROTTLE_CANCELS_LAND (1<<1)
#define THR_BEHAVE_DISARM_ON_LAND_DETECT (1<<2)
```

含义：

- bit0，值 `1`：自回中油门使用，落地时油门反馈从中位开始。
- bit1，值 `2`：高油门取消降落。
- bit2，值 `4`：检测到落地后自动 disarm。

结论：

- `PILOT_THR_BHV` 没有一个位直接实现“takeoff_allowed 严格门控”。
- 对自回中油门，建议设置 `PILOT_THR_BHV=1`。
- 如果希望落地自动 disarm，可考虑 `PILOT_THR_BHV=5`，但初期测试不建议先打开自动 disarm，以免混淆问题。

### THR_DZ

定义位置：

```text
ArduCopter/Parameters.cpp:243
ArduCopter/config.h:499
```

参数说明：

```cpp
// The deadzone above and below mid throttle in PWM microseconds. Used in AltHold, Loiter, PosHold flight modes
```

默认：

```cpp
#define THR_DZ_DEFAULT 100
```

作用：

- 决定自回中油门附近多大范围被认为是 `target_climb_rate = 0`。
- 飞行后影响“中位保持高度”。
- 地面 landed 时也影响起飞触发，因为 `takeoff.triggered()` 要求 `target_climb_rate > 0`。

建议：

- 自回中油门建议 `THR_DZ=100` 到 `150`。
- 如果遥控器中位抖动较大，可适当增加。
- 不建议过小，否则轻微中位偏移就可能触发正爬升率。

### MOT_SPIN_ARM

定义位置：

```text
libraries/AP_Motors/AP_MotorsMulticopter.cpp:118
```

参数说明：

```cpp
// Point at which the motors start to spin expressed as a number from 0 to 1 in the entire output range.
// Should be lower than MOT_SPIN_MIN.
```

作用：

- 解锁后 ground idle 怠速转动的目标。
- 对小飞机应设置为能稳定转动但不产生明显升力的最低值。

### MOT_SPIN_MIN

定义位置：

```text
libraries/AP_Motors/AP_MotorsMulticopter.cpp:111
libraries/AP_Motors/AP_Motors_Thrust_Linearization.cpp:46
```

参数说明：

```cpp
// Point at which the thrust starts expressed as a number from 0 to 1 in the entire output range.
// Should be higher than MOT_SPIN_ARM.
```

关键逻辑：

```text
libraries/AP_Motors/AP_Motors_Thrust_Linearization.cpp:99
```

```cpp
float Thrust_Linearization::thrust_to_actuator(float thrust_in) const
{
    thrust_in = constrain_float(thrust_in, 0.0, 1.0);
    return spin_min + (spin_max - spin_min) * apply_thrust_curve_and_volt_scaling(thrust_in);
}
```

含义：

- 在 `THROTTLE_UNLIMITED` 且控制器给 0 thrust 时，实际 actuator 输出仍可能是 `MOT_SPIN_MIN`。
- 小飞机上如果 `MOT_SPIN_MIN` 过高，即使没有正爬升率目标，也可能接近离地。

### MOT_THST_HOVER

定义位置：

```text
libraries/AP_Motors/AP_MotorsMulticopter.cpp:133
```

参数说明：

```cpp
// Motor thrust needed to hover expressed as a number from 0 to 1
```

作用：

- 高度控制模式中估算悬停所需推力。
- 影响飞行后油门中位保持高度的控制效果。
- 也影响 `get_non_takeoff_throttle()`，因为该函数返回约 hover throttle 的一半。

相关代码：

```text
ArduCopter/Attitude.cpp:101
```

```cpp
float Copter::get_non_takeoff_throttle()
{
    return MAX(0,motors->get_throttle_hover()/2.0f);
}
```

### MOT_HOVER_LEARN

定义位置：

```text
libraries/AP_Motors/AP_MotorsMulticopter.cpp:140
```

参数说明：

```cpp
// Enable/Disable automatic learning of hover throttle
```

默认：

```cpp
HOVER_LEARN_AND_SAVE
```

作用：

- 飞行中自动学习并保存悬停油门。
- 小飞机初期测试可以先手动估一个安全 `MOT_THST_HOVER`，确认稳定后再启用学习。

## 解锁后电机输出逻辑

### Spool 状态定义

文件：

```text
libraries/AP_Motors/AP_Motors_Class.h
```

关键位置：

```text
libraries/AP_Motors/AP_Motors_Class.h:172
```

状态含义：

```cpp
SHUT_DOWN = 0
GROUND_IDLE = 1
SPOOLING_UP = 2
THROTTLE_UNLIMITED = 3
SPOOLING_DOWN = 4
```

### GROUND_IDLE 输出

文件：

```text
libraries/AP_Motors/AP_MotorsMatrix.cpp
```

关键位置：

```text
libraries/AP_Motors/AP_MotorsMatrix.cpp:157
```

```cpp
case SpoolState::GROUND_IDLE:
    for (...) {
        set_actuator_with_slew(_actuator[i], actuator_spin_up_to_ground_idle());
    }
```

`actuator_spin_up_to_ground_idle()`：

```text
libraries/AP_Motors/AP_MotorsMulticopter.cpp:449
```

```cpp
return constrain_float(_spin_up_ratio, 0.0f, 1.0f) * thr_lin.get_spin_min();
```

在 `GROUND_IDLE` 且 desired 也是 `GROUND_IDLE` 时：

```text
libraries/AP_Motors/AP_MotorsMulticopter.cpp:582
```

```cpp
spin_up_armed_ratio = _spin_arm / thr_lin.get_spin_min();
_spin_up_ratio += constrain_float(spin_up_armed_ratio - _spin_up_ratio, ...);
```

因此 ground idle 通常对应 `MOT_SPIN_ARM` 附近。

### THROTTLE_UNLIMITED 输出

文件：

```text
libraries/AP_Motors/AP_MotorsMatrix.cpp
```

关键位置：

```text
libraries/AP_Motors/AP_MotorsMatrix.cpp:165
```

```cpp
case SpoolState::THROTTLE_UNLIMITED:
    set_actuator_with_slew(_actuator[i], thr_lin.thrust_to_actuator(_thrust_rpyt_out[i]));
```

如果 thrust 为 0，`thrust_to_actuator(0)` 也会输出 `MOT_SPIN_MIN`。

因此在 AltHold / Loiter landed 预起飞状态中，虽然控制器没有正爬升目标，但电机可能达到 `MOT_SPIN_MIN` 级别。

这就是默认逻辑和严格 DJI 式门控的区别。

## land_complete 状态切换

文件：

```text
ArduCopter/land_detector.cpp
```

关键位置：

```text
ArduCopter/land_detector.cpp:45
```

如果未 armed，强制 landed：

```cpp
if (!motors->armed()) {
    set_land_complete(true);
}
```

关键位置：

```text
ArduCopter/land_detector.cpp:48
```

如果当前 `ap.land_complete == true`，但非 takeoff 状态下电机输出已经高于非起飞油门阈值，并且 spool 状态是 `THROTTLE_UNLIMITED`，会清除 landed 标志并记录内部错误：

```cpp
if (!flightmode->is_taking_off() &&
    motors->get_throttle_out() > get_non_takeoff_throttle() &&
    motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED) {
    INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
    set_land_complete(false);
}
```

关键位置：

```text
ArduCopter/land_detector.cpp:117
```

落地检测条件满足一段时间后：

```cpp
set_land_complete(true);
```

结论：

- `land_complete` 是起飞门控的核心状态。
- 如果飞控误判已经 flying，即 `land_complete=false`，AltHold/Loiter 会进入 `AltHold_Flying`，高度控制正常输出油门。
- 误判 flying 时，中位油门会变成保持高度控制，而不是地面怠速逻辑。
- 因此必须打开 arming check、做好 IMU/气压计/震动/油门输出参数，避免状态误判。

## 解锁检查与自回中油门

文件：

```text
ArduCopter/AP_Arming.cpp
```

关键位置：

```text
ArduCopter/AP_Arming.cpp:618
```

非手动油门模式下，解锁前检查当前油门是否会产生正爬升率：

```cpp
if (copter.get_pilot_desired_climb_rate(copter.channel_throttle->get_control_in()) > 0.0f) {
    check_failed(ARMING_CHECK_RC, true, "%s too high", rc_item);
    return false;
}
```

含义：

- AltHold/Loiter 下，如果自回中油门在 `THR_DZ` 死区内，爬升率为 0，允许解锁。
- 如果中位偏高超过死区，解锁会失败并提示 throttle too high。
- 这也是必须打开 `ARMING_CHECK` 的原因之一。

文件：

```text
libraries/AP_Arming/AP_Arming.cpp
```

关键位置：

```text
libraries/AP_Arming/AP_Arming.cpp:802
```

底层 RC arming check 还可能要求 throttle channel neutral。实际行为取决于 RC arming check 配置。

## GCS Takeoff 入口

文件：

```text
ArduCopter/GCS_Mavlink.cpp
```

关键位置：

```text
ArduCopter/GCS_Mavlink.cpp:888
```

MAVLink `MAV_CMD_NAV_TAKEOFF` 调用：

```cpp
float takeoff_alt = packet.z * 100;
if (!copter.flightmode->do_user_takeoff(takeoff_alt, is_zero(packet.param3))) {
    return MAV_RESULT_FAILED;
}
return MAV_RESULT_ACCEPTED;
```

关键位置：

```text
ArduCopter/GCS_Mavlink.cpp:1056
```

部分 GCS 命令在 armed and landed 时会切到 Loiter 并调用：

```cpp
copter.flightmode->do_user_takeoff(packet.param1*100, true);
```

文件：

```text
ArduCopter/takeoff.cpp
```

关键位置：

```text
ArduCopter/takeoff.cpp:18
```

`do_user_takeoff()` 要求：

```cpp
motors->armed()
ap.land_complete == true
has_user_takeoff(must_navigate)
takeoff_alt_cm > current_loc.alt
```

不同模式是否支持 user takeoff：

```text
ArduCopter/mode.h:459   AltHold: return !must_navigate;
ArduCopter/mode.h:1228  Loiter: return true;
```

含义：

- Loiter 支持需要导航的 GCS takeoff。
- AltHold 只支持不要求导航的 user takeoff。

## 不改源码方案

如果目标是最接近 DJI，但暂时不改源码，推荐：

```text
起飞/待机模式：AltHold 或 Loiter
避免：Stabilize 解锁待机
```

推荐初始参数：

```text
PILOT_THR_BHV = 1
THR_DZ = 100 或 150
PILOT_TKOFF_ALT = 50 到 100
ARMING_CHECK = 1
MOT_SPIN_ARM = 能稳定转动但不产生明显升力的最低值
MOT_SPIN_MIN = 明显低于产生升力的值
MOT_THST_HOVER = 根据实测估计，初期不要离谱
MOT_HOVER_LEARN = 0 或 2
```

建议解释：

- `PILOT_THR_BHV=1`：适配自回中油门。
- `THR_DZ=100~150`：确保油门中位不会因抖动产生正爬升率。
- `PILOT_TKOFF_ALT=50~100`：油门上推后进入明确 user takeoff，先爬到 0.5m 到 1m。
- `ARMING_CHECK=1`：必须打开，防止油门偏高、传感器异常、GPS/罗盘/EKF/RC 异常时解锁。
- `MOT_SPIN_ARM/MOT_SPIN_MIN`：对 250g 小机特别关键，地面中位待机时不应产生可见升力。

不改源码时的行为：

1. AltHold / Loiter 解锁。
2. 油门回中。
3. `target_climb_rate = 0`，不触发 takeoff。
4. 电机可能处于预起飞 spool，通常不应离地。
5. 油门明显上推超过 `mid + THR_DZ`。
6. `target_climb_rate > 0`。
7. 当 spool 达到 `THROTTLE_UNLIMITED` 后触发 `AltHold_Takeoff`。
8. 进入 user takeoff，之后中位油门保持高度。

## 默认逻辑不足点

默认逻辑的问题不是“中位油门会直接触发 takeoff”，而是：

```text
land_complete=true + 中位油门
不会进入 Takeoff
但会把 desired spool state 设为 THROTTLE_UNLIMITED
```

这会让电机从 ground idle 进入预起飞状态，输出可能接近 `MOT_SPIN_MIN`。

如果用户要求非常严格：

```text
未触发 takeoff_allowed 前，电机必须始终只输出怠速
```

那么需要源码改动。

## 最小源码改动方案

目标：

```text
已解锁但未起飞时：
- 未触发 takeoff_allowed：电机只输出 GROUND_IDLE
- 油门中位不触发
- 油门从最低释放到中位不触发
- 只有明确触发后才允许进入 THROTTLE_UNLIMITED 和高度控制起飞流程
```

### 推荐改动位置

优先改公共逻辑：

```text
ArduCopter/mode.cpp
Mode::get_alt_hold_state()
```

原因：

- AltHold、Loiter、PosHold、Sport、FlowHold 等模式共用类似高度保持状态机。
- 不需要分别改 `mode_althold.cpp` 和 `mode_loiter.cpp`。
- 可以避免两个模式行为不一致。
- 可以限制只影响非手动油门的 user takeoff 模式，不影响 Stabilize、Acro、Auto、RTL、Land。

### 不建议只改 mode_althold.cpp

原因：

- Loiter 仍然会走原逻辑。
- FlowHold / PosHold / Sport 可能仍然行为不同。
- 未来维护容易遗漏。

### 是否新增参数

开发阶段建议先写死阈值测试：

```text
takeoff trigger threshold = get_throttle_mid() + max(g.throttle_deadzone, 100~150)
```

确认飞行行为后，再考虑新增参数，例如：

```text
PILOT_TKOFF_GATE
PILOT_TKOFF_THR
```

但上游式参数新增需要：

- `Parameters.cpp`
- `Parameters.h`
- 参数文档说明
- 默认值
- 参数转换兼容性评估

对自研板初期验证来说，先写死阈值更简单。

### 推荐触发条件评估

#### 条件 A：油门杆上推超过中位一定阈值

例：

```text
RC3 control_in > get_throttle_mid() + max(THR_DZ, 100~150)
```

优点：

- 不需要额外通道。
- 符合自回中油门直觉。
- 从最低释放到中位不会触发。
- 必须明确上推才触发。

缺点：

- 仍然是油门触发，不是独立起飞确认。

推荐度：最高，适合作为第一版最小改动。

#### 条件 B：油门从最低释放到中位不触发，必须先超过中位一定量

本质上和条件 A 相同。推荐采用。

#### 条件 C：RC7 解锁后，需要再拨一次或另一个 aux switch 触发 takeoff_allowed

优点：

- 最接近 DJI 的明确确认。
- 中位油门完全不会触发。

缺点：

- 需要设计 aux function 或复用已有通道逻辑。
- 用户操作复杂一些。
- 当前 `RC_Channel::AUX_FUNC::TAKEOFF` 在本地 Copter 代码中没有看到直接 case 处理，不能直接假设可用。

适合作为第二阶段增强。

#### 条件 D：GCS Takeoff 命令

优点：

- 当前已有代码支持。
- Mission Planner 起飞按钮或 MAVLink `NAV_TAKEOFF` 可用。
- 最少源码改动，甚至可以不改源码。

缺点：

- 不适合完全离线遥控器操作。
- AltHold 对 must_navigate 有限制，Loiter 更适合 GCS takeoff。

适合测试和地面验证。

#### 条件 E：设置 PILOT_TKOFF_ALT 后由现有 takeoff 流程触发

优点：

- 不改源码。
- 已有逻辑成熟。

缺点：

- 仍然是油门上推触发。
- 未触发前仍可能进入预起飞 spool。

适合作为当前不改源码方案。

### 推荐最小逻辑设计

伪代码示意：

```cpp
// 新增一个状态，例如 copter.ap.takeoff_allowed 或 Mode 内部静态/成员状态
// 起飞后、解锁后、落地后要正确复位

bool landed = !copter.ap.auto_armed || copter.ap.land_complete;
bool positive_takeoff_stick =
    channel_throttle->get_control_in() >
    (copter.get_throttle_mid() + MAX(g.throttle_deadzone, 150));

if (landed && !takeoff_allowed) {
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    return AltHold_Landed_Ground_Idle;
}

if (landed && positive_takeoff_stick) {
    takeoff_allowed = true;
}

if (takeoff_allowed && (takeoff.running() || takeoff.triggered(target_climb_rate_cms))) {
    return AltHold_Takeoff;
}
```

实际实现要注意：

- 不要在 `target_climb_rate == 0` 时设置 `THROTTLE_UNLIMITED`。
- 未 allow 前，强制 `GROUND_IDLE`。
- allow 后才允许进入原有 `takeoff.triggered()`。
- `takeoff_allowed` 在 `set_land_complete(true)` 或 disarm 时清零。
- GCS `do_user_takeoff()` 可以直接设置 allow。
- 如果用 aux switch，也设置 allow。

### 如何避免影响其他模式

建议只在以下条件同时成立时启用严格门控：

```text
mode has_manual_throttle() == false
ap.land_complete == true
mode supports user takeoff
不是 Auto / RTL / Land / Guided auto takeoff 正在执行
```

不要影响：

- Stabilize：手动油门模式，仍由用户直接控制。
- Acro：手动油门模式。
- Land：必须能下降和停机。
- RTL：自动返航流程不应被普通 pilot gate 卡住。
- Auto：任务 takeoff 应使用 AutoTakeoff 流程。
- Guided：GCS takeoff/companion control 应保留原逻辑或单独处理。

### 如何保证落地后重新回到 gated 状态

应在以下位置复位 `takeoff_allowed=false`：

1. disarm 后：

```text
ArduCopter/AP_Arming.cpp:832
```

这里 disarm 时调用：

```cpp
copter.set_land_complete(true);
copter.set_land_complete_maybe(true);
```

2. land detector 判定落地时：

```text
ArduCopter/land_detector.cpp:134
```

函数：

```cpp
void Copter::set_land_complete(bool b)
```

当 `b == true` 时清除起飞许可。

3. 模式切换时可考虑清除 takeoff running：

```text
ArduCopter/mode.cpp:437
old_flightmode->takeoff_stop();
```

已有逻辑会停止旧模式 takeoff。严格门控状态也应避免跨模式残留。

## 推荐地面测试步骤

### 不改源码测试

1. 拆桨。
2. 设置：

```text
PILOT_THR_BHV = 1
THR_DZ = 100 或 150
PILOT_TKOFF_ALT = 50 或 100
ARMING_CHECK = 1
```

3. 模式设为 AltHold。
4. RC7 解锁。
5. 油门杆回中。
6. 观察：

```text
ap.land_complete 应保持 true
不应进入 takeoff
电机只应低速旋转
```

7. 慢慢上推油门到中位死区以上。
8. 观察是否进入 takeoff 流程。
9. 切 Loiter 重复测试。
10. 切 Stabilize 测试时必须拆桨，因为中位油门会直接输出较高推力。

### 装桨前检查

1. 确认 `MOT_SPIN_ARM` 只是稳定怠速。
2. 确认 `MOT_SPIN_MIN` 不产生可见升力。
3. 确认油门中位没有正爬升率。
4. 确认 `ARMING_CHECK` 没被关闭。
5. 确认 RC3 中位稳定，`RC3_TRIM` 和遥控器校准正确。

### 首次装桨测试

1. 使用防护架或空旷室外。
2. 首次建议 `PILOT_TKOFF_ALT=50` 或 `100`。
3. 先 AltHold，后 Loiter。
4. 解锁后油门保持中位，确认不离地。
5. 缓慢上推油门，确认起飞动作可控。
6. 中位确认保持高度。
7. 下拉确认下降。
8. 随时准备切 Land 或 disarm。

### 如果准备测试源码严格门控

1. 拆桨。
2. 加日志或 GCS text 输出：

```text
takeoff_allowed=false
takeoff_allowed=true
gate holding ground idle
```

3. 验证：

```text
解锁 + 中位：始终 GROUND_IDLE
最低油门释放到中位：不触发
超过阈值：takeoff_allowed=true
进入原有 takeoff 流程
落地后：takeoff_allowed=false
```

4. 再装桨小油门测试。

## 最终建议

短期建议：

```text
不要先改源码。
先用 AltHold / Loiter + PILOT_THR_BHV=1 + THR_DZ=100~150 + PILOT_TKOFF_ALT=50~100 测试。
重点调低 MOT_SPIN_ARM 和 MOT_SPIN_MIN，保证待机不产生升力。
ARMING_CHECK 必须打开。
```

中期建议：

```text
如果仍想要 DJI 式严格门控，在 Mode::get_alt_hold_state() 做公共门控最小改动。
第一版用油门超过中位阈值触发 takeoff_allowed。
第二版再考虑新增 aux switch 或参数。
```

最关键判断：

```text
AltHold / Loiter 默认中位油门不会触发 takeoff。
但默认中位油门会进入预起飞 spool，不是严格怠速门控。
```
