#!/usr/bin/env python3
from pathlib import Path

root = Path(__file__).resolve().parents[1]
user = (root / "ArduCopter/UserCode.cpp").read_text(encoding="utf-8")
config = (root / "ArduCopter/APM_Config.h").read_text(encoding="utf-8")
mode_h = (root / "ArduCopter/mode.h").read_text(encoding="utf-8")
takeoff = (root / "ArduCopter/takeoff.cpp").read_text(encoding="utf-8")

# ---- One-key takeoff/land contract (USER_FUNC1 intended on RC8) ----
required_onekey = [
    "intended for the self-centering RC8",
    "#define ONEKEY_RC_INPUT_TIMEOUT_MS 500U",
    "#define ONEKEY_SPOOL_READY_TIMEOUT_MS 5000U",
    "#define ONEKEY_IDLE_HOLD_MS 1000U",
    "#define ONEKEY_TAKEOFF_LIFTOFF_TIMEOUT_MS 3000U",
    "#define ONEKEY_LIFTOFF_CONFIRM_CM 12.0f",
    "#define ONEKEY_LIFTOFF_ABORT_MAX_CM 8.0f",
    "WAIT_SPOOL",
    "WAIT_IDLE_HOLD",
    "WAIT_LIFTOFF",
    "set_mode(Mode::Number::LOITER, ModeReason::RC_COMMAND)",
    "arming.arm(AP_Arming::Method::AUXSWITCH, true)",
    "AP_Motors::SpoolState::THROTTLE_UNLIMITED",
    "OneKey motor spool ready; idle hold 1s",
    "mode_loiter.do_user_takeoff_relative(onekey_pending_takeoff_alt_cm, true)",
    "get_rangefinder_height_interpolated_cm(range_alt_cm)",
    "range_alt_cm >= int32_t(ONEKEY_LIFTOFF_CONFIRM_CM)",
    "range_alt_cm <= int32_t(ONEKEY_LIFTOFF_ABORT_MAX_CM)",
    "OneKey TO liftoff confirmed",
    "OneKey TO abort: no physical liftoff in 3s",
    "set_mode(Mode::Number::LAND, ModeReason::RC_COMMAND)",
]
for item in required_onekey:
    if item not in user:
        raise SystemExit(f"missing integrated OneKey contract: {item}")

for forbidden in [
    "ONEKEY_STICK_CENTER_TOLERANCE_PWM",
    "onekey_channel_centered(",
    "onekey_takeoff_start_inertial_z_cm",
    "inertial_delta_cm",
]:
    if forbidden in user:
        raise SystemExit(f"obsolete OneKey logic remains: {forbidden}")

active_aux_define = any(
    line.strip().startswith("#define USERHOOK_AUXSWITCH ENABLED")
    for line in config.splitlines()
)
if not active_aux_define:
    raise SystemExit("USERHOOK_AUXSWITCH is not actively enabled")

if "bool do_user_takeoff_relative(float climb_alt_cm, bool must_navigate);" not in mode_h:
    raise SystemExit("relative takeoff API declaration missing")

for item in [
    "bool Mode::do_user_takeoff_relative(float climb_alt_cm, bool must_navigate)",
    "if (!copter.motors->armed())",
    "if (!copter.ap.land_complete)",
    "if (!has_user_takeoff(must_navigate))",
    "do_user_takeoff_start(climb_alt_cm)",
    "copter.set_auto_armed(true)",
]:
    if item not in takeoff:
        raise SystemExit(f"relative takeoff safety gate missing: {item}")

hook_i = user.index("void Copter::userhook_50Hz()")
aux_i = user.index("void Copter::userhook_auxSwitch1")
wait_spool_i = user.index("OneKeyTakeoffState::WAIT_SPOOL", hook_i)
idle_i = user.index("OneKeyTakeoffState::WAIT_IDLE_HOLD", hook_i)
idle_time_i = user.index("ONEKEY_IDLE_HOLD_MS", idle_i)
takeoff_i = user.index(
    "mode_loiter.do_user_takeoff_relative(onekey_pending_takeoff_alt_cm, true)",
    idle_i,
)
liftoff_i = user.index("OneKeyTakeoffState::WAIT_LIFTOFF", takeoff_i)
if not (hook_i < wait_spool_i < idle_i < idle_time_i < takeoff_i < liftoff_i < aux_i):
    raise SystemExit("OneKey sequence must be WAIT_SPOOL -> 1s IDLE_HOLD -> TAKEOFF -> WAIT_LIFTOFF")

aux_text = user[aux_i:]
if "mode_loiter.do_user_takeoff_relative(" in aux_text:
    raise SystemExit("AUX callback directly starts Takeoff; async sequencing regressed")
loiter_i = aux_text.index("set_mode(Mode::Number::LOITER, ModeReason::RC_COMMAND)")
arm_i = aux_text.index("arming.arm(AP_Arming::Method::AUXSWITCH, true)")
queue_i = aux_text.index("onekey_takeoff_state = OneKeyTakeoffState::WAIT_SPOOL")
if not (loiter_i < arm_i < queue_i):
    raise SystemExit("AUX sequence must remain LOITER -> ARM -> WAIT_SPOOL")

watchdog = user[hook_i:aux_i]
range_i = watchdog.index("get_rangefinder_height_interpolated_cm(range_alt_cm)")
confirm_i = watchdog.index("OneKey TO liftoff confirmed")
abort_i = watchdog.index("OneKey TO abort: no physical liftoff in 3s")
if not (range_i < confirm_i < abort_i):
    raise SystemExit("physical rangefinder liftoff watchdog ordering is invalid")

# ---- Stable density gimbal contract, moved from RC9 to RC7 only ----
required_gimbal = [
    "#define GIMBAL_RZ7889_RC7_CONTROL_ENABLED 1",
    "#define GIMBAL_RC7_INDEX 6U",
    "#define GIMBAL_RC_CENTER_PWM 1500U",
    "#define GIMBAL_RC_DEADZONE_PWM 80U",
    "#define GIMBAL_RC_INPUT_TIMEOUT_MS 500U",
    "#define GIMBAL_OUTPUT_OVERRIDE_TIMEOUT_MS 100U",
    "#define GIMBAL_HW_PWM_FREQUENCY_HZ 1000U",
    "#define GIMBAL_MANUAL_DUTY_PERCENT 15U",
    "#define GIMBAL_DENSITY_ON_MS 10U",
    "#define GIMBAL_DENSITY_OFF_MS 50U",
    "#define GIMBAL_AUTO_HOME_ENABLED 0",
    "static bool gimbal_read_rc7_pwm(uint16_t &rc_pwm)",
    "RC_Channel *rc7 = rc().channel(GIMBAL_RC7_INDEX);",
    "static void gimbal_update_manual_request_from_rc7()",
    "gimbal_density_output_enabled",
    "GimbalDrive::M5",
    "GimbalDrive::M6",
    "MODE_PWM_BRUSHED",
    "hal.rcout->set_freq(GIMBAL_PWM_CH_MASK, GIMBAL_HW_PWM_FREQUENCY_HZ)",
    'Gimbal HW PWM: 1000Hz 15%%, density 10/50ms',
    'Gimbal auto-home: OFF',
]
for item in required_gimbal:
    if item not in user:
        raise SystemExit(f"missing stable RC7 density-gimbal contract: {item}")

for forbidden in ["GIMBAL_RC9", "gimbal_read_rc9", "gimbal_update_manual_request_from_rc9"]:
    if forbidden in user:
        raise SystemExit(f"old RC9 gimbal reference remains: {forbidden}")

# Preserve mutual exclusion: M5 and M6 selection is an if/else-if chain.
m5_sel = """if ((gimbal_requested_drive == GimbalDrive::M5) &&
            (gimbal_requested_duty_percent > 0U))"""
m6_sel = """else if ((gimbal_requested_drive == GimbalDrive::M6) &&
                   (gimbal_requested_duty_percent > 0U))"""
if m5_sel not in user or m6_sel not in user:
    raise SystemExit("gimbal M5/M6 mutual-exclusion output chain changed")

# Both timing hooks must remain active on MatekF405-UAV.
for item in [
    "#define USERHOOK_INIT userhook_init();",
    "#define USERHOOK_FASTLOOP userhook_FastLoop();",
    "#define USERHOOK_50HZLOOP userhook_50Hz();",
]:
    if item not in config:
        raise SystemExit(f"required user hook missing: {item}")

print("RC8_ONEKEY_RC7_DENSITY_GIMBAL_STATIC_CHECK_PASS")
