#!/usr/bin/env python3
from pathlib import Path

root = Path(__file__).resolve().parents[1]
user = (root / "ArduCopter/UserCode.cpp").read_text(encoding="utf-8")
config = (root / "ArduCopter/APM_Config.h").read_text(encoding="utf-8")

required = [
    "#define AUTO_SOURCE_MANAGER_ENABLED 1",
    "#define AUTOSRC_GPS_SOURCE_SET 0U",
    "#define AUTOSRC_FLOW_SOURCE_SET 1U",
    "#define AUTOSRC_GPS_RAW_HOLD_MS 3000U",
    "#define AUTOSRC_GPS_NAV_CONFIRM_MS 1000U",
    "#define AUTOSRC_GPS_HANDOVER_TIMEOUT_MS 5000U",
    "#define AUTOSRC_GPS_LOSS_HOLD_MS 2000U",
    "#define AUTOSRC_FLOW_MIN_QUALITY 50U",
    "#define AUTOSRC_FLOW_AIR_MAX_CM 250",
    "#define AUTOSRC_GPS_SWITCH_ALT_CM 150.0f",
    "#define AUTOSRC_GPS_SWITCH_MAX_XY_SPEED_CMS 60.0f",
    "AutoSourceState::GPS_HANDOVER",
    "AutoSourceState::FLOW_RECOVERY",
    "AP::ahrs().get_posvelyaw_source_set()",
    "set_posvelyaw_source_set(source_set)",
    "optflow.healthy()",
    "optflow.quality() >= AUTOSRC_FLOW_MIN_QUALITY",
    "get_rangefinder_height_interpolated_cm(autosrc_range_cm)",
    "autosrc_filter.flags.gps_quality_good",
    "autosrc_filter.flags.horiz_pos_abs",
    "autosrc_filter.flags.using_gps",
    "!autosrc_filter.flags.gps_glitching",
    "autosrc_alt_cm >= AUTOSRC_GPS_SWITCH_ALT_CM",
    "autosrc_xy_speed_cms <= AUTOSRC_GPS_SWITCH_MAX_XY_SPEED_CMS",
    'autosrc_select_source(AUTOSRC_GPS_SOURCE_SET,',
    'autosrc_select_source(AUTOSRC_FLOW_SOURCE_SET, "GPS lost")',
    "loiter_nav->init_target();",
    "AutoSrc GPS handover complete",
    "AutoSrc Flow recovery complete",
    "AS S%u st%u G%u F%u q%u r%ld h%.0f",
]
for item in required:
    if item not in user:
        raise SystemExit(f"missing auto-source contract: {item}")

# Low altitude normally starts on Flow; GPS is the fallback, not an unconditional boot choice.
ground_i = user.index("if (!motors->armed() && ap.land_complete")
flow_i = user.index("if (autosrc_flow_ground_ready)", ground_i)
gps_i = user.index("else if (autosrc_gps_raw_ready)", flow_i)
if not (ground_i < flow_i < gps_i):
    raise SystemExit("ground source priority must remain Flow -> GPS fallback")

# Normal handover must be gated by GPS stability, altitude, low XY speed and Loiter.
gate_i = user.index("const bool normal_gps_gate")
gate_text = user[gate_i:gate_i+900]
for item in [
    "autosrc_gps_raw_ready",
    "autosrc_alt_cm >= AUTOSRC_GPS_SWITCH_ALT_CM",
    "autosrc_xy_speed_cms <= AUTOSRC_GPS_SWITCH_MAX_XY_SPEED_CMS",
    "flightmode == &mode_loiter",
]:
    if item not in gate_text:
        raise SystemExit(f"normal GPS handover gate missing: {item}")

# Emergency GPS -> Flow degradation must be altitude-limited and Flow-ready.
fallback_i = user.index('autosrc_select_source(AUTOSRC_FLOW_SOURCE_SET, "GPS lost")')
fallback_text = user[max(0,fallback_i-1000):fallback_i+400]
for item in [
    "gps_failed",
    "autosrc_flow_air_ready",
    "AUTOSRC_FLOW_AIR_MAX_CM",
    "flightmode == &mode_loiter",
]:
    if item not in fallback_text:
        raise SystemExit(f"GPS emergency fallback guard missing: {item}")

# OneKey must now queue while waiting for automatic position rather than bypass checks.
for item in [
    "#define ONEKEY_POSITION_WAIT_TIMEOUT_MS 90000U",
    "WAIT_POSITION",
    "OneKey queued; waiting position",
    "else if (position_ok())",
    "set_mode(Mode::Number::LOITER, ModeReason::RC_COMMAND)",
    "arming.arm(AP_Arming::Method::AUXSWITCH, true)",
    "OneKey position ready; waiting motor spool",
]:
    if item not in user:
        raise SystemExit(f"OneKey source-aware queue missing: {item}")

aux_i = user.index("void Copter::userhook_auxSwitch1")
aux = user[aux_i:]
if "OneKey TO denied: no position" in aux:
    raise SystemExit("AUX edge still hard-denies no-position instead of queueing")
if "onekey_takeoff_state = OneKeyTakeoffState::WAIT_POSITION" not in aux:
    raise SystemExit("AUX HIGH does not queue WAIT_POSITION")

# Existing safety chain and hardware-tested gimbal must remain intact.
for item in [
    "#define ONEKEY_IDLE_HOLD_MS 1000U",
    "WAIT_SPOOL",
    "WAIT_IDLE_HOLD",
    "WAIT_LIFTOFF",
    "OneKey TO abort: no physical liftoff in 3s",
    "#define GIMBAL_RC7_INDEX 6U",
    "#define GIMBAL_HW_PWM_FREQUENCY_HZ 1000U",
    "#define GIMBAL_MANUAL_DUTY_PERCENT 15U",
    "#define GIMBAL_DENSITY_ON_MS 10U",
    "#define GIMBAL_DENSITY_OFF_MS 50U",
    "#define GIMBAL_AUTO_HOME_ENABLED 0",
]:
    if item not in user:
        raise SystemExit(f"regression in retained OneKey/gimbal contract: {item}")

for item in [
    "#define USERHOOK_INIT userhook_init();",
    "#define USERHOOK_FASTLOOP userhook_FastLoop();",
    "#define USERHOOK_50HZLOOP userhook_50Hz();",
    "#define USERHOOK_AUXSWITCH ENABLED",
]:
    if item not in config:
        raise SystemExit(f"required user hook missing: {item}")

print("AUTO_INDOOR_OUTDOOR_LOITER_STATIC_CHECK_PASS")
