#!/usr/bin/env python3
from pathlib import Path

root = Path(__file__).resolve().parents[1]
user = (root / "ArduCopter/UserCode.cpp").read_text(encoding="utf-8")
arming = (root / "ArduCopter/AP_Arming.cpp").read_text(encoding="utf-8")
copter_h = (root / "ArduCopter/Copter.h").read_text(encoding="utf-8")
config = (root / "ArduCopter/APM_Config.h").read_text(encoding="utf-8")

required = [
    "#define AUTO_SOURCE_MANAGER_ENABLED 1",
    "#define AUTOSRC_GPS_SOURCE_SET 0U",
    "#define AUTOSRC_FLOW_SOURCE_SET 1U",
    "#define AUTOSRC_GPS_MIN_SATS 6U",
    "#define AUTOSRC_GPS_MAX_HDOP 250U",
    "#define AUTOSRC_GPS_RAW_HOLD_MS 3000U",
    "#define AUTOSRC_GPS_NAV_CONFIRM_MS 1000U",
    "#define AUTOSRC_GPS_HANDOVER_TIMEOUT_MS 5000U",
    "#define AUTOSRC_GPS_HANDOVER_RETRY_COOLDOWN_MS 15000U",
    "#define AUTOSRC_GPS_LOSS_HOLD_MS 2000U",
    "#define AUTOSRC_FLOW_MIN_QUALITY 50U",
    "#define AUTOSRC_FLOW_GROUND_NAV_TIMEOUT_MS 10000U",
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
    "autosrc_gps_retry_ready",
    "autosrc_last_handover_fail_ms = onekey_now_ms",
    "autosrc_takeoff_ready()",
    'autosrc_select_source(AUTOSRC_GPS_SOURCE_SET,',
    'autosrc_select_source(AUTOSRC_FLOW_SOURCE_SET, "GPS lost")',
    "loiter_nav->init_target();",
    "AutoSrc GPS handover complete",
    "AutoSrc Flow recovery complete",
    "AS S%u st%u G%uN%u C%u F%u T%u P%u%u q%u r%ld h%.0f",
]
for item in required:
    if item not in user:
        raise SystemExit(f"missing auto-source contract: {item}")

# Low altitude normally starts on Flow; GPS is the fallback, not an unconditional boot choice.
ground_i = user.index("if (!motors->armed() && ap.land_complete")
flow_i = user.index("if (autosrc_flow_ground_ready && !autosrc_flow_ground_suppressed)", ground_i)
gps_i = user.index("else if (autosrc_gps_raw_ready)", flow_i)
if not (ground_i < flow_i < gps_i):
    raise SystemExit("ground source priority must remain Flow -> GPS fallback")

# Ground bootstrap must accept valid near-zero compensated range samples.  NoData
# is still rejected by autosrc_range_valid; only the old >=4 cm gate is removed.
flow_ground_i = user.index("const bool autosrc_flow_ground_now")
flow_air_i = user.index("const bool autosrc_flow_air_now", flow_ground_i)
flow_ground_text = user[flow_ground_i:flow_air_i]
for item in ["autosrc_range_valid", "autosrc_range_cm >= 0", "AUTOSRC_FLOW_GROUND_MAX_CM"]:
    if item not in flow_ground_text:
        raise SystemExit(f"Flow ground readiness lost sanity check: {item}")
if "ONEKEY_LIFTOFF_ABORT_MAX_CM / 2.0f" in flow_ground_text:
    raise SystemExit("Flow ground readiness still contains the old 4 cm lower bound")

# If Flow sensor readiness exists but EKF relative aiding never establishes,
# GPS must be able to take over after a bounded wait. Repeated automatic
# Flow/GPS retry churn on the ground is suppressed until Flow actually drops
# or GPS is no longer viable.
for item in [
    "AUTOSRC_FLOW_GROUND_NAV_TIMEOUT_MS",
    '"Flow nav timeout"',
    "autosrc_flow_ground_suppressed = true",
    "if (!autosrc_flow_ground_now || !autosrc_gps_raw_ready)",
    "autosrc_flow_ground_ready && !autosrc_flow_ground_suppressed",
]:
    if item not in user:
        raise SystemExit(f"ground Flow navigation deadlock guard missing: {item}")
if "AUTOSRC_FLOW_GROUND_RETRY_COOLDOWN_MS" in user:
    raise SystemExit("timed ground Flow retry churn must remain disabled")

# A failed normal handover must not immediately chatter back to GPS.
if "AUTOSRC_GPS_HANDOVER_RETRY_COOLDOWN_MS" not in user:
    raise SystemExit("GPS handover retry cooldown missing")
if "autosrc_last_handover_fail_ms = onekey_now_ms" not in user:
    raise SystemExit("GPS handover timeout does not arm retry cooldown")

# Normal handover must be gated by GPS stability, altitude, low XY speed and Loiter.
gate_i = user.index("const bool normal_gps_gate")
gate_text = user[gate_i:gate_i+900]
for item in [
    "autosrc_gps_raw_ready",
    "autosrc_gps_retry_ready",
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

# Flow-loss emergency GPS acquisition is intentionally allowed to bypass the
# normal retry cooldown because it may be the only remaining horizontal aid.
emergency_i = user.index("const bool emergency_gps_gate")
emergency_text = user[emergency_i:emergency_i+500]
if "autosrc_gps_retry_ready" in emergency_text:
    raise SystemExit("Flow-loss emergency GPS gate must bypass normal retry cooldown")

# OneKey must now queue while waiting for automatic position rather than bypass checks.
for item in [
    "#define ONEKEY_POSITION_WAIT_TIMEOUT_MS 90000U",
    "WAIT_POSITION",
    "OneKey queued; waiting position",
    "else if (autosrc_takeoff_ready() && position_ok())",
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


for item in [
    "OneKey TO abort: source/position lost before takeoff",
    "OneKey TO abort: source/position lost during idle",
]:
    if item not in user:
        raise SystemExit(f"OneKey launch no longer rechecks AutoSource readiness: {item}")


# Manual Rudder arm and OneKey AUX arm must share the same final AutoSource gate.
if "bool autosrc_takeoff_ready();" not in copter_h:
    raise SystemExit("Copter does not expose unified AutoSource takeoff readiness")
for item in [
    "AP_Arming::Method::RUDDER",
    "AP_Arming::Method::AUXSWITCH",
    "copter.flightmode->mode_number() != Mode::Number::LOITER",
    "copter.autosrc_takeoff_ready()",
    "AutoSrc takeoff not ready",
]:
    if item not in arming:
        raise SystemExit(f"manual/OneKey arming AutoSource gate missing: {item}")

# This product gate is intentionally in AP_Arming_Copter::arm(), before the
# normal AP_Arming::arm() path, so optional ARMING_CHECK settings cannot bypass it.
arm_fn_i = arming.index("bool AP_Arming_Copter::arm(")
arm_fn = arming[arm_fn_i:arm_fn_i+5000]
gate_i = arm_fn.index("copter.autosrc_takeoff_ready()")
base_arm_i = arm_fn.index("if (!AP_Arming::arm(method, do_arming_checks))")
if gate_i > base_arm_i:
    raise SystemExit("AutoSource arm gate must run before AP_Arming::arm")

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
