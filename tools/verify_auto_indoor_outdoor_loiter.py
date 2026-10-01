#!/usr/bin/env python3
from pathlib import Path

root = Path(__file__).resolve().parents[1]
user = (root / "ArduCopter/UserCode.cpp").read_text(encoding="utf-8")
arming = (root / "ArduCopter/AP_Arming.cpp").read_text(encoding="utf-8")
copter_h = (root / "ArduCopter/Copter.h").read_text(encoding="utf-8")
avoid_cpp = (root / "libraries/AC_Avoidance/AC_Avoid.cpp").read_text(encoding="utf-8")
avoid_h = (root / "libraries/AC_Avoidance/AC_Avoid.h").read_text(encoding="utf-8")
loiter = (root / "ArduCopter/mode_loiter.cpp").read_text(encoding="utf-8")
mode_cpp = (root / "ArduCopter/mode.cpp").read_text(encoding="utf-8")
land_cpp = (root / "ArduCopter/mode_land.cpp").read_text(encoding="utf-8")
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
    "#define AUTOSRC_FLOW_HANDOVER_TIMEOUT_MS 5000U",
    "#define AUTOSRC_FLOW_HANDOVER_RETRY_COOLDOWN_MS 15000U",
    "#define AUTOSRC_FLOW_NAV_CONFIRM_MS 500U",
    "#define AUTOSRC_NORMAL_SOURCE_MIN_RESIDENCE_MS 5000U",
    "#define AUTOSRC_GPS_LOSS_HOLD_MS 2000U",
    "#define AUTOSRC_FLOW_MIN_QUALITY 50U",
    "#define AUTOSRC_FLOW_GROUND_NAV_TIMEOUT_MS 10000U",
    "#define AUTOSRC_FLOW_AIR_MAX_CM 250",
    "#define AUTOSRC_BARO_GROUND_SETTLE_MS 1000U",
    "#define AUTOSRC_GPS_SWITCH_BARO_CM 200.0f",
    "#define AUTOSRC_GPS_SWITCH_BARO_HOLD_MS 400U",
    "#define AUTOSRC_FLOW_SWITCH_BARO_CM 170.0f",
    "#define AUTOSRC_FLOW_SWITCH_BARO_HOLD_MS 500U",
    "#define AUTOSRC_FLOW_MAX_BARO_CM 230.0f",
    "#define AUTOSRC_GPS_SWITCH_MAX_XY_SPEED_CMS 60.0f",
    "#define AUTOSRC_FLOW_SWITCH_MAX_XY_SPEED_CMS 60.0f",
    "AutoSourceState::GPS_HANDOVER",
    "AutoSourceState::FLOW_HANDOVER",
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
    "autosrc_baro_switch_ready",
    "autosrc_baro_flow_switch_ready",
    "autosrc_xy_speed_cms <= AUTOSRC_GPS_SWITCH_MAX_XY_SPEED_CMS",
    "autosrc_gps_retry_ready",
    "autosrc_flow_retry_ready",
    "autosrc_source_residence_ready",
    "autosrc_last_handover_fail_ms = onekey_now_ms",
    "autosrc_takeoff_ready()",
    'autosrc_select_source(AUTOSRC_GPS_SOURCE_SET,',
    'autosrc_select_source(AUTOSRC_FLOW_SOURCE_SET, "GPS lost")',
    "loiter_nav->init_target();",
    "AutoSrc GPS handover complete",
    "AutoSrc Flow recovery complete",
    "AS S%u st%u G%uN%u C%u F%u T%u P%u%u q%u r%ld b%.0f",
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

# Both normal handover directions must use the same frozen relative barometric
# height, never EKF local Z. The 2.0/1.7 m thresholds create a 30 cm deadband.
for item in [
    "barometer.healthy()",
    "float(baro_alt) - autosrc_baro_ground_ref_cm",
    "AUTOSRC_BARO_GROUND_TRACK_ALPHA",
    "AUTOSRC_BARO_GROUND_SETTLE_MS",
    "AUTOSRC_GPS_SWITCH_BARO_CM",
    "AUTOSRC_GPS_SWITCH_BARO_HOLD_MS",
    "AUTOSRC_FLOW_SWITCH_BARO_CM",
    "AUTOSRC_FLOW_SWITCH_BARO_HOLD_MS",
    "autosrc_baro_switch_ready",
    "autosrc_baro_flow_switch_ready",
]:
    if item not in user:
        raise SystemExit(f"relative baro handover contract missing: {item}")
if "const float autosrc_alt_cm = inertial_nav.get_position_z_up_cm();" in user:
    raise SystemExit("source manager must not use EKF local Z as handover altitude")

# The optical-flow flight ceiling must use exactly the same frozen barometric
# ground reference as the AutoSource handover. It replaces only the native
# EKF Flow height wall; fence/proximity limits must remain available.
for item in [
    "AUTOSRC_FLOW_MAX_BARO_CM",
    "autosrc_baro_ground_ref_valid",
    "autosrc_baro_healthy",
    "(AUTOSRC_FLOW_MAX_BARO_CM - autosrc_baro_rel_cm) * 0.01f",
    "set_optflow_baro_height_limit(flow_baro_ceiling_valid",
    "autosrc_state == AutoSourceState::GPS_HANDOVER",
]:
    if item not in user:
        raise SystemExit(f"baro Flow ceiling producer missing: {item}")

for item in [
    "void set_optflow_baro_height_limit(bool valid, float alt_diff_m);",
    "_optflow_baro_height_limit_valid",
    "_optflow_baro_height_alt_diff_m",
    "_optflow_baro_height_limit_update_ms",
]:
    if item not in avoid_h:
        raise SystemExit(f"baro Flow ceiling interface missing: {item}")

for item in [
    "OPTFLOW_BARO_HEIGHT_LIMIT_TIMEOUT_MS = 250U",
    "optflow_baro_limit_fresh",
    "_optflow_baro_height_alt_diff_m",
    "(_enabled == AC_AVOID_DISABLED) && !optflow_baro_limit_fresh",
    "_ahrs.get_hgt_ctrl_limit(alt_limit)",
    "_ahrs.get_relative_position_D_origin(curr_alt)",
]:
    if item not in avoid_cpp:
        raise SystemExit(f"baro Flow ceiling/fallback contract missing: {item}")

external_i = avoid_cpp.index("if (optflow_baro_limit_fresh)")
native_i = avoid_cpp.index("_ahrs.get_hgt_ctrl_limit(alt_limit)", external_i)
if external_i > native_i:
    raise SystemExit("fresh baro Flow ceiling must take precedence over native EKF height limit")

# Do not solve the Flow ceiling by disabling the whole avoidance chain.
for item in [
    "AC_AVOID_STOP_AT_FENCE",
    "proximity_avoidance_enabled()",
    "get_upward_distance(proximity_alt_diff)",
]:
    if item not in avoid_cpp:
        raise SystemExit(f"fence/proximity vertical protection regressed: {item}")

gate_i = user.index("const bool normal_gps_gate")
gate_text = user[gate_i:gate_i+1000]
for item in [
    "autosrc_gps_raw_ready",
    "autosrc_gps_retry_ready",
    "autosrc_source_residence_ready",
    "autosrc_baro_switch_ready",
    "autosrc_xy_speed_cms <= AUTOSRC_GPS_SWITCH_MAX_XY_SPEED_CMS",
    "flightmode == &mode_loiter",
]:
    if item not in gate_text:
        raise SystemExit(f"normal GPS handover gate missing: {item}")

# Normal healthy GPS->Flow is now intentionally enabled below 1.7 m, with
# Flow readiness, low speed, minimum residence and a retry cooldown.
flow_gate_i = user.index("const bool normal_flow_gate")
flow_gate_text = user[flow_gate_i:flow_gate_i+1200]
for item in [
    "!gps_failed",
    "autosrc_gps_nav_now",
    "autosrc_flow_air_ready",
    "autosrc_flow_retry_ready",
    "autosrc_source_residence_ready",
    "autosrc_baro_flow_switch_ready",
    "autosrc_xy_speed_cms <= AUTOSRC_FLOW_SWITCH_MAX_XY_SPEED_CMS",
    "flightmode == &mode_loiter",
]:
    if item not in flow_gate_text:
        raise SystemExit(f"normal Flow handover gate missing: {item}")

if 'autosrc_select_source(AUTOSRC_FLOW_SOURCE_SET, "low-alt Flow")' not in user:
    raise SystemExit("normal GPS->Flow request is missing")

# FLOW_HANDOVER confirmation is intentionally relaxed after flight testing:
# an EKF that has already established a GPS origin may keep absolute/GPS flags
# valid even after SRC2 is selected. Success therefore requires the selected
# source-set to be SRC2, usable relative navigation, Flow readiness and a short
# confirmation hold; it must NOT require horiz_pos_abs/using_gps to clear.
flow_handover_i = user.index("AutoSourceState::FLOW_HANDOVER")
flow_handover_logic_i = user.index("} else if (autosrc_state == AutoSourceState::FLOW_HANDOVER)", flow_handover_i)
flow_handover_text = user[flow_handover_logic_i:flow_handover_logic_i+4700]
for item in [
    "autosrc_active_set == AUTOSRC_FLOW_SOURCE_SET",
    "flow_handover_filter.flags.horiz_pos_rel",
    "!flow_handover_filter.flags.const_pos_mode",
    "autosrc_flow_air_ready",
    "AUTOSRC_FLOW_NAV_CONFIRM_MS",
    "AUTOSRC_FLOW_HANDOVER_TIMEOUT_MS",
    "AUTOSRC_FLOW_LOSS_HOLD_MS",
    "autosrc_last_flow_handover_fail_ms = onekey_now_ms",
    'autosrc_select_source(AUTOSRC_GPS_SOURCE_SET,',
    "loiter_nav->init_target();",
    "AutoSrc Flow handover complete",
]:
    if item not in flow_handover_text:
        raise SystemExit(f"Flow handover confirmation/rollback missing: {item}")
for forbidden in [
    "!flow_handover_filter.flags.horiz_pos_abs",
    "!flow_handover_filter.flags.using_gps",
]:
    if forbidden in flow_handover_text:
        raise SystemExit(f"Flow handover is still over-constrained: {forbidden}")

# Emergency FLOW_RECOVERY uses the same source-set + relative-navigation
# completion principle so retained GPS capability flags cannot leave recovery
# stuck forever after an absolute origin has existed.
recovery_i = user.index("if (autosrc_state == AutoSourceState::FLOW_RECOVERY)")
recovery_text = user[recovery_i:recovery_i+1700]
for item in [
    "recovery_flow_nav_ok",
    "AUTOSRC_FLOW_SOURCE_SET",
    "recovery_filter.flags.horiz_pos_rel",
    "!recovery_filter.flags.const_pos_mode",
    "loiter_nav->init_target();",
    "AutoSrc Flow recovery complete",
]:
    if item not in recovery_text:
        raise SystemExit(f"Flow recovery completion contract missing: {item}")
for forbidden in [
    "!recovery_filter.flags.horiz_pos_abs",
    "!recovery_filter.flags.using_gps",
]:
    if forbidden in recovery_text:
        raise SystemExit(f"Flow recovery is still over-constrained: {forbidden}")

# The special FLOW_HANDOVER state must not be swallowed by the generic active
# Flow branch after set_posvelyaw_source_set() immediately changes source_set.
flow_active_i = user.index("if ((autosrc_active_set == AUTOSRC_FLOW_SOURCE_SET)")
flow_active_head = user[flow_active_i:flow_active_i+350]
if "autosrc_state != AutoSourceState::FLOW_HANDOVER" not in flow_active_head:
    raise SystemExit("generic Flow branch can overwrite FLOW_HANDOVER state")

# Emergency GPS -> Flow degradation remains separate from normal low-alt Flow
# handover and must bypass altitude hysteresis, residence and retry cooldown.
fallback_i = user.index('autosrc_select_source(AUTOSRC_FLOW_SOURCE_SET, "GPS lost")')
fallback_text = user[max(0,fallback_i-1300):fallback_i+500]
for item in [
    "gps_failed",
    "autosrc_flow_air_ready",
    "flightmode == &mode_loiter",
]:
    if item not in fallback_text:
        raise SystemExit(f"GPS emergency fallback guard missing: {item}")
for forbidden in [
    "autosrc_baro_flow_switch_ready",
    "autosrc_flow_retry_ready",
    "autosrc_source_residence_ready",
    "AUTOSRC_FLOW_SWITCH_MAX_XY_SPEED_CMS",
]:
    emergency_i = user.index("const bool emergency_flow_gate")
    emergency_text = user[emergency_i:emergency_i+500]
    if forbidden in emergency_text:
        raise SystemExit(f"emergency GPS->Flow must bypass normal gate: {forbidden}")

# No source transition may depend on the broken EKF local-Z altitude.
if "autosrc_alt_cm <= float(AUTOSRC_FLOW_AIR_MAX_CM)" in user:
    raise SystemExit("Flow fallback still depends on EKF local-Z altitude")

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

# Low-altitude throttle landing contract.
for item in [
    "#define LOWALT_FLOOR_ENTER_CM 50",
    "#define LOWALT_HARD_MIN_CM 40",
    "#define LOWALT_FLOOR_RELEASE_CM 60",
    "#define LOWALT_RANGE_CONFIRM_MS 200U",
    "#define LOWALT_HARD_STOP_HOLD_MS 250U",
    "#define LOWALT_THROTTLE_MIN_CONTROL 50",
    "#define LOWALT_THROTTLE_LAND_HOLD_MS 1000U",
    "#define LOWALT_LAND_MAX_XY_SPEED_CMS 50.0f",
    "#define LOWALT_RANGE_FRESH_MS 350U",
    "#define LOWALT_PREDICT_TARGET_CM 45.0f",
    "#define LOWALT_PREDICT_SENSOR_LATENCY_MS 120U",
    "#define LOWALT_PREDICT_MARGIN_CM 3.0f",
    "#define LOWALT_PREDICT_ACCEL_SCALE 0.70f",
    "#define LOWALT_PREDICT_MIN_ACCEL_CMSS 50.0f",
    "#define LOWALT_PREDICT_MAX_TRIGGER_CM 100.0f",
    "rangefinder.last_reading_ms(ROTATION_PITCH_270)",
    "rangefinder_state.alt_cm",
    "rangefinder_alt_ok()",
    "lowalt_landing_state = LowAltLandingState::HOLD",
    "range_cm > LOWALT_FLOOR_RELEASE_CM",
    "(range_fresh && (range_cm <= LOWALT_HARD_MIN_CM))",
    "stopping_distance_cm",
    "sensor_travel_cm",
    "predicted_trigger_cm",
    "LOWALT_PREDICT_ACCEL_SCALE",
    "LOWALT_PREDICT_MAX_TRIGGER_CM",
    "if ((floor_active || hard_stop_active || predictive_stop_active)",
    "target_climb_rate = 0.0f",
    "channel_throttle->get_control_in() <= LOWALT_THROTTLE_MIN_CONTROL",
    "LOWALT_LAND_MAX_XY_SPEED_CMS",
    "LOWALT_THROTTLE_LAND_HOLD_MS",
    "lowalt_land_request_pending = true",
    "LowAlt LAND request queued",
    "LowAlt throttle LAND started",
]:
    if item not in user:
        raise SystemExit(f"low-alt throttle LAND contract missing: {item}")

if "void low_alt_landing_guard(float &target_climb_rate);" not in copter_h:
    raise SystemExit("low-alt guard must be a non-transitioning void helper")

# LAND confirmation must require a current valid low-alt range. A latched floor
# may survive NoData, but NoData must never be enough to queue LAND.
land_height_i = user.index("const bool land_height_confirmed")
land_height_text = user[land_height_i:land_height_i+500]
for item in ["range_fresh", "range_cm >= 0", "range_cm <= LOWALT_FLOOR_RELEASE_CM"]:
    if item not in land_height_text:
        raise SystemExit(f"low-alt LAND current-range gate missing: {item}")

# ModeLoiter may only clamp/queue. It must never change flight mode or return
# early from the controller because the previous implementation caused a
# flow_of_control internal error immediately after LAND transition.
guard_start = user.index("void Copter::low_alt_landing_guard(float &target_climb_rate)")
guard_end = user.index("bool Copter::autosrc_takeoff_ready()", guard_start)
guard_text = user[guard_start:guard_end]
if "set_mode(Mode::Number::LAND" in guard_text:
    raise SystemExit("low-alt guard must not change mode inside ModeLoiter control stack")
if "lowalt_land_request_pending = true" not in guard_text:
    raise SystemExit("low-alt guard does not queue deferred LAND request")

for item in [
    "if (loiter_state != AltHold_Flying)",
    "copter.low_alt_landing_guard_reset();",
    "copter.low_alt_landing_guard(target_climb_rate);",
]:
    if item not in loiter:
        raise SystemExit(f"Loiter low-alt integration missing: {item}")
flying_i = loiter.index("case AltHold_Flying:")
guard_i = loiter.index("copter.low_alt_landing_guard(target_climb_rate);", flying_i)
if guard_i < flying_i:
    raise SystemExit("low-alt landing guard must run only in AltHold_Flying")
guard_tail = loiter[guard_i:guard_i+250]
if "return;" in guard_tail:
    raise SystemExit("Loiter must not early-return after low-alt guard")

# The 50 Hz manager owns the actual LAND transition and must revalidate every
# gate immediately before set_mode().
hook_i = user.index("if (lowalt_land_request_pending)")
hook_text = user[hook_i:hook_i+2200]
for item in [
    "rangefinder.last_reading_ms(ROTATION_PITCH_270)",
    "lowalt_landing_state == LowAltLandingState::HOLD",
    "lowalt_range_fresh",
    "lowalt_range_cm <= LOWALT_FLOOR_RELEASE_CM",
    "channel_throttle->get_control_in() <= LOWALT_THROTTLE_MIN_CONTROL",
    "LOWALT_LAND_MAX_XY_SPEED_CMS",
    "flightmode == &mode_loiter",
    "set_mode(Mode::Number::LAND, ModeReason::RC_COMMAND)",
    "low_alt_landing_guard_reset();",
]:
    if item not in hook_text:
        raise SystemExit(f"deferred low-alt LAND revalidation missing: {item}")

# Product LAND must return to Loiter standby only after native LAND has
# completed and the vehicle is disarmed. Low-alt throttle LAND and RC8 LAND
# both arm this pending return; unrelated LAND modes must not be forced.
for item in [
    "product_land_return_loiter_pending = true",
    "product_land_return_loiter_pending = false",
    "!motors->armed() && ap.land_complete && (flightmode == &mode_land)",
    "set_mode(Mode::Number::LOITER, ModeReason::RC_COMMAND)",
    "Product LAND complete; Loiter standby",
]:
    if item not in user:
        raise SystemExit(f"product LAND Loiter-return contract missing: {item}")

return_i = user.index("if (product_land_return_loiter_pending)")
return_text = user[return_i:return_i+1700]
if "flightmode != &mode_land" not in return_text:
    raise SystemExit("product LAND return must cancel if another mode owner takes control")
if return_text.index("!motors->armed() && ap.land_complete") > return_text.index("set_mode(Mode::Number::LOITER"):
    raise SystemExit("product LAND must verify disarmed+landed before Loiter standby")

aux_i2 = user.index("void Copter::userhook_auxSwitch1")
aux_text2 = user[aux_i2:]
if "product_land_return_loiter_pending = true" not in aux_text2:
    raise SystemExit("RC8 LAND does not arm Loiter standby return")

# Product-triggered LAND (low-alt throttle or RC8) locks pilot reposition and
# high-throttle escape, while non-product LAND retains native parameter logic.
if "product_land_stick_lock = true" not in user:
    raise SystemExit("product LAND stick lock is never armed")
if "product_land_stick_lock = false" not in user:
    raise SystemExit("product LAND stick lock is never cleared")
for source_name, source in [("mode.cpp", mode_cpp), ("mode_land.cpp", land_cpp)]:
    if "!copter.product_land_stick_locked()" not in source:
        raise SystemExit(f"{source_name} does not respect product LAND stick lock")
    if "g.land_repositioning && !copter.product_land_stick_locked()" not in source:
        raise SystemExit(f"{source_name} still permits product LAND repositioning")

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
