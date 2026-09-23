#!/usr/bin/env python3
from pathlib import Path

root = Path(__file__).resolve().parents[1]
user = (root / "ArduCopter/UserCode.cpp").read_text(encoding="utf-8")
config = (root / "ArduCopter/APM_Config.h").read_text(encoding="utf-8")
mode_h = (root / "ArduCopter/mode.h").read_text(encoding="utf-8")
takeoff = (root / "ArduCopter/takeoff.cpp").read_text(encoding="utf-8")

required_user = [
    "ONEKEY_RC_INPUT_TIMEOUT_MS 500U",
    "ONEKEY_SPOOL_READY_TIMEOUT_MS 5000U",
    "ONEKEY_TAKEOFF_LIFTOFF_TIMEOUT_MS 3000U",
    "ONEKEY_LIFTOFF_CONFIRM_CM 12.0f",
    "ONEKEY_LIFTOFF_ABORT_MAX_CM 8.0f",
    "enum class OneKeyTakeoffState",
    "WAIT_SPOOL",
    "WAIT_LIFTOFF",
    "static bool center_seen = false;",
    "if (ch_flag == RC_Channel::AuxSwitchPos::MIDDLE)",
    "if (!center_seen)",
    "if (!onekey_rc_input_fresh())",
    "if (motors->armed() || arming.is_armed())",
    "if (!ap.land_complete)",
    "if (!position_ok())",
    "set_mode(Mode::Number::LOITER, ModeReason::RC_COMMAND)",
    "arming.arm(AP_Arming::Method::AUXSWITCH, true)",
    "OneKey armed; waiting motor spool",
    "!ap.in_arming_delay",
    "motors->get_interlock()",
    "AP_Motors::SpoolState::THROTTLE_UNLIMITED",
    "position lost before takeoff",
    "mode_loiter.do_user_takeoff_relative(onekey_pending_takeoff_alt_cm, true)",
    "get_rangefinder_height_interpolated_cm(range_alt_cm)",
    "OneKey TO liftoff confirmed",
    "Mode::takeoff_stop();",
    "set_auto_armed(false);",
    "OneKey TO abort: no physical liftoff in 3s",
    "arming.disarm(AP_Arming::Method::AUXSWITCH, false)",
    "set_mode(Mode::Number::LAND, ModeReason::RC_COMMAND)",
]
for item in required_user:
    if item not in user:
        raise SystemExit(f"missing one-key safety contract: {item}")

active_aux_define = any(
    line.strip().startswith("#define USERHOOK_AUXSWITCH ENABLED")
    for line in config.splitlines()
)
if not active_aux_define:
    raise SystemExit("USERHOOK_AUXSWITCH is not actively enabled")

if "bool do_user_takeoff_relative(float climb_alt_cm, bool must_navigate);" not in mode_h:
    raise SystemExit("relative takeoff API declaration missing")

required_takeoff = [
    "bool Mode::do_user_takeoff_relative(float climb_alt_cm, bool must_navigate)",
    "if (!copter.motors->armed())",
    "if (!copter.ap.land_complete)",
    "if (!has_user_takeoff(must_navigate))",
    "if (climb_alt_cm <= 0.0f)",
    "do_user_takeoff_start(climb_alt_cm)",
    "copter.set_auto_armed(true)",
]
for item in required_takeoff:
    if item not in takeoff:
        raise SystemExit(f"relative takeoff safety gate missing: {item}")

# Four-stick custom gating stays intentionally removed; normal ArduPilot arming
# checks remain authoritative.
for forbidden in [
    "ONEKEY_STICK_CENTER_TOLERANCE_PWM",
    "onekey_channel_centered(",
    "sticks not centered",
]:
    if forbidden in user:
        raise SystemExit(f"obsolete one-key stick gate still present: {forbidden}")

hook_i = user.index("void Copter::userhook_50Hz()")
aux_i = user.index("void Copter::userhook_auxSwitch1")
wait_spool_i = user.index("OneKeyTakeoffState::WAIT_SPOOL", hook_i)
spool_ready_i = user.index("AP_Motors::SpoolState::THROTTLE_UNLIMITED", hook_i)
takeoff_i = user.index(
    "mode_loiter.do_user_takeoff_relative(onekey_pending_takeoff_alt_cm, true)",
    hook_i,
)
if not (hook_i < wait_spool_i < spool_ready_i < takeoff_i < aux_i):
    raise SystemExit("takeoff must start asynchronously from userhook_50Hz after spool ready")

# The AUX callback must only switch LOITER, arm, and queue WAIT_SPOOL. It must
# not directly start Takeoff in the same callback.
aux_text = user[aux_i:]
if "mode_loiter.do_user_takeoff_relative(" in aux_text:
    raise SystemExit("aux callback directly starts Takeoff; spool sequencing regression")
loiter_i = aux_text.index("set_mode(Mode::Number::LOITER, ModeReason::RC_COMMAND)")
arm_i = aux_text.index("arming.arm(AP_Arming::Method::AUXSWITCH, true)")
queue_i = aux_text.index("onekey_takeoff_state = OneKeyTakeoffState::WAIT_SPOOL")
if not (loiter_i < arm_i < queue_i):
    raise SystemExit("aux sequence must be LOITER -> ARM -> WAIT_SPOOL")

# Never use land_complete alone as physical liftoff confirmation. The watchdog
# must use the downward rangefinder before any automatic no-liftoff disarm.
watchdog = user[hook_i:aux_i]
if "else if (!ap.land_complete)" in watchdog and "liftoff confirmed" in watchdog:
    # land_complete is allowed for spool preconditions, but not as the success signal
    success_region = watchdog[watchdog.index("WAIT_LIFTOFF"):]
    if "if (!ap.land_complete)" in success_region.split("OneKey TO liftoff confirmed", 1)[0]:
        raise SystemExit("land_complete is still used as liftoff confirmation")
range_i = watchdog.index("get_rangefinder_height_interpolated_cm(range_alt_cm)")
confirm_i = watchdog.index("OneKey TO liftoff confirmed")
abort_i = watchdog.index("OneKey TO abort: no physical liftoff in 3s")
if not (range_i < confirm_i < abort_i):
    raise SystemExit("physical liftoff watchdog ordering is invalid")

print("ONE_KEY_TAKEOFF_LAND_STATIC_CHECK_PASS")
