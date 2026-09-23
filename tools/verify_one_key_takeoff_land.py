#!/usr/bin/env python3
from pathlib import Path

root = Path(__file__).resolve().parents[1]
user = (root / "ArduCopter/UserCode.cpp").read_text(encoding="utf-8")
config = (root / "ArduCopter/APM_Config.h").read_text(encoding="utf-8")
mode_h = (root / "ArduCopter/mode.h").read_text(encoding="utf-8")
takeoff = (root / "ArduCopter/takeoff.cpp").read_text(encoding="utf-8")

required_user = [
    "ONEKEY_RC_INPUT_TIMEOUT_MS 500U",
    "ONEKEY_TAKEOFF_LIFTOFF_TIMEOUT_MS 3000U",
    "static bool onekey_takeoff_watchdog_active = false;",
    "static uint32_t onekey_takeoff_watchdog_start_ms = 0U;",
    "static bool center_seen = false;",
    "if (ch_flag == RC_Channel::AuxSwitchPos::MIDDLE)",
    "if (!center_seen)",
    "if (!onekey_rc_input_fresh())",
    "if (motors->armed() || arming.is_armed())",
    "if (!ap.land_complete)",
    "if (!position_ok())",
    "set_mode(Mode::Number::LOITER, ModeReason::RC_COMMAND)",
    "arming.arm(AP_Arming::Method::AUXSWITCH, true)",
    "mode_loiter.do_user_takeoff_relative(takeoff_alt_cm, true)",
    "onekey_takeoff_watchdog_start_ms = AP_HAL::millis();",
    "onekey_takeoff_watchdog_active = true;",
    "Mode::takeoff_stop();",
    "OneKey TO abort: no liftoff in 3s, disarmed",
    "arming.disarm(AP_Arming::Method::AUXSWITCH)",
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


# V2 test-build intentionally removes the four-stick precondition while keeping
# normal ArduPilot arming checks. The obsolete gate must not remain in source.
for forbidden in [
    "ONEKEY_STICK_CENTER_TOLERANCE_PWM",
    "onekey_channel_centered(",
    "sticks not centered",
]:
    if forbidden in user:
        raise SystemExit(f"obsolete one-key stick gate still present: {forbidden}")

# The watchdog must run from the existing 50 Hz user hook so the 3 s timeout
# is independent of further aux-switch movements.
hook_i = user.index("void Copter::userhook_50Hz()")
abort_i = user.index("OneKey TO abort: no liftoff in 3s, disarmed")
if abort_i < hook_i:
    raise SystemExit("one-key liftoff watchdog is not serviced from userhook_50Hz")

# The takeoff callback must mode-switch before arming, and arm before starting takeoff.
loiter_i = user.index("set_mode(Mode::Number::LOITER, ModeReason::RC_COMMAND)")
arm_i = user.index("arming.arm(AP_Arming::Method::AUXSWITCH, true)")
takeoff_i = user.index("mode_loiter.do_user_takeoff_relative(takeoff_alt_cm, true)")
if not (loiter_i < arm_i < takeoff_i):
    raise SystemExit("one-key takeoff sequence is not LOITER -> ARM -> TAKEOFF")

print("ONE_KEY_TAKEOFF_LAND_STATIC_CHECK_PASS")
