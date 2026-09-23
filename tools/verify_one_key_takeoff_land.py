#!/usr/bin/env python3
from pathlib import Path

root = Path(__file__).resolve().parents[1]
user = (root / "ArduCopter/UserCode.cpp").read_text(encoding="utf-8")
config = (root / "ArduCopter/APM_Config.h").read_text(encoding="utf-8")
mode_h = (root / "ArduCopter/mode.h").read_text(encoding="utf-8")
takeoff = (root / "ArduCopter/takeoff.cpp").read_text(encoding="utf-8")

required_user = [
    "ONEKEY_STICK_CENTER_TOLERANCE_PWM 80",
    "ONEKEY_RC_INPUT_TIMEOUT_MS 500U",
    "static bool center_seen = false;",
    "if (ch_flag == RC_Channel::AuxSwitchPos::MIDDLE)",
    "if (!center_seen)",
    "if (!onekey_rc_input_fresh())",
    "if (motors->armed() || arming.is_armed())",
    "if (!ap.land_complete)",
    "!onekey_channel_centered(channel_roll)",
    "!onekey_channel_centered(channel_pitch)",
    "!onekey_channel_centered(channel_throttle)",
    "!onekey_channel_centered(channel_yaw)",
    "if (!position_ok())",
    "set_mode(Mode::Number::LOITER, ModeReason::RC_COMMAND)",
    "arming.arm(AP_Arming::Method::AUXSWITCH, true)",
    "mode_loiter.do_user_takeoff_relative(takeoff_alt_cm, true)",
    "arming.disarm(AP_Arming::Method::AUXSWITCH)",
    "set_mode(Mode::Number::LAND, ModeReason::RC_COMMAND)",
]
for item in required_user:
    if item not in user:
        raise SystemExit(f"missing one-key safety contract: {item}")

if "#define USERHOOK_AUXSWITCH ENABLED" not in config:
    raise SystemExit("USERHOOK_AUXSWITCH is not enabled")

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

# The takeoff callback must mode-switch before arming, and arm before starting takeoff.
loiter_i = user.index("set_mode(Mode::Number::LOITER, ModeReason::RC_COMMAND)")
arm_i = user.index("arming.arm(AP_Arming::Method::AUXSWITCH, true)")
takeoff_i = user.index("mode_loiter.do_user_takeoff_relative(takeoff_alt_cm, true)")
if not (loiter_i < arm_i < takeoff_i):
    raise SystemExit("one-key takeoff sequence is not LOITER -> ARM -> TAKEOFF")

print("ONE_KEY_TAKEOFF_LAND_STATIC_CHECK_PASS")
