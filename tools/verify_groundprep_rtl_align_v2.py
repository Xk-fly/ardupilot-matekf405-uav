#!/usr/bin/env python3
"""Static regression assertions for MatekF405-UAV ground-prep and normal RTL.

This does not substitute for SITL, a hardware bench test, or flight testing.
"""
from pathlib import Path
u = Path("ArduCopter/UserCode.cpp").read_text()
r = Path("ArduCopter/mode_rtl.cpp").read_text()
h = Path("ArduCopter/mode.h").read_text()
a = Path("ArduCopter/AP_Arming.cpp").read_text()
m = Path("ArduCopter/motors.cpp").read_text()
lo = Path("ArduCopter/mode_loiter.cpp").read_text()

tests = {
    "ground standby SRC3": 'AUTOSRC_STANDBY_SOURCE_SET 2U' in u
       and 'autosrc_select_source(AUTOSRC_STANDBY_SOURCE_SET, "ground standby")' in u,
    "source selection only on request": 'ground_prep_state == GroundPrepState::SELECT' in u
       and 'takeoff_prep_request(true)' in u and 'takeoff_prep_request(false)' in m,
    "common ARM check": 'copter.autosrc_takeoff_ready()' in a,
    "manual Loiter takeoff gate": 'copter.takeoff_prep_vertical_ready()' in lo,
    "one-key takeoff gate": 'takeoff_prep_vertical_ready()' in u,
    "severe Z check": 'AUTOSRC_PREP_SEVERE_Z_CM 300.0f' in u
       and 'GroundPrep unsafe Z' in u,
    "single transient not FAILED": '"GroundPrep vertical/nav unstable"' not in u
       and 'ground_prep_z_eval_since_ms = 0U;' in u,
    "failed prep retry": 'ground_prep_state == GroundPrepState::FAILED) {' in u,
    "baro primary height": 'set_and_save_by_name_ifchanged("EK3_SRC3_POSZ", 1.0f)' in u,
    "ordinary RTL only": 'copter.flightmode == &copter.mode_rtl' in r
       and 'copter.flightmode != &copter.mode_rtl' in r,
    "RTL align enum + handlers": 'ALIGN_HOME,' in h
       and r.count('case SubMode::ALIGN_HOME:') >= 3,
    "RTL stationary controller": 'void ModeRTL::align_home_run()' in r
       and 'wp_nav->update_wpnav()' in r and 'pos_control->update_z_controller();' in r,
    "RTL yaw shortest path": 'wrap_180_cd(_align_home_target_cd - ahrs.yaw_sensor)' in r,
    "no near-Home bypass": 'distance_m >= 0.05f' in r,
    "RTL timeout": '_align_home_start_ms >= 8000U' in r,
    "Home Z not averaged from GPS": 'Location home_loc = ahrs_ref.get_home();' in u
}
for name, passed in tests.items():
    print(f"{'PASS' if passed else 'FAIL'} {name}")
if not all(tests.values()):
    raise SystemExit("Static regression failed")
print(f"PASS {len(tests)} static checks")
