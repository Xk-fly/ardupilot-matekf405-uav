#!/usr/bin/env python3
"""V4 Flow-ground EKF-origin static safety regression."""
from pathlib import Path
u = Path("ArduCopter/UserCode.cpp").read_text()
e = Path("libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp").read_text()
o = Path("libraries/AP_NavEKF3/AP_NavEKF3_Outputs.cpp").read_text()
g = Path("libraries/GCS_MAVLink/GCS_Common.cpp").read_text()
s = u.index("} else if (ground_prep_state == GroundPrepState::SELECT) {")
n = u.index("} else if (ground_prep_state == GroundPrepState::WAIT_NAV) {", s)
z = u.index("} else if (ground_prep_state == GroundPrepState::VERIFY_Z) {", n)
a = u.index("} else if (motors->armed() && !ap.land_complete) {", z)
select, nav, checks = u[s:n], u[n:z], u[z:a]
checks_v4 = {
    "ground standby SRC3 intact": 'autosrc_select_source(AUTOSRC_STANDBY_SOURCE_SET, "ground standby")' in u,
    "ground selects only SRC2": "if (autosrc_flow_ground_ready)" in select and
        "ground_prep_chosen_source = AUTOSRC_FLOW_SOURCE_SET;" in select and
        "AUTOSRC_GPS_SOURCE_SET" not in select,
    "no ground GPS fallback": "ground_prep_flow_fallback_used" not in u and
        '"prep Flow nav timeout"' not in nav,
    "Flow selection bounded": "AUTOSRC_PREP_SELECT_TIMEOUT_MS 20000U" in u and
        "GroundPrep Flow/range not ready" in select,
    "Flow navigation bounded": "AUTOSRC_PREP_NAV_TIMEOUT_MS 20000U" in u and
        "GroundPrep Flow nav timeout" in nav and "AUTOSRC_PREP_NAV_HOLD_MS 400U" in u,
    "origin checked before any reset": "AP::ahrs().get_origin(ekf_origin)" in nav and
        nav.index("get_origin(ekf_origin)") < nav.index("resetHeightDatum()"),
    "reset possible only if origin absent": "else if (!origin_established && !AP::ahrs().resetHeightDatum())" in nav,
    "existing origin diagnostic present": "GroundPrep keep origin Z" in nav,
    "EKF original potential offset grounded": "EKF_origin.alt = dal.gps().location().alt" in e and
        "public_origin.alt - local_origin.alt" in o,
    "gross vertical error still rejects": "AUTOSRC_PREP_SEVERE_Z_CM 300.0f" in u and
        "fabsf(z_cm) > AUTOSRC_PREP_SEVERE_Z_CM" in checks and "GroundPrep unsafe Z" in checks,
    "normal height velocity and baro checks retained": "fabsf(z_cm) <= AUTOSRC_PREP_MAX_Z_CM" in checks and
        "fabsf(vz_cms) <= AUTOSRC_PREP_MAX_VZ_CMS" in checks and "fabsf(db) <= AUTOSRC_PREP_MAX_Z_DRIFT_CM" in checks,
    "READY and pre-takeoff gates retained": "GroundPrep readiness lost" in checks and
        "bool Copter::takeoff_prep_vertical_ready() const" in u and "bool Copter::autosrc_takeoff_ready()" in u,
    "Serial1 MT06 source filter retained": "get_uart() != AP::serialmanager().get_serial_by_id(1U)" in g,
    "GPS aerial handover retained": '"high-alt GPS"' in u and "AutoSourceState::GPS_HANDOVER" in u,
    "original GPS-origin mutation not modified": "EKF_origin.alt = dal.gps().location().alt" in e,
}
for name, passed in checks_v4.items():
    print(("PASS" if passed else "FAIL") + " " + name)
if not all(checks_v4.values()):
    raise SystemExit("V4 safety regression failed")
print(f"PASS {len(checks_v4)} V4 checks")
