# V4 GroundPrep: Flow-only ground preparation + existing EKF origin preservation

Base V3: 3962ffacdfad0451065ea3c9a8641889f6b3a8bb

Changes (MatekF405-UAV only):
- SRC3 remains disarmed standby. Explicit pre-arm requests may choose only SRC2 Flow when fresh ground Flow+MT-06 are available, never SRC1 GPS fallback. GPS remains available to the EKF, for airborne Flow/GPS handover, Home and RTL.
- The Flow/range selection and Flow EKF navigation phases have explicit 20-second timeouts; nav requires the existing 400 ms healthy hold.
- After successful relative navigation, do not call AHRS resetHeightDatum() if an EKF origin already exists. The old call can reassign EKF local origin altitude to latest GPS altitude while the common public origin stays fixed, creating false 3–6 m ground Z.
- If no EKF origin exists, retain original first-time datum reset. If actual public-height Z is already inconsistent, all preflight height/vertical-velocity limits still fail closed; no subtraction of arbitrary baselines, no acceptance merely because MT-06 shows 2 cm.
- V2 3m severe-Z, 1.5m normal-Z, 35 cm/s vertical-speed and baro stability tests, READY retention, manual/RC8 native arming checks remain unchanged.
- V3 downward-MAVLink source filtering is unchanged.
- Logs distinguish Flow/range acquisition timeout, Flow nav timeout, baro unhealthy, first height datum reset and preserved origin.

Verification: run V2+V3+V4 static contracts and full firmware compilation. Bench with propellers removed: no-GPS initial prepare, GPS already fixed initial prepare, GPS acquired during flight then land/reprepare (for latter first use reproducible replay/SITL). A successful compile is not proof of safe flight.
