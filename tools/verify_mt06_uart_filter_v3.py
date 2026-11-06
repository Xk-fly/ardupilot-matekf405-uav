#!/usr/bin/env python3
"""Static regression checks for V3 MT-06 downward rangefinder UART filtering."""
from pathlib import Path

gcs = Path("libraries/GCS_MAVLink/GCS_Common.cpp").read_text()
h = Path("libraries/GCS_MAVLink/GCS.h").read_text()
rf = Path("libraries/AP_RangeFinder/AP_RangeFinder_MAVLink.cpp").read_text()
flow = Path("libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp").read_text()
hw = Path("libraries/AP_HAL_ChibiOS/hwdef/MatekF405/hwdef.dat").read_text()
target = Path("libraries/AP_HAL_ChibiOS/hwdef/MatekF405-UAV/hwdef.dat").read_text()
user = Path("ArduCopter/UserCode.cpp").read_text()

start = gcs.index("void GCS_MAVLINK::handle_distance_sensor(")
end = gcs.index("\n#if HAL_PROXIMITY_ENABLED\nvoid GCS_MAVLINK::handle_obstacle_distance(", start)
handler = gcs[start:end]
gate = handler.index("if (distance_sensor.orientation ==")
rf_dispatch = handler.index("rangefinder->handle_msg(msg);")
proximity_dispatch = handler.index("proximity->handle_msg(msg);")

tests = {
    "target guard only MatekF405-UAV": "#if defined(HAL_MATEKF405_UAV) && HAL_MATEKF405_UAV" in handler
        and "define HAL_MATEKF405_UAV 1" in target,
    "physical UART3 maps SERIAL1": "SERIAL_ORDER OTG1 USART3 UART4 USART1 UART5 USART2" in hw
        and "PC11 USART3_RX USART3" in hw,
    "source matched by UART driver": "get_uart() != AP::serialmanager().get_serial_by_id(1U)" in handler
        and "AP_HAL::UARTDriver *get_uart() { return _port; }" in h,
    "only downward orientation filtered": "distance_sensor.orientation == static_cast<uint8_t>(ROTATION_PITCH_270)" in handler,
    "filter precedes both destinations": gate < rf_dispatch and gate < proximity_dispatch
        and "return;" in handler[gate:rf_dispatch],
    "non-downward messages still dispatched": "rangefinder->handle_msg(msg);" in handler
        and "proximity->handle_msg(msg);" in handler,
    "MAVLink rangefinder source ambiguity": "if (packet.orientation == orientation())" in rf
        and "packet.id" not in rf,
    "OPTICAL_FLOW dispatch unchanged": "void GCS_MAVLINK::handle_optical_flow(const mavlink_message_t &msg)" in gcs
        and "optflow->handle_msg(msg);" in gcs,
    "PMW3901 backend unchanged": "mavlink_msg_optical_flow_decode(&msg, &packet);" in flow,
    "AutoSource unchanged": "get_rangefinder_height_interpolated_cm(autosrc_range_cm)" in user
        and "AUTOSRC_STANDBY_SOURCE_SET 2U" in user,
}
for name, ok in tests.items():
    print(f"{'PASS' if ok else 'FAIL'} {name}")
if not all(tests.values()):
    raise SystemExit("MT06 V3 static regression failed")
print(f"PASS {len(tests)} V3 static checks")
