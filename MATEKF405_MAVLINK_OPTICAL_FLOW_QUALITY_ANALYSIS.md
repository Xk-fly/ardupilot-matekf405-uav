# MatekF405 ArduCopter 4.5.7 MAVLink 光流 quality 调试分析

本文档用于分析当前 ArduPilot/ArduCopter v4.5.7 源码中，`FLOW_TYPE=5` 即 MAVLink OpticalFlow 后端的消息入口、quality 字段路径、串口协议要求和调试建议。

当前背景：

```text
固件：ArduCopter v4.5.7
板型：MatekF405
光流连接：UART5，映射为 Serial4
参数：FLOW_TYPE = 5，也就是 MAVLink
现象：comp_m_x / comp_m_y 有值，但 opt_qua / quality 始终为 0
```

## 结论概要

当前源码里的 `FLOW_TYPE=5` 对应 `AP_OpticalFlow_MAV` 后端。

这个后端只处理 MAVLink `OPTICAL_FLOW` 消息，也就是：

```text
MAVLINK_MSG_ID_OPTICAL_FLOW
msg id: 100
结构体：mavlink_optical_flow_t
```

当前源码没有处理：

```text
MAVLINK_MSG_ID_OPTICAL_FLOW_RAD
msg id: 226
结构体：mavlink_optical_flow_rad_t
```

如果你的传感器实际发的是 `OPTICAL_FLOW_RAD` msg#226，那么在这个版本里会被忽略，不会进入 `AP_OpticalFlow_MAV::handle_msg()`。

`quality` 字段在 MAVLink 光流后端中的路径非常直接：

```text
GCS MAVLink parser
  -> GCS_MAVLINK::handle_optical_flow()
  -> AP_OpticalFlow::handle_msg()
  -> AP_OpticalFlow_MAV::handle_msg()
  -> packet.quality
  -> quality_sum
  -> state.surface_quality = quality_sum / count
  -> AP_OpticalFlow::_state.surface_quality
  -> optflow->quality()
  -> EKF / log / GCS output
```

在 `AP_OpticalFlow_MAV` 自身中，没有看到最低 quality 阈值、没有将非零 quality 强制置 0 的分支。它只是把收到的 `packet.quality` 累加并在 `update()` 时平均。

所以如果 `flow_x / flow_y` 能进入但 `quality` 一直为 0，最优先怀疑：

1. 传感器发出的 `OPTICAL_FLOW` msg#100 中 `quality` 字段本来就是 0。
2. 传感器实际发的是 `OPTICAL_FLOW_RAD` msg#226，而当前代码不处理。
3. 你看到的 `comp_m_x / comp_m_y` 不是来自当前 `AP_OpticalFlow_MAV` 后端的 `OPTICAL_FLOW` msg#100 原始路径，可能来自其他转发、显示或不同消息。
4. `SERIAL4_PROTOCOL` 没有设成 MAVLink，导致该 UART 不作为 GCS MAVLink 通道解析。

## 1. FLOW_TYPE=5 的消息处理入口

### FLOW_TYPE 参数枚举

文件：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow.cpp
```

关键位置：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow.cpp:31
```

参数说明：

```cpp
// @Param: _TYPE
// @DisplayName: Optical flow sensor type
// @Description: Optical flow sensor type
// @Values: 0:None, 1:PX4Flow, 2:Pixart, 3:Bebop, 4:CXOF, 5:MAVLink, 6:DroneCAN, 7:MSP, 8:UPFLOW
AP_GROUPINFO_FLAGS("_TYPE", 0,  AP_OpticalFlow,    _type,   (float)OPTICAL_FLOW_TYPE_DEFAULT, AP_PARAM_FLAG_ENABLE),
```

结论：

```text
FLOW_TYPE = 5 -> Type::MAVLINK
```

### FLOW_TYPE=5 创建的后端

文件：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow.cpp
```

关键位置：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow.cpp:152
```

代码：

```cpp
case Type::MAVLINK:
#if AP_OPTICALFLOW_MAV_ENABLED
    backend = AP_OpticalFlow_MAV::detect(*this);
#endif
    break;
```

后端 enable 条件：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow_config.h:27
```

```cpp
#ifndef AP_OPTICALFLOW_MAV_ENABLED
#define AP_OPTICALFLOW_MAV_ENABLED (AP_OPTICALFLOW_BACKEND_DEFAULT_ENABLED && HAL_GCS_ENABLED)
#endif
```

含义：

- `FLOW_TYPE=5` 依赖 `AP_OPTICALFLOW_MAV_ENABLED`。
- 对普通 Copter 固件，只要 optical flow 和 GCS/MAVLink 没被裁掉，后端会被编译。

### MAVLink 光流后端处理函数

文件：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp
```

函数：

```cpp
void AP_OpticalFlow_MAV::handle_msg(const mavlink_message_t &msg)
```

关键位置：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp:91
```

代码：

```cpp
// handle OPTICAL_FLOW mavlink messages
void AP_OpticalFlow_MAV::handle_msg(const mavlink_message_t &msg)
{
    mavlink_optical_flow_t packet;
    mavlink_msg_optical_flow_decode(&msg, &packet);

    latest_frame_us = AP_HAL::micros64();

    flow_sum.x += packet.flow_x;
    flow_sum.y += packet.flow_y;
    quality_sum += packet.quality;
    count++;

    sensor_id = packet.sensor_id;
}
```

结论：

- 该函数注释明确写的是 `OPTICAL_FLOW mavlink messages`。
- 使用的是 `mavlink_optical_flow_t`。
- 解码函数是 `mavlink_msg_optical_flow_decode()`。
- 读取 quality 的字段是：

```cpp
packet.quality
```

### 它期望的 MAVLink 消息类型

由函数中使用的结构体和解码函数可确定：

```text
期望消息：OPTICAL_FLOW
MAVLink msg id：100
结构体：mavlink_optical_flow_t
quality 字段：mavlink_optical_flow_t::quality
flow 字段：mavlink_optical_flow_t::flow_x / flow_y
```

当前源码没有 `mavlink_msg_optical_flow_rad_decode()` 调用，也没有 `MAVLINK_MSG_ID_OPTICAL_FLOW_RAD` case。

搜索结果：

```text
rg "MAVLINK_MSG_ID_OPTICAL_FLOW_RAD|mavlink_optical_flow_rad" libraries/AP_OpticalFlow libraries/GCS_MAVLink/*.cpp ArduCopter/*.cpp
```

在业务源码中没有命中。

## 2. quality 字段的过滤逻辑

### MAVLink 光流后端中的 quality 处理

文件：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp
```

关键位置：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp:52
```

代码：

```cpp
state.surface_quality = quality_sum / count;
```

关键位置：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp:83
```

代码：

```cpp
quality_sum = 0;
```

关键位置：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp:104
```

代码：

```cpp
quality_sum += packet.quality;
```

结论：

- `AP_OpticalFlow_MAV` 对 quality 只做累加和平均。
- 没有看到 `quality < 某值` 则忽略的判断。
- 没有看到将 `packet.quality` 强制改为 0 的逻辑。
- 没有单独滤波器，只有按 `count` 做平均。

### 前端 AP_OpticalFlow 中的 quality 使用

文件：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow.h
```

关键位置：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow.h:80
```

代码：

```cpp
uint8_t quality() const { return _state.surface_quality; }
```

关键位置：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow.h:95
```

代码：

```cpp
uint8_t surface_quality;   // image quality (below TBD you can't trust the dx,dy values returned)
```

文件：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow.cpp
```

关键位置：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow.cpp:266
```

代码：

```cpp
void AP_OpticalFlow::update_state(const OpticalFlow_state &state)
{
    _state = state;
    _last_update_ms = AP_HAL::millis();

    AP::ahrs().writeOptFlowMeas(quality(),
                                _state.flowRate,
                                _state.bodyRate,
                                _last_update_ms,
                                get_pos_offset(),
                                get_height_override());

    Log_Write_Optflow();
}
```

关键位置：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow.cpp:297
```

日志写入：

```cpp
struct log_Optflow pkt = {
    LOG_PACKET_HEADER_INIT(LOG_OPTFLOW_MSG),
    time_us         : AP_HAL::micros64(),
    surface_quality : _state.surface_quality,
    flow_x          : _state.flowRate.x,
    flow_y          : _state.flowRate.y,
    body_x          : _state.bodyRate.x,
    body_y          : _state.bodyRate.y
};
```

结论：

- AP_OpticalFlow 前端直接保存后端给出的 `surface_quality`。
- 再把 `quality()` 传给 AHRS/EKF。
- 日志中的 opt quality 也来自 `_state.surface_quality`。
- 前端没有对 quality 设置最低阈值。
- 前端没有把 quality 强制置 0 的判断。

### 健康状态对 quality 的间接影响

文件：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow.cpp
```

关键位置：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow.cpp:194
```

代码：

```cpp
_flags.healthy = (AP_HAL::millis() - _last_update_ms < 500);
```

含义：

- 如果 500ms 内没有成功 `update_state()`，`optflow.healthy()` 会变 false。
- 这不会直接把 `AP_OpticalFlow::_state.surface_quality` 清零。
- 但某些使用者会在 unhealthy 时把自己的滤波质量清零。

### FlowHold 中的 quality 阈值和清零

如果你使用的是 FlowHold 模式，还需要看 `ModeFlowHold` 自己的质量门槛。

文件：

```text
ArduCopter/mode_flowhold.cpp
```

参数：

```text
ArduCopter/mode_flowhold.cpp:57
```

```cpp
// @Param: _QUAL_MIN
// @DisplayName: FlowHold Flow quality minimum
// @Description: Minimum flow quality to use flow position hold
// @Range: 0 255
AP_GROUPINFO("_QUAL_MIN", 4, ModeFlowHold, flow_min_quality, 10),
```

初始化清零：

```text
ArduCopter/mode_flowhold.cpp:102
```

```cpp
quality_filtered = 0;
```

运行时滤波：

```text
ArduCopter/mode_flowhold.cpp:256
```

```cpp
if (copter.optflow.healthy()) {
    const float filter_constant = 0.95;
    quality_filtered = filter_constant * quality_filtered + (1-filter_constant) * copter.optflow.quality();
} else {
    quality_filtered = 0;
}
```

质量低于门槛时不使用 FlowHold 位置修正：

```text
ArduCopter/mode_flowhold.cpp:322
```

```cpp
if (quality_filtered >= flow_min_quality &&
    AP_HAL::millis() - copter.arm_time_ms > 3000) {
    ...
}
```

结论：

- `FHLD_QUAL_MIN` 默认是 10。
- 这是 FlowHold 模式自己的门槛，不是 `AP_OpticalFlow_MAV` 的消息解析门槛。
- 如果 `optflow.healthy()==false`，`quality_filtered` 会被置 0。
- 如果你看到的是 FlowHold 日志中的 `Qual`，它可能是 `quality_filtered`，不是原始 MAVLink `packet.quality`。

### 其他后端 quality 赋值位置

这些不是 `FLOW_TYPE=5` 的路径，但可用于对比：

```text
AP_OpticalFlow_PX4Flow.cpp:126       state.surface_quality = frame.qual;
AP_OpticalFlow_Pixart.cpp:339        state.surface_quality = burst.squal;
AP_OpticalFlow_CXOF.cpp:164          对 qual_sum 做范围映射后赋值
AP_OpticalFlow_MSP.cpp:54            state.surface_quality = quality_sum / count;
AP_OpticalFlow_MSP.cpp:106           quality_sum += pkt.quality * 100 / 255;
AP_OpticalFlow_UPFLOW.cpp:161        state.surface_quality = updata.quality;
AP_OpticalFlow_HereFlow.cpp:59       surface_quality = msg.quality;
AP_OpticalFlow_Onboard.cpp:52        state.surface_quality = data_frame.quality;
```

对本问题最关键的是 `AP_OpticalFlow_MAV.cpp`，即 `FLOW_TYPE=5`。

## 3. MAVLink 消息分发路径

### GCS MAVLink switch 只分发 OPTICAL_FLOW msg#100

文件：

```text
libraries/GCS_MAVLink/GCS_Common.cpp
```

关键位置：

```text
libraries/GCS_MAVLink/GCS_Common.cpp:4168
```

代码：

```cpp
#if AP_OPTICALFLOW_ENABLED
case MAVLINK_MSG_ID_OPTICAL_FLOW:
    handle_optical_flow(msg);
    break;
#endif
```

结论：

- `GCS_MAVLink` 只对 `MAVLINK_MSG_ID_OPTICAL_FLOW` 调用 `handle_optical_flow()`。
- 当前源码没有 `case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD`。
- 因此 `OPTICAL_FLOW_RAD` msg#226 不会进入 AP_OpticalFlow。

### GCS handle_optical_flow 函数

文件：

```text
libraries/GCS_MAVLink/GCS_Common.cpp
```

关键位置：

```text
libraries/GCS_MAVLink/GCS_Common.cpp:3823
```

代码：

```cpp
#if AP_OPTICALFLOW_ENABLED
void GCS_MAVLINK::handle_optical_flow(const mavlink_message_t &msg)
{
    AP_OpticalFlow *optflow = AP::opticalflow();
    if (optflow == nullptr) {
        return;
    }
    optflow->handle_msg(msg);
}
#endif
```

结论：

- GCS 层不 decode 光流消息。
- GCS 层不读取或修改 quality。
- GCS 层只是把整条 `mavlink_message_t` 传给 `AP_OpticalFlow`。

### AP_OpticalFlow 前端转发

文件：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow.cpp
```

关键位置：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow.cpp:215
```

代码：

```cpp
void AP_OpticalFlow::handle_msg(const mavlink_message_t &msg)
{
    if (!enabled()) {
        return;
    }

    if (backend != nullptr) {
        backend->handle_msg(msg);
    }
}
```

结论：

- 如果 `FLOW_TYPE=0` 或后端没创建，消息会被忽略。
- 如果 `FLOW_TYPE=5` 且后端创建成功，消息会进入 `AP_OpticalFlow_MAV::handle_msg()`。

### GCS 发送光流消息的路径

文件：

```text
libraries/GCS_MAVLink/GCS_Common.cpp
```

关键位置：

```text
libraries/GCS_MAVLink/GCS_Common.cpp:2661
```

代码：

```cpp
void GCS_MAVLINK::send_opticalflow()
{
    const AP_OpticalFlow *optflow = AP::opticalflow();

    if (optflow == nullptr ||
        !optflow->healthy()) {
        return;
    }

    const Vector2f &flowRate = optflow->flowRate();
    const Vector2f &bodyRate = optflow->bodyRate();

    mavlink_msg_optical_flow_send(
        chan,
        AP_HAL::millis(),
        0,
        flowRate.x,
        flowRate.y,
        flowRate.x - bodyRate.x,
        flowRate.y - bodyRate.y,
        optflow->quality(),
        hagl,
        flowRate.x,
        flowRate.y);
}
```

含义：

- 飞控往地面站发的也是 `OPTICAL_FLOW` msg#100。
- 地面站看到的 `quality` 是 `optflow->quality()`，即 `_state.surface_quality`。

## 4. Serial4 协议与光流的关系

### MatekF405 的 UART 与 Serial 序号

文件：

```text
libraries/AP_HAL_ChibiOS/hwdef/MatekF405/hwdef.dat
```

关键位置：

```text
libraries/AP_HAL_ChibiOS/hwdef/MatekF405/hwdef.dat:26
```

代码：

```text
SERIAL_ORDER OTG1 USART3 UART4 USART1 UART5 USART2
```

映射关系：

```text
SERIAL0 -> OTG1 / USB
SERIAL1 -> USART3
SERIAL2 -> UART4
SERIAL3 -> USART1
SERIAL4 -> UART5
SERIAL5 -> USART2
```

UART5 引脚：

```text
libraries/AP_HAL_ChibiOS/hwdef/MatekF405/hwdef.dat:105
```

```text
PD2  UART5_RX UART5
PC12 UART5_TX UART5
```

结论：

```text
光流接 UART5 时，对应参数就是 SERIAL4_xxx。
```

### SERIAL4_PROTOCOL 应设置为什么

文件：

```text
libraries/AP_SerialManager/AP_SerialManager.h
```

关键位置：

```text
libraries/AP_SerialManager/AP_SerialManager.h:37
```

协议枚举：

```cpp
SerialProtocol_MAVLink = 1,
SerialProtocol_MAVLink2 = 2,
SerialProtocol_OpticalFlow = 18,
SerialProtocol_MAVLinkHL = 43,
```

文件：

```text
libraries/AP_SerialManager/AP_SerialManager.cpp
```

关键位置：

```text
libraries/AP_SerialManager/AP_SerialManager.cpp:178
```

参数文档说明 `SERIALx_PROTOCOL` 可选值：

```cpp
// @Values: -1:None, 1:MAVLink1, 2:MAVLink2, ..., 18:OpticalFlow, ..., 43:MAVLink High Latency
```

对 `FLOW_TYPE=5` 的 MAVLink 光流，`SERIAL4_PROTOCOL` 应设置为：

```text
SERIAL4_PROTOCOL = 1   # MAVLink1
```

或：

```text
SERIAL4_PROTOCOL = 2   # MAVLink2
```

通常建议：

```text
SERIAL4_PROTOCOL = 2
```

并设置与传感器一致的波特率：

```text
SERIAL4_BAUD = 115     # 115200
```

或者按传感器实际波特率设置。

### 为什么不是 SERIAL4_PROTOCOL=18

`SERIALx_PROTOCOL=18` 是 `SerialProtocol_OpticalFlow`，用于某些串口私有协议光流后端，例如 UPFLOW 一类会自己查找该串口。

例如：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow_UPFLOW.cpp:70
```

该后端会查找：

```cpp
serial_manager->find_serial(AP_SerialManager::SerialProtocol_OpticalFlow, 0);
```

但 `FLOW_TYPE=5` 的 `AP_OpticalFlow_MAV` 不查找 `SerialProtocol_OpticalFlow`。它假设 MAVLink 消息已经由 GCS MAVLink 层解析并分发给它。

因此：

```text
FLOW_TYPE=5 + SERIAL4_PROTOCOL=18
```

通常不会让 MAVLink 光流消息被 GCS MAVLink parser 接收。

正确关系是：

```text
FLOW_TYPE=5
SERIAL4_PROTOCOL=1 或 2
SERIAL4_BAUD=传感器 MAVLink 输出波特率
```

### 固件是否会自动监听对应串口

不会因为 `FLOW_TYPE=5` 自动把某个串口设置为 MAVLink。

MAVLink 串口创建由 GCS 层完成。

文件：

```text
libraries/GCS_MAVLink/GCS_Common.cpp
```

关键位置：

```text
libraries/GCS_MAVLink/GCS_Common.cpp:2540
```

代码：

```cpp
void GCS::setup_uarts()
{
    for (uint8_t i = 1; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        AP_HAL::UARTDriver *uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_MAVLink, i);
        if (uart == nullptr) {
            break;
        }
        create_gcs_mavlink_backend(chan_parameters[i], *uart);
    }
}
```

`find_serial(SerialProtocol_MAVLink, i)` 会匹配 MAVLink1、MAVLink2 和 MAVLinkHL。

协议匹配逻辑：

```text
libraries/AP_SerialManager/AP_SerialManager.cpp:762
```

```cpp
bool AP_SerialManager::protocol_match(enum SerialProtocol protocol1, enum SerialProtocol protocol2) const
{
    if (protocol1 == protocol2) {
        return true;
    }

    if (((protocol1 == SerialProtocol_MAVLink) || (protocol1 == SerialProtocol_MAVLink2) || (protocol1 == SerialProtocol_MAVLinkHL)) &&
        ((protocol2 == SerialProtocol_MAVLink) || (protocol2 == SerialProtocol_MAVLink2) || (protocol2 == SerialProtocol_MAVLinkHL))) {
        return true;
    }
    ...
}
```

结论：

- 只有 `SERIAL4_PROTOCOL=1/2/43` 时，UART5/Serial4 才会被 GCS MAVLink 层作为 MAVLink 通道处理。
- `FLOW_TYPE=5` 只决定 AP_OpticalFlow 使用 MAVLink 后端，不负责串口 parser。
- 如果 Serial4 没设为 MAVLink，`OPTICAL_FLOW` msg#100 不会进入 `GCS_MAVLINK::handle_optical_flow()`。

## 5. 针对 quality=0 的源码级判断

### 当前源码不会把 MAVLink quality 非零值强制置 0

`AP_OpticalFlow_MAV::handle_msg()`：

```cpp
quality_sum += packet.quality;
```

`AP_OpticalFlow_MAV::update()`：

```cpp
state.surface_quality = quality_sum / count;
```

没有：

```cpp
if (quality < ...)
quality = 0
return
```

因此若 `AP_OpticalFlow_MAV::handle_msg()` 收到的 `packet.quality` 非零，最终 `state.surface_quality` 应该也非零，除非平均窗口里所有消息 quality 都是 0。

### 可能混淆的字段

MAVLink `OPTICAL_FLOW` msg#100 中有：

```text
flow_x
flow_y
flow_comp_m_x
flow_comp_m_y
quality
ground_distance
flow_rate_x
flow_rate_y
```

当前 `AP_OpticalFlow_MAV::handle_msg()` 使用的是：

```cpp
flow_sum.x += packet.flow_x;
flow_sum.y += packet.flow_y;
quality_sum += packet.quality;
```

它不使用：

```text
flow_comp_m_x
flow_comp_m_y
flow_rate_x
flow_rate_y
```

这点很关键。

如果你在地面站看到 `comp_m_x / comp_m_y` 有值，说明某处的 `OPTICAL_FLOW` 消息里补偿字段有值，但当前后端真正用于光流积分的是 `flow_x / flow_y`，quality 则来自同一个 msg#100 的 `quality` 字段。

如果传感器只填充了 `flow_comp_m_x / flow_comp_m_y`，但 `flow_x / flow_y` 或 `quality` 不按 ArduPilot 期望填充，可能出现显示上“有光流值”，但 AP_OpticalFlow quality 仍为 0 的现象。

## 6. 调试建议

### 建议 1：在 GCS 分发入口确认是否收到 msg#100

位置：

```text
libraries/GCS_MAVLink/GCS_Common.cpp:4169
```

可临时在 case 中加入打印：

```cpp
case MAVLINK_MSG_ID_OPTICAL_FLOW:
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RX OPTICAL_FLOW chan=%u sys=%u comp=%u len=%u",
                  (unsigned)chan,
                  (unsigned)msg.sysid,
                  (unsigned)msg.compid,
                  (unsigned)msg.len);
    handle_optical_flow(msg);
    break;
```

注意：

- 高频消息不能一直打印，否则会刷屏并影响实时性。
- 建议加限频，例如每 1 秒打印一次。

### 建议 2：在 GCS 分发层确认是否收到 msg#226

当前源码没有 `OPTICAL_FLOW_RAD` case。为了确认传感器是否发的是 msg#226，可临时增加：

```cpp
case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RX OPTICAL_FLOW_RAD chan=%u sys=%u comp=%u len=%u",
                  (unsigned)chan,
                  (unsigned)msg.sysid,
                  (unsigned)msg.compid,
                  (unsigned)msg.len);
    break;
```

如果能打印 `OPTICAL_FLOW_RAD`，但没有 `OPTICAL_FLOW`，说明当前 `FLOW_TYPE=5` 后端收不到有效光流，因为本版本没有处理 msg#226。

### 建议 3：在 AP_OpticalFlow_MAV::handle_msg() 打印原始 quality

位置：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp:91
```

建议临时调试：

```cpp
static uint32_t last_debug_ms;
const uint32_t now_ms = AP_HAL::millis();
if (now_ms - last_debug_ms > 1000) {
    last_debug_ms = now_ms;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                  "FLOW_MAV raw q=%u flow=%d,%d comp=%.3f,%.3f sid=%u",
                  (unsigned)packet.quality,
                  (int)packet.flow_x,
                  (int)packet.flow_y,
                  (double)packet.flow_comp_m_x,
                  (double)packet.flow_comp_m_y,
                  (unsigned)packet.sensor_id);
}
```

这个位置最能确认：

```text
飞控是否真的收到 OPTICAL_FLOW msg#100
packet.quality 原始值是否为 0
flow_x / flow_y 是否被传感器填充
comp_m_x / comp_m_y 是否只是显示字段
```

### 建议 4：在 AP_OpticalFlow_MAV::update() 打印平均后的 quality

位置：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp:50
```

建议临时调试：

```cpp
GCS_SEND_TEXT(MAV_SEVERITY_INFO,
              "FLOW_MAV avg q=%u count=%u dt=%.3f sum=%ld,%ld",
              (unsigned)state.surface_quality,
              (unsigned)count,
              (double)dt,
              (long)flow_sum.x,
              (long)flow_sum.y);
```

注意 `dt` 在当前代码中第 55 行才计算，如果要打印 `dt`，放在 dt 计算之后。

### 建议 5：检查 DataFlash 日志

`AP_OpticalFlow` 会写 `OPTFLOW` 日志。

位置：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow.cpp:286
```

字段：

```cpp
surface_quality
flow_x
flow_y
body_x
body_y
```

如果 `.BIN` 日志中 `OPTFLOW.surface_quality` 始终为 0，而 `AP_OpticalFlow_MAV::handle_msg()` 打印 `packet.quality` 非零，则需要继续查 `update()` 和 `_update_frontend()`。

如果 `handle_msg()` 打印 `packet.quality=0`，则问题在传感器发送的 MAVLink 消息内容或消息类型。

### 建议 6：用 MAVProxy/地面站看消息类型

如果能通过 MAVProxy 连接飞控，可尝试：

```text
status
messages
watch OPTICAL_FLOW
```

重点确认：

```text
是否有 OPTICAL_FLOW msg#100
quality 字段是多少
是否只有 OPTICAL_FLOW_RAD msg#226
```

Mission Planner 中也可以看 MAVLink Inspector：

```text
OPTICAL_FLOW.quality
OPTICAL_FLOW.flow_x / flow_y
OPTICAL_FLOW.flow_comp_m_x / flow_comp_m_y
OPTICAL_FLOW_RAD.quality
```

## 7. 参数检查清单

### 必查参数

```text
FLOW_TYPE = 5
SERIAL4_PROTOCOL = 1 或 2
SERIAL4_BAUD = 传感器实际 MAVLink 波特率
SERIAL4_OPTIONS = 按硬件电平/半双工/反相需求设置，通常先保持 0
```

推荐：

```text
SERIAL4_PROTOCOL = 2
```

如果传感器只支持 MAVLink1：

```text
SERIAL4_PROTOCOL = 1
```

不要用：

```text
SERIAL4_PROTOCOL = 18
```

除非你使用的是 `FLOW_TYPE=8` 或其他串口私有协议光流后端，而不是 MAVLink 光流。

### MatekF405 串口映射

```text
SERIAL4 = UART5
UART5_RX = PD2
UART5_TX = PC12
```

所以你的接线应按：

```text
传感器 TX -> 飞控 UART5_RX / PD2
传感器 RX -> 飞控 UART5_TX / PC12
GND 共地
电平匹配
```

如果只是传感器单向发送 MAVLink 光流，至少需要：

```text
传感器 TX -> PD2 UART5_RX
GND 共地
```

## 8. 如果传感器发的是 OPTICAL_FLOW_RAD

当前代码不处理 msg#226。

可能方案：

### 方案 A：修改传感器输出 OPTICAL_FLOW msg#100

这是最小风险方案。

要求传感器发送：

```text
MAVLINK_MSG_ID_OPTICAL_FLOW = 100
quality 非零
flow_x / flow_y 有效
```

不要只填：

```text
flow_comp_m_x / flow_comp_m_y
```

### 方案 B：在 ArduPilot 中增加 OPTICAL_FLOW_RAD 支持

需要改：

```text
libraries/GCS_MAVLink/GCS_Common.cpp
libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp
```

思路：

1. `GCS_Common.cpp` 增加：

```cpp
case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
    handle_optical_flow(msg);
    break;
```

2. `AP_OpticalFlow_MAV::handle_msg()` 内按 `msg.msgid` 区分：

```cpp
if (msg.msgid == MAVLINK_MSG_ID_OPTICAL_FLOW) {
    mavlink_optical_flow_t packet;
    mavlink_msg_optical_flow_decode(&msg, &packet);
    quality_sum += packet.quality;
    flow_sum.x += packet.flow_x;
    flow_sum.y += packet.flow_y;
} else if (msg.msgid == MAVLINK_MSG_ID_OPTICAL_FLOW_RAD) {
    mavlink_optical_flow_rad_t packet;
    mavlink_msg_optical_flow_rad_decode(&msg, &packet);
    quality_sum += packet.quality;
    // 这里不能直接照搬 flow_x / flow_y，因为 RAD 消息字段是 integrated_x / integrated_y，单位和语义不同。
}
```

注意：

- `OPTICAL_FLOW_RAD` 的 `integrated_x / integrated_y` 是积分角，单位和 `OPTICAL_FLOW.flow_x / flow_y` 不同。
- 当前 `AP_OpticalFlow_MAV` 的 `update()` 用 `flow_sum/count * dt` 计算 flowRate，直接接 RAD 消息需要重新确认单位转换。
- 所以不建议盲目加 case 后直接把 `integrated_x` 当 `flow_x` 使用。

## 9. 最可能原因排序

结合当前源码，针对“comp_m_x / comp_m_y 有值但 quality 始终 0”，最可能原因：

1. 传感器发出的 `OPTICAL_FLOW` msg#100 中 `quality` 字段就是 0。
2. 传感器只填充 `flow_comp_m_x / flow_comp_m_y`，没有正确填充 `quality`。
3. 传感器发的是 `OPTICAL_FLOW_RAD` msg#226，当前固件没有分发和处理该消息。
4. `SERIAL4_PROTOCOL` 不是 MAVLink1/2，导致 UART5 的数据没有进入 MAVLink parser。
5. `SERIAL4_BAUD` 不匹配，解析不稳定，但如果能稳定看到相关 MAVLink 字段，这个优先级略低。
6. FlowHold 模式中 `optflow.healthy()==false`，导致 `quality_filtered` 被清零；这影响 FlowHold 日志/控制，不一定代表原始 MAVLink quality 为 0。

## 10. 快速验证步骤

### 第一步：确认参数

```text
FLOW_TYPE = 5
SERIAL4_PROTOCOL = 2
SERIAL4_BAUD = 传感器波特率
```

如果传感器只发 MAVLink1：

```text
SERIAL4_PROTOCOL = 1
```

### 第二步：用 MAVLink Inspector 看消息

查看：

```text
OPTICAL_FLOW
OPTICAL_FLOW.quality
OPTICAL_FLOW.flow_x
OPTICAL_FLOW.flow_y
OPTICAL_FLOW.flow_comp_m_x
OPTICAL_FLOW.flow_comp_m_y
OPTICAL_FLOW_RAD
OPTICAL_FLOW_RAD.quality
```

判断：

```text
只有 OPTICAL_FLOW_RAD，没有 OPTICAL_FLOW -> 当前 AP_OpticalFlow_MAV 不处理
OPTICAL_FLOW 有但 quality=0 -> 传感器 msg#100 字段填充问题
OPTICAL_FLOW quality 非零但飞控日志为 0 -> 需要在 AP_OpticalFlow_MAV::handle_msg/update 加打印追踪
```

### 第三步：加限频 GCS_SEND_TEXT

优先加在：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp:91
```

确认原始：

```text
packet.quality
packet.flow_x
packet.flow_y
packet.flow_comp_m_x
packet.flow_comp_m_y
```

然后加在：

```text
libraries/AP_OpticalFlow/AP_OpticalFlow_MAV.cpp:52
```

确认平均后的：

```text
state.surface_quality
count
```

### 第四步：看日志

查看 `OPTFLOW` 日志：

```text
surface_quality
flow_x
flow_y
body_x
body_y
```

如果使用 FlowHold，同时看 `FHLD` 日志：

```text
Qual
```

注意：

```text
FHLD.Qual 是 quality_filtered，不是原始 packet.quality。
```

## 11. 最终判断

对当前 ArduCopter v4.5.7 源码：

```text
FLOW_TYPE=5 只接收 OPTICAL_FLOW msg#100。
quality 来自 mavlink_optical_flow_t::quality。
AP_OpticalFlow_MAV 不处理 OPTICAL_FLOW_RAD msg#226。
AP_OpticalFlow_MAV 不做 quality 最低阈值过滤，也不会把非零 quality 置 0。
Serial4/UART5 必须配置为 MAVLink 协议，即 SERIAL4_PROTOCOL=1 或 2。
FLOW_TYPE=5 不会自动把 Serial4 设置成 MAVLink，也不会监听 SERIAL4_PROTOCOL=18 的 OpticalFlow 私有协议。
```

因此本问题下一步最该验证的是：

```text
传感器实际发的是 msg#100 还是 msg#226；
msg#100 的 quality 字段原始值是否非零；
SERIAL4_PROTOCOL 是否为 1 或 2；
传感器是否只填了 comp_m_x/comp_m_y 而没填 flow_x/flow_y/quality。
```
