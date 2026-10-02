#include "Copter.h"

#define MY_CUSTOM_LED_PIN 59

// One-key takeoff / landing on USER_FUNC1 (intended for the self-centering RC8).
// HIGH edge: disarmed+landed -> wait for automatic Flow/GPS position -> LOITER
// -> normal arming checks -> relative PILOT_TKOFF_ALT takeoff -> remain in LOITER.
// LOW edge: if armed, enter native LAND.
// MIDDLE only re-arms the edge detector; returning to center never changes mode.
#define ONEKEY_RC_INPUT_TIMEOUT_MS 500U
#define ONEKEY_POSITION_WAIT_TIMEOUT_MS 90000U
#define ONEKEY_SPOOL_READY_TIMEOUT_MS 5000U
#define ONEKEY_IDLE_HOLD_MS 1000U
#define ONEKEY_TAKEOFF_LIFTOFF_TIMEOUT_MS 3000U
#define ONEKEY_LIFTOFF_CONFIRM_CM 12.0f
#define ONEKEY_LIFTOFF_ABORT_MAX_CM 8.0f
#define ONEKEY_LIFTOFF_CONFIRM_HOLD_MS 150U
#define ONEKEY_MIN_TAKEOFF_ALT_CM 10.0f
#define ONEKEY_MAX_TAKEOFF_ALT_CM 1000.0f

// Automatic indoor/outdoor source manager.
// Source-set contract for this build:
//   SRC1 (index 0): GPS position + GPS velocity + Baro Z + Compass yaw
//   SRC2 (index 1): no XY position + OpticalFlow velocity + Baro Z + Compass yaw
// Normal takeoff prefers SRC2 whenever low-altitude Flow/Range is healthy.
// In flight, normal source selection is bidirectional with hysteresis:
//   SRC2 Flow -> SRC1 GPS above 2.0 m after GPS/height/speed gates pass.
//   SRC1 GPS  -> SRC2 Flow below 1.7 m after Flow/height/speed gates pass.
// Emergency source degradation remains independent of the normal hysteresis.
#if defined(HAL_MATEKF405_UAV) && HAL_MATEKF405_UAV
#define AUTO_SOURCE_MANAGER_ENABLED 1
#else
#define AUTO_SOURCE_MANAGER_ENABLED 0
#endif
#define AUTOSRC_GPS_SOURCE_SET 0U
#define AUTOSRC_FLOW_SOURCE_SET 1U
#define AUTOSRC_GPS_MIN_SATS 6U
#define AUTOSRC_GPS_MAX_HDOP 250U
#define AUTOSRC_GPS_RAW_HOLD_MS 3000U
#define AUTOSRC_GPS_NAV_CONFIRM_MS 1000U
#define AUTOSRC_GPS_HANDOVER_TIMEOUT_MS 5000U
#define AUTOSRC_GPS_HANDOVER_FAST_ROLLBACK_MS 2500U
#define AUTOSRC_FLOW_HANDOVER_TIMEOUT_MS 5000U
#define AUTOSRC_FLOW_HANDOVER_FAST_ROLLBACK_MS 2500U
#define AUTOSRC_HANDOVER_NAV_GAP_HOLD_MS 300U
#define AUTOSRC_GPS_HANDOVER_RETRY_COOLDOWN_MS 15000U
#define AUTOSRC_FLOW_HANDOVER_RETRY_COOLDOWN_MS 15000U
#define AUTOSRC_FLOW_NAV_CONFIRM_MS 500U
#define AUTOSRC_NORMAL_SOURCE_MIN_RESIDENCE_MS 5000U
#define AUTOSRC_GPS_LOSS_HOLD_MS 2000U
#define AUTOSRC_FLOW_MIN_QUALITY 50U
#define AUTOSRC_FLOW_FRESH_MS 350U
#define AUTOSRC_FLOW_GROUND_MAX_CM 30
#define AUTOSRC_FLOW_AIR_MAX_CM 250
#define AUTOSRC_FLOW_READY_HOLD_MS 800U
#define AUTOSRC_FLOW_GROUND_NAV_TIMEOUT_MS 10000U
#define AUTOSRC_FLOW_LOSS_HOLD_MS 1200U
#define AUTOSRC_BARO_GROUND_SETTLE_MS 1000U
#define AUTOSRC_BARO_GROUND_TRACK_ALPHA 0.05f
#define AUTOSRC_GPS_SWITCH_BARO_CM 200.0f
#define AUTOSRC_GPS_SWITCH_BARO_HOLD_MS 400U
#define AUTOSRC_FLOW_SWITCH_BARO_CM 170.0f
#define AUTOSRC_FLOW_SWITCH_BARO_HOLD_MS 500U
#define AUTOSRC_FLOW_MAX_BARO_CM 230.0f
#define AUTOSRC_GPS_SWITCH_MAX_XY_SPEED_CMS 60.0f
#define AUTOSRC_FLOW_SWITCH_MAX_XY_SPEED_CMS 60.0f
#define AUTOSRC_DIAG_PERIOD_MS 5000U
#define AUTOSRC_BOOT_SETTLE_MS 1500U

// Low-altitude throttle landing assist for product Loiter operation.
// A rangefinder-confirmed floor blocks only downward pilot commands; horizontal
// control and climb remain available until the pilot deliberately holds minimum
// throttle long enough to request native LAND.
#if defined(HAL_MATEKF405_UAV) && HAL_MATEKF405_UAV
#define LOWALT_THROTTLE_LAND_ENABLED 1
#else
#define LOWALT_THROTTLE_LAND_ENABLED 0
#endif
#define LOWALT_FLOOR_ENTER_CM 50
#define LOWALT_HARD_MIN_CM 40
#define LOWALT_FLOOR_RELEASE_CM 60
#define LOWALT_RANGE_CONFIRM_MS 200U
#define LOWALT_HARD_STOP_HOLD_MS 250U
#define LOWALT_THROTTLE_MIN_CONTROL 50
#define LOWALT_THROTTLE_LAND_HOLD_MS 1000U
#define LOWALT_LAND_MAX_XY_SPEED_CMS 50.0f
#define LOWALT_RANGE_FRESH_MS 350U
#define LOWALT_PREDICT_TARGET_CM 45.0f
#define LOWALT_PREDICT_SENSOR_LATENCY_MS 120U
#define LOWALT_PREDICT_MARGIN_CM 3.0f
#define LOWALT_PREDICT_ACCEL_SCALE 0.70f
#define LOWALT_PREDICT_MIN_ACCEL_CMSS 50.0f
#define LOWALT_PREDICT_MAX_TRIGGER_CM 100.0f

namespace {

enum class OneKeyTakeoffState : uint8_t {
    IDLE = 0,
    WAIT_POSITION,
    WAIT_SPOOL,
    WAIT_IDLE_HOLD,
    WAIT_LIFTOFF
};

static bool onekey_rc_input_fresh()
{
    if (!rc().has_valid_input()) {
        return false;
    }
    return (AP_HAL::millis() - rc().last_input_ms()) <= ONEKEY_RC_INPUT_TIMEOUT_MS;
}

static OneKeyTakeoffState onekey_takeoff_state = OneKeyTakeoffState::IDLE;
static uint32_t onekey_takeoff_phase_start_ms = 0U;
static uint32_t onekey_liftoff_above_since_ms = 0U;
static float onekey_pending_takeoff_alt_cm = 0.0f;

#if LOWALT_THROTTLE_LAND_ENABLED
enum class LowAltLandingState : uint8_t {
    NORMAL = 0,
    HOLD,
};

static LowAltLandingState lowalt_landing_state = LowAltLandingState::NORMAL;
static uint32_t lowalt_enter_since_ms = 0U;
static uint32_t lowalt_release_since_ms = 0U;
static uint32_t lowalt_throttle_land_since_ms = 0U;
static uint32_t lowalt_hard_stop_until_ms = 0U;
static uint32_t lowalt_predict_stop_until_ms = 0U;
static uint32_t lowalt_last_range_sample_ms = 0U;
static bool lowalt_land_request_pending = false;
static bool product_land_stick_lock = false;
static bool product_land_return_loiter_pending = false;
#endif

#if AUTO_SOURCE_MANAGER_ENABLED
enum class AutoSourceState : uint8_t {
    INIT = 0,
    FLOW_GROUND,
    FLOW_ACTIVE,
    GPS_GROUND,
    GPS_HANDOVER,
    GPS_ACTIVE,
    FLOW_HANDOVER,
    FLOW_RECOVERY,
};

static AutoSourceState autosrc_state = AutoSourceState::INIT;
static uint32_t autosrc_boot_ms = 0U;
static uint32_t autosrc_gps_raw_since_ms = 0U;
static uint32_t autosrc_gps_nav_since_ms = 0U;
static uint32_t autosrc_gps_bad_since_ms = 0U;
static uint32_t autosrc_flow_ground_since_ms = 0U;
static uint32_t autosrc_flow_air_since_ms = 0U;
static uint32_t autosrc_flow_bad_since_ms = 0U;
static uint32_t autosrc_transition_start_ms = 0U;
static uint32_t autosrc_gps_handover_gap_since_ms = 0U;
static uint32_t autosrc_flow_handover_gap_since_ms = 0U;
static uint32_t autosrc_last_handover_fail_ms = 0U;
static uint32_t autosrc_last_flow_handover_fail_ms = 0U;
static uint32_t autosrc_last_successful_handover_ms = 0U;
static uint32_t autosrc_flow_nav_since_ms = 0U;
static uint32_t autosrc_flow_ground_selected_ms = 0U;
static bool autosrc_flow_ground_suppressed = false;
static float autosrc_baro_ground_ref_cm = 0.0f;
static bool autosrc_baro_ground_ref_valid = false;
static uint32_t autosrc_baro_ground_track_since_ms = 0U;
static uint32_t autosrc_baro_switch_since_ms = 0U;
static uint32_t autosrc_baro_flow_switch_since_ms = 0U;
static bool autosrc_gps_raw_ready_cached = false;
static bool autosrc_gps_nav_ready_cached = false;
static bool autosrc_flow_ground_ready_cached = false;
static uint32_t autosrc_last_diag_ms = 0U;

static void autosrc_select_source(const uint8_t source_set, const char *reason)
{
    AP_AHRS &ahrs_ref = AP::ahrs();
    if (ahrs_ref.get_posvelyaw_source_set() != source_set) {
        ahrs_ref.set_posvelyaw_source_set(source_set);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoSrc -> SRC%u %s",
                      unsigned(source_set + 1U), reason);
    }
}
#endif

static void onekey_reset_takeoff_state()
{
    onekey_takeoff_state = OneKeyTakeoffState::IDLE;
    onekey_takeoff_phase_start_ms = 0U;
    onekey_liftoff_above_since_ms = 0U;
    onekey_pending_takeoff_alt_cm = 0.0f;
}

} // namespace

// Unified takeoff permission for both manual rudder arming and RC8 OneKey.
// The selected source must satisfy the AutoSource manager's own readiness
// contract; native position/arming checks remain in force in addition to this.
void Copter::low_alt_landing_guard_reset()
{
#if LOWALT_THROTTLE_LAND_ENABLED
    lowalt_landing_state = LowAltLandingState::NORMAL;
    lowalt_enter_since_ms = 0U;
    lowalt_release_since_ms = 0U;
    lowalt_throttle_land_since_ms = 0U;
    lowalt_hard_stop_until_ms = 0U;
    lowalt_predict_stop_until_ms = 0U;
    lowalt_last_range_sample_ms = 0U;
    lowalt_land_request_pending = false;
#endif
}

bool Copter::product_land_stick_locked() const
{
#if LOWALT_THROTTLE_LAND_ENABLED
    return product_land_stick_lock;
#else
    return false;
#endif
}

void Copter::low_alt_landing_guard(float &target_climb_rate)
{
#if LOWALT_THROTTLE_LAND_ENABLED
    const uint32_t now_ms = AP_HAL::millis();

    // This helper is intentionally called only from Loiter's normal Flying
    // state.  It may clamp downward demand and queue a LAND request, but it
    // must never change flight mode from inside ModeLoiter::run().
    if (!motors->armed() ||
        ap.land_complete ||
        (flightmode != &mode_loiter) ||
        failsafe.radio) {
        low_alt_landing_guard_reset();
        return;
    }

    const uint32_t range_sample_ms =
        rangefinder.last_reading_ms(ROTATION_PITCH_270);
    const bool range_fresh =
        rangefinder_alt_ok() &&
        (range_sample_ms != 0U) &&
        ((now_ms - range_sample_ms) <= LOWALT_RANGE_FRESH_MS);
    const int32_t range_cm = range_fresh ? int32_t(rangefinder_state.alt_cm) : -1;
    const bool new_range_sample =
        range_fresh &&
        (range_sample_ms != lowalt_last_range_sample_ms);

    // Predict the distance needed to stop an existing descent.  Use 70% of
    // the configured Z acceleration as a conservative braking estimate, then
    // add one sensor-latency allowance and a small range margin.  This is only
    // a transient descent clamp; it never latches LAND or the low-alt floor
    // from a single sample.
    const float down_speed_cms =
        MAX(0.0f, -inertial_nav.get_velocity_z_up_cms());
    const float configured_accel_cmss =
        MAX(pos_control->get_max_accel_z_cmss(), LOWALT_PREDICT_MIN_ACCEL_CMSS);
    const float brake_accel_cmss =
        MAX(configured_accel_cmss * LOWALT_PREDICT_ACCEL_SCALE,
            LOWALT_PREDICT_MIN_ACCEL_CMSS);
    const float stopping_distance_cm =
        (down_speed_cms * down_speed_cms) / (2.0f * brake_accel_cmss);
    const float sensor_travel_cm =
        down_speed_cms * (float(LOWALT_PREDICT_SENSOR_LATENCY_MS) * 0.001f);
    const float predicted_trigger_cm =
        MIN(LOWALT_PREDICT_MAX_TRIGGER_CM,
            LOWALT_PREDICT_TARGET_CM +
            stopping_distance_cm +
            sensor_travel_cm +
            LOWALT_PREDICT_MARGIN_CM);

    if (new_range_sample) {
        lowalt_last_range_sample_ms = range_sample_ms;

        // A single <=40 cm sample immediately pauses further descent for a
        // short bounded interval, but never permanently latches the floor.
        if (range_cm <= LOWALT_HARD_MIN_CM) {
            lowalt_hard_stop_until_ms = now_ms + LOWALT_HARD_STOP_HOLD_MS;
        } else {
            lowalt_hard_stop_until_ms = 0U;
        }

        // While actually descending, begin braking early enough that the
        // vehicle should settle near the 40-50 cm band instead of overshooting
        // toward 20 cm. A single anomalous low sample can only cause a short
        // pause; the persistent floor still requires the 200 ms confirmation.
        if ((target_climb_rate < 0.0f) &&
            (down_speed_cms > 5.0f) &&
            (float(range_cm) <= predicted_trigger_cm)) {
            lowalt_predict_stop_until_ms =
                now_ms + LOWALT_HARD_STOP_HOLD_MS;
        }

        if (lowalt_landing_state == LowAltLandingState::NORMAL) {
            lowalt_release_since_ms = 0U;

            if (range_cm <= LOWALT_FLOOR_ENTER_CM) {
                if (lowalt_enter_since_ms == 0U) {
                    lowalt_enter_since_ms = range_sample_ms;
                } else if ((range_sample_ms - lowalt_enter_since_ms) >=
                           LOWALT_RANGE_CONFIRM_MS) {
                    lowalt_landing_state = LowAltLandingState::HOLD;
                    lowalt_enter_since_ms = 0U;
                    lowalt_throttle_land_since_ms = 0U;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                                  "LowAlt floor active r%ldcm",
                                  long(range_cm));
                }
            } else {
                lowalt_enter_since_ms = 0U;
            }
        } else {
            // Once the floor is latched, NoData never releases it. Only a
            // confirmed valid >60 cm sequence returns to normal flight.
            lowalt_enter_since_ms = 0U;
            if (range_cm > LOWALT_FLOOR_RELEASE_CM) {
                if (lowalt_release_since_ms == 0U) {
                    lowalt_release_since_ms = range_sample_ms;
                } else if ((range_sample_ms - lowalt_release_since_ms) >=
                           LOWALT_RANGE_CONFIRM_MS) {
                    lowalt_landing_state = LowAltLandingState::NORMAL;
                    lowalt_release_since_ms = 0U;
                    lowalt_throttle_land_since_ms = 0U;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                                  "LowAlt floor released r%ldcm",
                                  long(range_cm));
                }
            } else {
                lowalt_release_since_ms = 0U;
            }
        }
    }

    const bool hard_stop_active =
        (range_fresh && (range_cm <= LOWALT_HARD_MIN_CM)) ||
        ((lowalt_hard_stop_until_ms != 0U) &&
         (int32_t(lowalt_hard_stop_until_ms - now_ms) > 0));
    const bool predictive_stop_active =
        (lowalt_predict_stop_until_ms != 0U) &&
        (int32_t(lowalt_predict_stop_until_ms - now_ms) > 0);
    const bool floor_active =
        (lowalt_landing_state == LowAltLandingState::HOLD);

    // The guard is deliberately asymmetric: only downward pilot demand is
    // blocked. Horizontal Loiter and upward climb remain fully available.
    if ((floor_active || hard_stop_active || predictive_stop_active) &&
        (target_climb_rate < 0.0f)) {
        target_climb_rate = 0.0f;
    }

    if (!floor_active) {
        lowalt_throttle_land_since_ms = 0U;
        lowalt_land_request_pending = false;
        return;
    }

    // LAND confirmation requires a current valid low-altitude range as well
    // as the latched floor. If the laser goes NoData over an edge, the floor
    // remains protective but LAND cannot be triggered blindly.
    const bool land_height_confirmed =
        range_fresh &&
        (range_cm >= 0) &&
        (range_cm <= LOWALT_FLOOR_RELEASE_CM);
    const bool throttle_at_min =
        (channel_throttle != nullptr) &&
        (channel_throttle->get_control_in() <= LOWALT_THROTTLE_MIN_CONTROL);
    const bool xy_slow =
        inertial_nav.get_velocity_xy_cms().length() <=
        LOWALT_LAND_MAX_XY_SPEED_CMS;

    if (land_height_confirmed && throttle_at_min && xy_slow) {
        if (lowalt_throttle_land_since_ms == 0U) {
            lowalt_throttle_land_since_ms = now_ms;
        } else if ((now_ms - lowalt_throttle_land_since_ms) >=
                   LOWALT_THROTTLE_LAND_HOLD_MS) {
            // Queue the request only. userhook_50Hz() performs the actual mode
            // transition outside the active Loiter control stack.
            if (!lowalt_land_request_pending) {
                lowalt_land_request_pending = true;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                              "LowAlt LAND request queued");
            }
            target_climb_rate = 0.0f;
        }
    } else {
        lowalt_throttle_land_since_ms = 0U;
        lowalt_land_request_pending = false;
    }
#else
    (void)target_climb_rate;
#endif
}

bool Copter::autosrc_takeoff_ready()
{
#if AUTO_SOURCE_MANAGER_ENABLED
    const uint8_t source_set = AP::ahrs().get_posvelyaw_source_set();
    const nav_filter_status filter = inertial_nav.get_filter_status();

    if (source_set == AUTOSRC_FLOW_SOURCE_SET) {
        // GPS navigation also reports horiz_pos_rel, so require that absolute
        // GPS position is no longer the active aiding mode.
        return autosrc_flow_ground_ready_cached &&
               filter.flags.horiz_pos_rel &&
               !filter.flags.horiz_pos_abs &&
               !filter.flags.const_pos_mode;
    }

    if (source_set == AUTOSRC_GPS_SOURCE_SET) {
        return autosrc_gps_raw_ready_cached &&
               autosrc_gps_nav_ready_cached &&
               filter.flags.horiz_pos_abs &&
               filter.flags.using_gps &&
               filter.flags.gps_quality_good &&
               !filter.flags.gps_glitching &&
               !filter.flags.const_pos_mode;
    }

    return false;
#else
    return position_ok();
#endif
}

// MatekF405 + RZ7889 RC7 hardware-PWM gimbal control.
// Hardware path:
//   M5 / PA15 / TIM2_CH1 / PWM5 -> RZ7889 A1
//   M6 / PA8  / TIM1_CH1 / PWM6 -> RZ7889 B1
// RCOutput owns the timer mode/frequency; SRV_Channels owns all periodic values.
#if defined(HAL_MATEKF405_UAV) && HAL_MATEKF405_UAV
#define GIMBAL_RZ7889_RC7_CONTROL_ENABLED 1
#else
#define GIMBAL_RZ7889_RC7_CONTROL_ENABLED 0
#endif
#define GIMBAL_PWM5_CH 4U
#define GIMBAL_PWM6_CH 5U
#define GIMBAL_PWM_CH_MASK ((1UL << GIMBAL_PWM5_CH) | (1UL << GIMBAL_PWM6_CH))
#define GIMBAL_RC7_INDEX 6U
#define GIMBAL_RC_CENTER_PWM 1500U
#define GIMBAL_RC_DEADZONE_PWM 80U
#define GIMBAL_RC_INPUT_TIMEOUT_MS 500U
#define GIMBAL_OUTPUT_OVERRIDE_TIMEOUT_MS 100U
#define GIMBAL_RC_MIN_VALID_PWM 800U
#define GIMBAL_RC_MAX_VALID_PWM 2200U

// Hardware carrier and short-period density modulation. During the ON window
// only the requested direction receives 15% hardware PWM; during the OFF
// window both RZ7889 inputs are held at 0%.
#define GIMBAL_HW_PWM_FREQUENCY_HZ 1000U
#define GIMBAL_MANUAL_DUTY_PERCENT 15U
#define GIMBAL_DENSITY_ON_MS 10U
#define GIMBAL_DENSITY_OFF_MS 50U
#define GIMBAL_DENSITY_PERIOD_MS (GIMBAL_DENSITY_ON_MS + GIMBAL_DENSITY_OFF_MS)

// Auto-home remains implemented but deliberately disabled for this build.
#define GIMBAL_AUTO_HOME_ENABLED 0
#define GIMBAL_AUTO_HOME_DELAY_MS 2000U
#define GIMBAL_AUTO_HOME_TIME_MS 800U
#define GIMBAL_AUTO_HOME_DUTY_PERCENT 20U
#define GIMBAL_AUTO_HOME_USE_M5_DIRECTION 1

#if GIMBAL_RZ7889_RC7_CONTROL_ENABLED
namespace {

static constexpr SRV_Channel::Aux_servo_function_t GIMBAL_M5_SRV_FUNCTION = SRV_Channel::k_scripting1;
static constexpr SRV_Channel::Aux_servo_function_t GIMBAL_M6_SRV_FUNCTION = SRV_Channel::k_scripting2;

enum class GimbalDrive : uint8_t {
    Stop = 0,
    M5 = 1,
    M6 = 2,
};

static GimbalDrive gimbal_requested_drive = GimbalDrive::Stop;
static uint8_t gimbal_requested_duty_percent = 0;
static uint32_t gimbal_last_command_ms = 0;
static uint16_t gimbal_pwm_min = 1000U;
static uint16_t gimbal_pwm_max = 2000U;
static bool gimbal_hw_pwm_ready = false;
static GimbalDrive gimbal_density_drive = GimbalDrive::Stop;
static uint32_t gimbal_density_cycle_start_ms = 0;

static void gimbal_request_drive(const GimbalDrive drive, const uint8_t duty_percent)
{
    gimbal_requested_drive = drive;
    gimbal_requested_duty_percent = MIN(duty_percent, 100U);
    gimbal_last_command_ms = AP_HAL::millis();
}

static uint16_t gimbal_duty_to_pwm(const uint8_t duty_percent)
{
    const uint32_t duty = MIN(duty_percent, 100U);
    const uint32_t span = uint32_t(gimbal_pwm_max - gimbal_pwm_min);
    return uint16_t(uint32_t(gimbal_pwm_min) + ((span * duty + 50U) / 100U));
}

static bool gimbal_read_rc7_pwm(uint16_t &rc_pwm)
{
    if (!rc().has_valid_input()) {
        return false;
    }

    if (RC_Channels::get_valid_channel_count() <= GIMBAL_RC7_INDEX) {
        return false;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - rc().last_input_ms() > GIMBAL_RC_INPUT_TIMEOUT_MS) {
        return false;
    }

    RC_Channel *rc7 = rc().channel(GIMBAL_RC7_INDEX);
    if (rc7 == nullptr) {
        return false;
    }

    const int16_t radio_in = rc7->get_radio_in();
    if ((radio_in < int16_t(GIMBAL_RC_MIN_VALID_PWM)) ||
        (radio_in > int16_t(GIMBAL_RC_MAX_VALID_PWM))) {
        return false;
    }

    rc_pwm = uint16_t(radio_in);
    return true;
}

#if GIMBAL_AUTO_HOME_ENABLED
static bool gimbal_rc_pwm_in_deadzone(const uint16_t rc_pwm)
{
    return (rc_pwm >= (GIMBAL_RC_CENTER_PWM - GIMBAL_RC_DEADZONE_PWM)) &&
           (rc_pwm <= (GIMBAL_RC_CENTER_PWM + GIMBAL_RC_DEADZONE_PWM));
}
#endif

static void gimbal_update_manual_request_from_rc7()
{
    uint16_t rc_pwm = GIMBAL_RC_CENTER_PWM;
    if (!gimbal_read_rc7_pwm(rc_pwm)) {
        gimbal_request_drive(GimbalDrive::Stop, 0);
        return;
    }

    if (rc_pwm > (GIMBAL_RC_CENTER_PWM + GIMBAL_RC_DEADZONE_PWM)) {
        gimbal_request_drive(GimbalDrive::M5, GIMBAL_MANUAL_DUTY_PERCENT);
    } else if (rc_pwm < (GIMBAL_RC_CENTER_PWM - GIMBAL_RC_DEADZONE_PWM)) {
        gimbal_request_drive(GimbalDrive::M6, GIMBAL_MANUAL_DUTY_PERCENT);
    } else {
        gimbal_request_drive(GimbalDrive::Stop, 0);
    }
}

static bool gimbal_update_auto_home_request()
{
#if GIMBAL_AUTO_HOME_ENABLED
    static const uint32_t boot_ms = AP_HAL::millis();
    static bool home_started = false;
    static bool home_done = false;
    static uint32_t home_start_ms = 0;

    if (home_done) {
        return false;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - boot_ms < GIMBAL_AUTO_HOME_DELAY_MS) {
        gimbal_request_drive(GimbalDrive::Stop, 0);
        return true;
    }

    if (!home_started) {
        uint16_t rc_pwm = GIMBAL_RC_CENTER_PWM;
        if (!gimbal_read_rc7_pwm(rc_pwm) || !gimbal_rc_pwm_in_deadzone(rc_pwm)) {
            gimbal_request_drive(GimbalDrive::Stop, 0);
            return true;
        }
        home_started = true;
        home_start_ms = now_ms;
    }

    if (now_ms - home_start_ms < GIMBAL_AUTO_HOME_TIME_MS) {
#if GIMBAL_AUTO_HOME_USE_M5_DIRECTION
        gimbal_request_drive(GimbalDrive::M5, GIMBAL_AUTO_HOME_DUTY_PERCENT);
#else
        gimbal_request_drive(GimbalDrive::M6, GIMBAL_AUTO_HOME_DUTY_PERCENT);
#endif
        return true;
    }

    gimbal_request_drive(GimbalDrive::Stop, 0);
    home_done = true;
    return true;
#else
    return false;
#endif
}

static bool gimbal_prepare_srv_channels()
{
    // Reserve one independent SRV function per RZ7889 input. If either function
    // is already assigned elsewhere, fail closed instead of taking it over.
    if (!SRV_Channels::set_aux_channel_default(GIMBAL_M5_SRV_FUNCTION, GIMBAL_PWM5_CH) ||
        !SRV_Channels::set_aux_channel_default(GIMBAL_M6_SRV_FUNCTION, GIMBAL_PWM6_CH)) {
        return false;
    }

    SRV_Channels::update_aux_servo_function();
    if ((SRV_Channels::channel_function(GIMBAL_PWM5_CH) != GIMBAL_M5_SRV_FUNCTION) ||
        (SRV_Channels::channel_function(GIMBAL_PWM6_CH) != GIMBAL_M6_SRV_FUNCTION)) {
        return false;
    }

    SRV_Channel *m5_channel = SRV_Channels::get_channel_for(GIMBAL_M5_SRV_FUNCTION);
    SRV_Channel *m6_channel = SRV_Channels::get_channel_for(GIMBAL_M6_SRV_FUNCTION);
    if ((m5_channel == nullptr) || (m6_channel == nullptr)) {
        return false;
    }

    // In brushed mode MIN is 0% duty. Make zero scaled output and timeout
    // fallback map to MIN, never to the normal-servo 1500us midpoint.
    m5_channel->set_output_min(gimbal_pwm_min);
    m5_channel->set_output_max(gimbal_pwm_max);
    m6_channel->set_output_min(gimbal_pwm_min);
    m6_channel->set_output_max(gimbal_pwm_max);
    SRV_Channels::set_trim_to_pwm_for(GIMBAL_M5_SRV_FUNCTION, gimbal_pwm_min);
    SRV_Channels::set_trim_to_pwm_for(GIMBAL_M6_SRV_FUNCTION, gimbal_pwm_min);
    SRV_Channels::set_output_scaled(GIMBAL_M5_SRV_FUNCTION, 0.0f);
    SRV_Channels::set_output_scaled(GIMBAL_M6_SRV_FUNCTION, 0.0f);

    hal.rcout->enable_ch(GIMBAL_PWM5_CH);
    hal.rcout->enable_ch(GIMBAL_PWM6_CH);
    return true;
}

static bool gimbal_verify_hw_pwm()
{
    uint32_t mode_mask = 0;
    const AP_HAL::RCOutput::output_mode mode = hal.rcout->get_output_mode(mode_mask);
    return (mode == AP_HAL::RCOutput::MODE_PWM_BRUSHED) &&
           ((mode_mask & GIMBAL_PWM_CH_MASK) == GIMBAL_PWM_CH_MASK) &&
           (hal.rcout->get_freq(GIMBAL_PWM5_CH) == GIMBAL_HW_PWM_FREQUENCY_HZ) &&
           (hal.rcout->get_freq(GIMBAL_PWM6_CH) == GIMBAL_HW_PWM_FREQUENCY_HZ);
}

static void gimbal_disable_hw_pwm()
{
    // MODE_PWM_NONE stops the timer groups, so a later 1 Hz auxiliary-servo
    // enable pass cannot accidentally expose a normal-servo pulse on A1/B1.
    hal.rcout->set_output_mode(GIMBAL_PWM_CH_MASK, AP_HAL::RCOutput::MODE_PWM_NONE);
    hal.rcout->disable_ch(GIMBAL_PWM5_CH);
    hal.rcout->disable_ch(GIMBAL_PWM6_CH);
    gimbal_hw_pwm_ready = false;
}

static bool gimbal_density_output_enabled(const uint32_t now_ms)
{
    if ((gimbal_requested_drive == GimbalDrive::Stop) ||
        (gimbal_requested_duty_percent == 0U)) {
        gimbal_density_drive = GimbalDrive::Stop;
        gimbal_density_cycle_start_ms = now_ms;
        return false;
    }

    // A new direction always starts a fresh 10 ms ON window. This also resets
    // the envelope after RC loss/deadzone instead of inheriting an old phase.
    if (gimbal_density_drive != gimbal_requested_drive) {
        gimbal_density_drive = gimbal_requested_drive;
        gimbal_density_cycle_start_ms = now_ms;
        return true;
    }

    const uint32_t cycle_ms = (now_ms - gimbal_density_cycle_start_ms) %
                              GIMBAL_DENSITY_PERIOD_MS;
    return cycle_ms < GIMBAL_DENSITY_ON_MS;
}

static void gimbal_apply_hardware_pwm()
{
    if (!gimbal_hw_pwm_ready) {
        return;
    }

    uint16_t m5_pwm = gimbal_pwm_min;
    uint16_t m6_pwm = gimbal_pwm_min;

    const uint32_t now_ms = AP_HAL::millis();
    const bool command_fresh =
        (now_ms - gimbal_last_command_ms) <= GIMBAL_RC_INPUT_TIMEOUT_MS;
    if (command_fresh && gimbal_density_output_enabled(now_ms)) {
        const uint16_t active_pwm = gimbal_duty_to_pwm(gimbal_requested_duty_percent);
        if ((gimbal_requested_drive == GimbalDrive::M5) &&
            (gimbal_requested_duty_percent > 0U)) {
            m5_pwm = active_pwm;
        } else if ((gimbal_requested_drive == GimbalDrive::M6) &&
                   (gimbal_requested_duty_percent > 0U)) {
            m6_pwm = active_pwm;
        }
    } else if (!command_fresh) {
        // A stale RC command immediately invalidates the old density phase.
        gimbal_density_drive = GimbalDrive::Stop;
        gimbal_density_cycle_start_ms = now_ms;
    }

    // Re-establish safe scaled fallbacks before applying short PWM overrides.
    // Both channel values are then published together by Copter's cork/push path.
    SRV_Channels::set_output_scaled(GIMBAL_M5_SRV_FUNCTION, 0.0f);
    SRV_Channels::set_output_scaled(GIMBAL_M6_SRV_FUNCTION, 0.0f);
    SRV_Channels::set_output_pwm_chan_timeout(GIMBAL_PWM5_CH, m5_pwm,
                                              GIMBAL_OUTPUT_OVERRIDE_TIMEOUT_MS);
    SRV_Channels::set_output_pwm_chan_timeout(GIMBAL_PWM6_CH, m6_pwm,
                                              GIMBAL_OUTPUT_OVERRIDE_TIMEOUT_MS);
}

static void gimbal_periodic_hw_pwm_check()
{
    static uint32_t last_check_ms = 0;
    const uint32_t now_ms = AP_HAL::millis();
    if (!gimbal_hw_pwm_ready || (now_ms - last_check_ms < 1000U)) {
        return;
    }
    last_check_ms = now_ms;

    if (!gimbal_verify_hw_pwm()) {
        gimbal_disable_hw_pwm();
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Gimbal HW PWM lost; outputs disabled");
    }
}

} // namespace
#endif

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    hal.gpio->pinMode(MY_CUSTOM_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->write(MY_CUSTOM_LED_PIN, 1);

#if AUTO_SOURCE_MANAGER_ENABLED
    autosrc_boot_ms = AP_HAL::millis();
    autosrc_state = AutoSourceState::INIT;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoSrc enabled: bidir Flow/GPS");
#endif

#if GIMBAL_RZ7889_RC7_CONTROL_ENABLED
    gimbal_request_drive(GimbalDrive::Stop, 0);

    // Align RCOutput brushed scaling with the current MOT_PWM_MIN/MAX values.
    motors->update_throttle_range();
    const int16_t pwm_min = motors->get_pwm_output_min();
    const int16_t pwm_max = motors->get_pwm_output_max();
    if ((pwm_min < int16_t(GIMBAL_RC_MIN_VALID_PWM)) ||
        (pwm_max > int16_t(GIMBAL_RC_MAX_VALID_PWM)) ||
        (pwm_max <= pwm_min)) {
        gimbal_disable_hw_pwm();
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Gimbal HW PWM invalid MOT_PWM range");
        return;
    }
    gimbal_pwm_min = uint16_t(pwm_min);
    gimbal_pwm_max = uint16_t(pwm_max);

    // Ensure the pending RCOutput values are zero before changing timer mode.
    hal.rcout->cork();
    hal.rcout->write(GIMBAL_PWM5_CH, 0U);
    hal.rcout->write(GIMBAL_PWM6_CH, 0U);
    hal.rcout->push();

    if (!gimbal_prepare_srv_channels()) {
        gimbal_disable_hw_pwm();
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Gimbal HW PWM SERVO5/6 unavailable");
        return;
    }

    // Order is intentional: normal PWM clamps rates above 400 Hz.
    hal.rcout->set_output_mode(GIMBAL_PWM_CH_MASK, AP_HAL::RCOutput::MODE_PWM_BRUSHED);
    hal.rcout->set_freq(GIMBAL_PWM_CH_MASK, GIMBAL_HW_PWM_FREQUENCY_HZ);

    if (!gimbal_verify_hw_pwm()) {
        gimbal_disable_hw_pwm();
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Gimbal HW PWM init verification failed");
        return;
    }

    gimbal_hw_pwm_ready = true;
    gimbal_apply_hardware_pwm();
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Gimbal HW PWM: 1000Hz 15%%, density 10/50ms");
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Gimbal auto-home: OFF");
#endif
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
#if GIMBAL_RZ7889_RC7_CONTROL_ENABLED
    // Copter schedules this hook at 100 Hz, allowing a real 10 ms envelope
    // window. The 1 kHz carrier itself remains generated by hardware timers.
    gimbal_apply_hardware_pwm();
#endif
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    const uint32_t onekey_now_ms = AP_HAL::millis();

#if LOWALT_THROTTLE_LAND_ENABLED
    // Product-triggered LAND returns to the product's single normal standby
    // mode only after native LAND has completed and the vehicle is disarmed.
    // Non-product LAND (failsafe/GCS/etc.) is deliberately left untouched.
    if (product_land_return_loiter_pending) {
        if (!motors->armed() && ap.land_complete && (flightmode == &mode_land)) {
            if (set_mode(Mode::Number::LOITER, ModeReason::RC_COMMAND)) {
                product_land_return_loiter_pending = false;
                product_land_stick_lock = false;
                low_alt_landing_guard_reset();
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                              "Product LAND complete; Loiter standby");
            }
        } else if (flightmode != &mode_land) {
            // Another explicit mode owner took control before the product
            // landing episode completed. Do not force a later Loiter return.
            product_land_return_loiter_pending = false;
        }
    }

    // Product LAND stick lock is scoped to the active LAND episode only.
    if (!motors->armed() ||
        (product_land_stick_lock && (flightmode != &mode_land))) {
        product_land_stick_lock = false;
    }

    // A low-altitude throttle LAND request is executed here, outside
    // ModeLoiter::run(). Revalidate every safety gate immediately before the
    // mode transition so a stale request can never force LAND.
    if (lowalt_land_request_pending) {
        const uint32_t lowalt_range_ms =
            rangefinder.last_reading_ms(ROTATION_PITCH_270);
        const bool lowalt_range_fresh =
            rangefinder_alt_ok() &&
            (lowalt_range_ms != 0U) &&
            ((onekey_now_ms - lowalt_range_ms) <= LOWALT_RANGE_FRESH_MS);
        const int32_t lowalt_range_cm =
            lowalt_range_fresh ? int32_t(rangefinder_state.alt_cm) : -1;
        const bool lowalt_request_valid =
            motors->armed() &&
            !ap.land_complete &&
            !failsafe.radio &&
            (flightmode == &mode_loiter) &&
            (lowalt_landing_state == LowAltLandingState::HOLD) &&
            lowalt_range_fresh &&
            (lowalt_range_cm >= 0) &&
            (lowalt_range_cm <= LOWALT_FLOOR_RELEASE_CM) &&
            (channel_throttle != nullptr) &&
            (channel_throttle->get_control_in() <= LOWALT_THROTTLE_MIN_CONTROL) &&
            (inertial_nav.get_velocity_xy_cms().length() <=
             LOWALT_LAND_MAX_XY_SPEED_CMS);

        lowalt_land_request_pending = false;

        if (lowalt_request_valid) {
            product_land_stick_lock = true;
            product_land_return_loiter_pending = true;
            if (!set_mode(Mode::Number::LAND, ModeReason::RC_COMMAND)) {
                product_land_stick_lock = false;
                product_land_return_loiter_pending = false;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "LowAlt throttle LAND failed");
            } else {
                low_alt_landing_guard_reset();
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                              "LowAlt throttle LAND started");
            }
        } else {
            lowalt_throttle_land_since_ms = 0U;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "LowAlt LAND request cancelled");
        }
    }
#endif

#if AUTO_SOURCE_MANAGER_ENABLED
    // ---------- automatic indoor/outdoor source selection ----------
    nav_filter_status autosrc_filter = inertial_nav.get_filter_status();
    AP_GPS &autosrc_gps = AP::gps();

    int32_t autosrc_range_cm = 0;
    const bool autosrc_range_valid =
        get_rangefinder_height_interpolated_cm(autosrc_range_cm);

    const bool autosrc_flow_sensor_ok =
        optflow.enabled() &&
        optflow.healthy() &&
        (optflow.quality() >= AUTOSRC_FLOW_MIN_QUALITY) &&
        ((onekey_now_ms - optflow.last_update()) <= AUTOSRC_FLOW_FRESH_MS);

    const bool autosrc_flow_ground_now =
        autosrc_flow_sensor_ok &&
        autosrc_range_valid &&
        // Do not impose the old 4 cm lower threshold here. The VL53 ground
        // bootstrap may report only ~2-3 cm after GNDCLEAR compensation even
        // though the sample is valid. Reject only physically invalid negative
        // values; NoData is still rejected by autosrc_range_valid.
        (autosrc_range_cm >= 0) &&
        (autosrc_range_cm <= AUTOSRC_FLOW_GROUND_MAX_CM);

    const bool autosrc_flow_air_now =
        autosrc_flow_sensor_ok &&
        autosrc_range_valid &&
        (autosrc_range_cm >= int32_t(ONEKEY_LIFTOFF_ABORT_MAX_CM / 2.0f)) &&
        (autosrc_range_cm <= AUTOSRC_FLOW_AIR_MAX_CM);

    const AP_GPS::GPS_Status autosrc_gps_status = autosrc_gps.status();
    const uint8_t autosrc_sats = autosrc_gps.num_sats();
    const uint16_t autosrc_hdop = autosrc_gps.get_hdop();
    const bool autosrc_gps_raw_now =
        (autosrc_gps_status >= AP_GPS::GPS_OK_FIX_3D) &&
        (autosrc_sats >= AUTOSRC_GPS_MIN_SATS) &&
        (autosrc_hdop > 0U) &&
        (autosrc_hdop <= AUTOSRC_GPS_MAX_HDOP) &&
        autosrc_filter.flags.gps_quality_good;

    const bool autosrc_gps_nav_now =
        autosrc_filter.flags.horiz_pos_abs &&
        autosrc_filter.flags.using_gps &&
        autosrc_filter.flags.gps_quality_good &&
        !autosrc_filter.flags.gps_glitching &&
        !autosrc_filter.flags.const_pos_mode;

    auto update_hold = [onekey_now_ms](const bool condition, uint32_t &since_ms) {
        if (condition) {
            if (since_ms == 0U) {
                since_ms = onekey_now_ms;
            }
        } else {
            since_ms = 0U;
        }
    };

    update_hold(autosrc_gps_raw_now, autosrc_gps_raw_since_ms);
    update_hold(autosrc_gps_nav_now, autosrc_gps_nav_since_ms);
    update_hold(autosrc_flow_ground_now, autosrc_flow_ground_since_ms);
    update_hold(autosrc_flow_air_now, autosrc_flow_air_since_ms);

    // Track a dedicated barometric ground reference only while disarmed and
    // landed. Once armed, the reference is frozen for the entire flight. This
    // deliberately avoids EKF local-Z origin resets when deciding when the
    // low-altitude Flow phase should hand over to GPS.
    const bool autosrc_baro_healthy = barometer.healthy();
    if (!motors->armed() && ap.land_complete) {
        if (autosrc_baro_healthy) {
            if (autosrc_baro_ground_track_since_ms == 0U) {
                autosrc_baro_ground_track_since_ms = onekey_now_ms;
                autosrc_baro_ground_ref_cm = float(baro_alt);
                autosrc_baro_ground_ref_valid = false;
            } else {
                autosrc_baro_ground_ref_cm +=
                    AUTOSRC_BARO_GROUND_TRACK_ALPHA *
                    (float(baro_alt) - autosrc_baro_ground_ref_cm);
                if ((onekey_now_ms - autosrc_baro_ground_track_since_ms) >=
                    AUTOSRC_BARO_GROUND_SETTLE_MS) {
                    autosrc_baro_ground_ref_valid = true;
                }
            }
        } else {
            autosrc_baro_ground_track_since_ms = 0U;
            autosrc_baro_ground_ref_valid = false;
        }
        autosrc_baro_switch_since_ms = 0U;
        autosrc_baro_flow_switch_since_ms = 0U;
        autosrc_flow_nav_since_ms = 0U;
        autosrc_gps_handover_gap_since_ms = 0U;
        autosrc_flow_handover_gap_since_ms = 0U;
        autosrc_last_successful_handover_ms = 0U;
        autosrc_last_flow_handover_fail_ms = 0U;
    } else if (motors->armed()) {
        // Force a fresh one-second ground reference after the next disarm.
        autosrc_baro_ground_track_since_ms = 0U;
    }

    const float autosrc_baro_rel_cm =
        autosrc_baro_ground_ref_valid ?
            (float(baro_alt) - autosrc_baro_ground_ref_cm) : 0.0f;
    const bool autosrc_baro_switch_now =
        motors->armed() &&
        autosrc_baro_ground_ref_valid &&
        autosrc_baro_healthy &&
        (autosrc_baro_rel_cm >= AUTOSRC_GPS_SWITCH_BARO_CM);
    update_hold(autosrc_baro_switch_now, autosrc_baro_switch_since_ms);
    const bool autosrc_baro_switch_ready =
        (autosrc_baro_switch_since_ms != 0U) &&
        ((onekey_now_ms - autosrc_baro_switch_since_ms) >=
         AUTOSRC_GPS_SWITCH_BARO_HOLD_MS);

    const bool autosrc_baro_flow_switch_now =
        motors->armed() &&
        autosrc_baro_ground_ref_valid &&
        autosrc_baro_healthy &&
        (autosrc_baro_rel_cm <= AUTOSRC_FLOW_SWITCH_BARO_CM);
    update_hold(autosrc_baro_flow_switch_now,
                autosrc_baro_flow_switch_since_ms);
    const bool autosrc_baro_flow_switch_ready =
        (autosrc_baro_flow_switch_since_ms != 0U) &&
        ((onekey_now_ms - autosrc_baro_flow_switch_since_ms) >=
         AUTOSRC_FLOW_SWITCH_BARO_HOLD_MS);

    const bool autosrc_gps_raw_ready =
        (autosrc_gps_raw_since_ms != 0U) &&
        ((onekey_now_ms - autosrc_gps_raw_since_ms) >= AUTOSRC_GPS_RAW_HOLD_MS);
    const bool autosrc_gps_nav_ready =
        (autosrc_gps_nav_since_ms != 0U) &&
        ((onekey_now_ms - autosrc_gps_nav_since_ms) >= AUTOSRC_GPS_NAV_CONFIRM_MS);
    const bool autosrc_flow_ground_ready =
        (autosrc_flow_ground_since_ms != 0U) &&
        ((onekey_now_ms - autosrc_flow_ground_since_ms) >= AUTOSRC_FLOW_READY_HOLD_MS);
    const bool autosrc_flow_air_ready =
        (autosrc_flow_air_since_ms != 0U) &&
        ((onekey_now_ms - autosrc_flow_air_since_ms) >= AUTOSRC_FLOW_READY_HOLD_MS);

    autosrc_gps_raw_ready_cached = autosrc_gps_raw_ready;
    autosrc_gps_nav_ready_cached = autosrc_gps_nav_ready;
    autosrc_flow_ground_ready_cached = autosrc_flow_ground_ready;

    const uint8_t autosrc_active_set = AP::ahrs().get_posvelyaw_source_set();

#if AC_AVOID_ENABLED == ENABLED
    // Replace only ArduPilot's native optical-flow height wall with the same
    // barometer-relative height reference used by the AutoSource manager.
    // Fence and proximity vertical limits remain active inside AC_Avoid.
    // If this 50 Hz producer becomes stale or the baro reference is invalid,
    // AC_Avoid automatically falls back to the native EKF height protection.
    if (AC_Avoid *avoid_ref = AP::ac_avoid()) {
        const bool flow_baro_ceiling_valid =
            motors->armed() &&
            ((autosrc_active_set == AUTOSRC_FLOW_SOURCE_SET) ||
             (autosrc_state == AutoSourceState::GPS_HANDOVER)) &&
            autosrc_baro_ground_ref_valid &&
            autosrc_baro_healthy;
        const float flow_baro_ceiling_diff_m =
            (AUTOSRC_FLOW_MAX_BARO_CM - autosrc_baro_rel_cm) * 0.01f;
        avoid_ref->set_optflow_baro_height_limit(flow_baro_ceiling_valid,
                                                 flow_baro_ceiling_diff_m);
    }
#endif

    const float autosrc_xy_speed_cms = inertial_nav.get_velocity_xy_cms().length();
    const bool autosrc_gps_retry_ready =
        (autosrc_last_handover_fail_ms == 0U) ||
        ((onekey_now_ms - autosrc_last_handover_fail_ms) >=
         AUTOSRC_GPS_HANDOVER_RETRY_COOLDOWN_MS);
    const bool autosrc_flow_retry_ready =
        (autosrc_last_flow_handover_fail_ms == 0U) ||
        ((onekey_now_ms - autosrc_last_flow_handover_fail_ms) >=
         AUTOSRC_FLOW_HANDOVER_RETRY_COOLDOWN_MS);
    const bool autosrc_source_residence_ready =
        (autosrc_last_successful_handover_ms == 0U) ||
        ((onekey_now_ms - autosrc_last_successful_handover_ms) >=
         AUTOSRC_NORMAL_SOURCE_MIN_RESIDENCE_MS);
    // A failed Flow navigation acquisition is suppressed for the remainder
    // of the current healthy-ground episode while GPS remains viable. This
    // prevents periodic Flow/GPS source oscillation before takeoff. A real
    // Flow/Range drop or loss of GPS viability permits a fresh Flow attempt.
    if (!autosrc_flow_ground_now || !autosrc_gps_raw_ready) {
        autosrc_flow_ground_suppressed = false;
    }

    // On the ground, Flow is the preferred takeoff source whenever it is
    // healthy. If it is unavailable, a stable GPS solution is the fallback.
    if (!motors->armed() && ap.land_complete &&
        ((onekey_now_ms - autosrc_boot_ms) >= AUTOSRC_BOOT_SETTLE_MS)) {
        autosrc_gps_bad_since_ms = 0U;
        autosrc_flow_bad_since_ms = 0U;

        if (autosrc_flow_ground_ready && !autosrc_flow_ground_suppressed) {
            if (autosrc_active_set != AUTOSRC_FLOW_SOURCE_SET) {
                autosrc_select_source(AUTOSRC_FLOW_SOURCE_SET, "Flow takeoff");
                autosrc_flow_ground_selected_ms = onekey_now_ms;
            } else if (autosrc_flow_ground_selected_ms == 0U) {
                autosrc_flow_ground_selected_ms = onekey_now_ms;
            }
            autosrc_state = AutoSourceState::FLOW_GROUND;

            const bool flow_nav_established =
                autosrc_filter.flags.horiz_pos_rel &&
                !autosrc_filter.flags.horiz_pos_abs &&
                !autosrc_filter.flags.const_pos_mode;
            if (flow_nav_established) {
                autosrc_flow_ground_suppressed = false;
                // Keep this timestamp at the last known-good Flow navigation
                // instant. A later transient loss must persist for the full
                // timeout before GPS fallback is allowed.
                autosrc_flow_ground_selected_ms = onekey_now_ms;
            } else if (autosrc_gps_raw_ready &&
                       (autosrc_flow_ground_selected_ms != 0U) &&
                       ((onekey_now_ms - autosrc_flow_ground_selected_ms) >=
                        AUTOSRC_FLOW_GROUND_NAV_TIMEOUT_MS)) {
                // Avoid a ground deadlock where Flow sensor data looks valid
                // but EKF relative aiding never becomes usable. Prefer GPS for
                // this attempt, then allow Flow another try after cooldown.
                autosrc_select_source(AUTOSRC_GPS_SOURCE_SET, "Flow nav timeout");
                autosrc_flow_ground_suppressed = true;
                autosrc_flow_ground_selected_ms = 0U;
                autosrc_state = AutoSourceState::GPS_GROUND;
            }
        } else if (autosrc_gps_raw_ready) {
            autosrc_select_source(AUTOSRC_GPS_SOURCE_SET, "GPS fallback");
            autosrc_flow_ground_selected_ms = 0U;
            autosrc_state = AutoSourceState::GPS_GROUND;
        }
    } else if (motors->armed() && !ap.land_complete) {
        // Normal path uses hysteresis: Flow owns low altitude and GPS owns
        // high altitude. Normal Flow->GPS occurs above 2.0 m; normal GPS->Flow
        // occurs below 1.7 m. A 30 cm deadband, minimum residence time and
        // per-direction retry cooldown prevent source chatter.
        if ((autosrc_active_set == AUTOSRC_FLOW_SOURCE_SET) &&
            (autosrc_state != AutoSourceState::FLOW_HANDOVER) &&
            (autosrc_state != AutoSourceState::FLOW_RECOVERY)) {
            autosrc_state = AutoSourceState::FLOW_ACTIVE;

            if (!autosrc_flow_air_now) {
                if (autosrc_flow_bad_since_ms == 0U) {
                    autosrc_flow_bad_since_ms = onekey_now_ms;
                }
            } else {
                autosrc_flow_bad_since_ms = 0U;
            }

            const bool flow_failed =
                (autosrc_flow_bad_since_ms != 0U) &&
                ((onekey_now_ms - autosrc_flow_bad_since_ms) >= AUTOSRC_FLOW_LOSS_HOLD_MS);

            const bool normal_gps_gate =
                autosrc_gps_raw_ready &&
                autosrc_gps_retry_ready &&
                autosrc_source_residence_ready &&
                autosrc_baro_switch_ready &&
                (autosrc_xy_speed_cms <= AUTOSRC_GPS_SWITCH_MAX_XY_SPEED_CMS) &&
                (flightmode == &mode_loiter);

            const bool emergency_gps_gate =
                flow_failed &&
                autosrc_gps_raw_ready &&
                (flightmode == &mode_loiter);

            if (normal_gps_gate || emergency_gps_gate) {
                autosrc_select_source(AUTOSRC_GPS_SOURCE_SET,
                                      emergency_gps_gate ? "Flow lost" : "high-alt GPS");
                autosrc_transition_start_ms = onekey_now_ms;
                autosrc_gps_nav_since_ms = 0U;
                autosrc_gps_handover_gap_since_ms = 0U;
                autosrc_state = AutoSourceState::GPS_HANDOVER;
            }
        } else if (autosrc_state == AutoSourceState::GPS_HANDOVER) {
            nav_filter_status gps_handover_filter = inertial_nav.get_filter_status();
            const bool gps_handover_has_horizontal_nav =
                (gps_handover_filter.flags.horiz_pos_abs ||
                 gps_handover_filter.flags.horiz_pos_rel) &&
                !gps_handover_filter.flags.const_pos_mode;
            update_hold(!gps_handover_has_horizontal_nav,
                        autosrc_gps_handover_gap_since_ms);

            const bool gps_handover_nav_gap =
                (autosrc_gps_handover_gap_since_ms != 0U) &&
                ((onekey_now_ms - autosrc_gps_handover_gap_since_ms) >=
                 AUTOSRC_HANDOVER_NAV_GAP_HOLD_MS);
            const bool gps_handover_slow =
                ((onekey_now_ms - autosrc_transition_start_ms) >=
                 AUTOSRC_GPS_HANDOVER_FAST_ROLLBACK_MS) &&
                !autosrc_gps_nav_ready;

            if ((autosrc_active_set == AUTOSRC_GPS_SOURCE_SET) &&
                autosrc_gps_nav_ready) {
                if (flightmode == &mode_loiter) {
                    // Re-anchor the Loiter target after EKF relative->absolute
                    // position reset. This avoids commanding the old Flow-frame
                    // target in the new GPS frame.
                    loiter_nav->init_target();
                }
                autosrc_state = AutoSourceState::GPS_ACTIVE;
                autosrc_gps_bad_since_ms = 0U;
                autosrc_gps_handover_gap_since_ms = 0U;
                autosrc_last_handover_fail_ms = 0U;
                autosrc_last_successful_handover_ms = onekey_now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AutoSrc GPS handover complete");
            } else if ((gps_handover_nav_gap || gps_handover_slow) &&
                       autosrc_flow_air_ready) {
                // Do not sit in a source-transition gap waiting for the full
                // hard timeout. If GPS has not established absolute aiding
                // quickly enough, or all horizontal navigation disappeared
                // for a sustained interval, immediately return to still-ready
                // Flow before the EKF failsafe counter can mature.
                autosrc_select_source(AUTOSRC_FLOW_SOURCE_SET,
                                      gps_handover_nav_gap ?
                                      "GPS handover nav gap" :
                                      "GPS handover slow");
                autosrc_transition_start_ms = onekey_now_ms;
                autosrc_gps_handover_gap_since_ms = 0U;
                autosrc_flow_nav_since_ms = 0U;
                autosrc_last_handover_fail_ms = onekey_now_ms;
                autosrc_state = AutoSourceState::FLOW_RECOVERY;
            } else if ((onekey_now_ms - autosrc_transition_start_ms) >=
                       AUTOSRC_GPS_HANDOVER_TIMEOUT_MS) {
                if (autosrc_flow_air_ready) {
                    autosrc_select_source(AUTOSRC_FLOW_SOURCE_SET, "GPS handover timeout");
                    autosrc_transition_start_ms = onekey_now_ms;
                    autosrc_gps_handover_gap_since_ms = 0U;
                    autosrc_flow_nav_since_ms = 0U;
                    autosrc_last_handover_fail_ms = onekey_now_ms;
                    autosrc_state = AutoSourceState::FLOW_RECOVERY;
                } else {
                    // No safe horizontal fallback is available. Keep GPS
                    // selected and let native EKF failsafe policy own the case.
                    autosrc_gps_handover_gap_since_ms = 0U;
                    autosrc_state = AutoSourceState::GPS_ACTIVE;
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                                  "AutoSrc GPS handover slow; no safe Flow fallback");
                }
            }
        } else if (autosrc_state == AutoSourceState::FLOW_HANDOVER) {
            nav_filter_status flow_handover_filter = inertial_nav.get_filter_status();
            const bool flow_handover_has_horizontal_nav =
                (flow_handover_filter.flags.horiz_pos_abs ||
                 flow_handover_filter.flags.horiz_pos_rel) &&
                !flow_handover_filter.flags.const_pos_mode;
            update_hold(!flow_handover_has_horizontal_nav,
                        autosrc_flow_handover_gap_since_ms);

            // EKF may keep horiz_pos_abs/using_gps flags valid after a GPS
            // origin has been established, even though SRC2 is now the
            // selected horizontal aiding set. Do not require those historical
            // capabilities to disappear. Confirm the handover from the actual
            // source-set selection plus usable relative navigation.
            const bool flow_nav_now =
                (autosrc_active_set == AUTOSRC_FLOW_SOURCE_SET) &&
                flow_handover_filter.flags.horiz_pos_rel &&
                !flow_handover_filter.flags.const_pos_mode;
            update_hold(flow_nav_now, autosrc_flow_nav_since_ms);

            if (!autosrc_flow_air_now) {
                if (autosrc_flow_bad_since_ms == 0U) {
                    autosrc_flow_bad_since_ms = onekey_now_ms;
                }
            } else {
                autosrc_flow_bad_since_ms = 0U;
            }

            const bool flow_handover_sensor_failed =
                (autosrc_flow_bad_since_ms != 0U) &&
                ((onekey_now_ms - autosrc_flow_bad_since_ms) >=
                 AUTOSRC_FLOW_LOSS_HOLD_MS);
            const bool flow_nav_confirmed =
                autosrc_flow_air_ready &&
                (autosrc_flow_nav_since_ms != 0U) &&
                ((onekey_now_ms - autosrc_flow_nav_since_ms) >=
                 AUTOSRC_FLOW_NAV_CONFIRM_MS);
            const bool flow_handover_nav_gap =
                (autosrc_flow_handover_gap_since_ms != 0U) &&
                ((onekey_now_ms - autosrc_flow_handover_gap_since_ms) >=
                 AUTOSRC_HANDOVER_NAV_GAP_HOLD_MS);
            const bool flow_handover_slow =
                ((onekey_now_ms - autosrc_transition_start_ms) >=
                 AUTOSRC_FLOW_HANDOVER_FAST_ROLLBACK_MS) &&
                !flow_nav_confirmed;

            if (flow_nav_confirmed) {
                if (flightmode == &mode_loiter) {
                    // Re-anchor after absolute->relative aiding becomes real.
                    loiter_nav->init_target();
                }
                autosrc_state = AutoSourceState::FLOW_ACTIVE;
                autosrc_flow_bad_since_ms = 0U;
                autosrc_flow_handover_gap_since_ms = 0U;
                autosrc_last_flow_handover_fail_ms = 0U;
                autosrc_last_successful_handover_ms = onekey_now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                              "AutoSrc Flow handover complete");
            } else if ((flow_handover_sensor_failed ||
                        flow_handover_nav_gap ||
                        flow_handover_slow ||
                        ((onekey_now_ms - autosrc_transition_start_ms) >=
                         AUTOSRC_FLOW_HANDOVER_TIMEOUT_MS)) &&
                       autosrc_gps_raw_ready) {
                // Normal low-altitude handover failed while GPS is still
                // viable. Roll back early on a sustained navigation gap or
                // slow Flow acquisition instead of waiting for the full hard
                // timeout, then suppress another normal Flow try.
                autosrc_last_flow_handover_fail_ms = onekey_now_ms;
                const char *flow_fail_reason =
                    flow_handover_sensor_failed ? "Flow handover lost" :
                    (flow_handover_nav_gap ? "Flow handover nav gap" :
                     (flow_handover_slow ? "Flow handover slow" :
                      "Flow handover timeout"));
                autosrc_select_source(AUTOSRC_GPS_SOURCE_SET,
                                      flow_fail_reason);
                autosrc_transition_start_ms = onekey_now_ms;
                autosrc_gps_nav_since_ms = 0U;
                autosrc_gps_handover_gap_since_ms = 0U;
                autosrc_flow_nav_since_ms = 0U;
                autosrc_flow_handover_gap_since_ms = 0U;
                autosrc_state = AutoSourceState::GPS_HANDOVER;
            } else if ((onekey_now_ms - autosrc_transition_start_ms) >=
                       AUTOSRC_FLOW_HANDOVER_TIMEOUT_MS) {
                // GPS is no longer a safe rollback target. Keep Flow selected
                // and let the existing recovery/failsafe path own the case.
                autosrc_last_flow_handover_fail_ms = onekey_now_ms;
                autosrc_flow_handover_gap_since_ms = 0U;
                autosrc_state = AutoSourceState::FLOW_RECOVERY;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "AutoSrc Flow handover slow; GPS unavailable");
            }
        } else if ((autosrc_active_set == AUTOSRC_GPS_SOURCE_SET) &&
                   (autosrc_state != AutoSourceState::FLOW_RECOVERY)) {
            if (autosrc_state != AutoSourceState::GPS_HANDOVER) {
                autosrc_state = AutoSourceState::GPS_ACTIVE;
            }

            if (!autosrc_gps_nav_now) {
                if (autosrc_gps_bad_since_ms == 0U) {
                    autosrc_gps_bad_since_ms = onekey_now_ms;
                }
            } else {
                autosrc_gps_bad_since_ms = 0U;
            }

            const bool gps_failed =
                (autosrc_gps_bad_since_ms != 0U) &&
                ((onekey_now_ms - autosrc_gps_bad_since_ms) >=
                 AUTOSRC_GPS_LOSS_HOLD_MS);

            const bool normal_flow_gate =
                !gps_failed &&
                autosrc_gps_nav_now &&
                autosrc_flow_air_ready &&
                autosrc_flow_retry_ready &&
                autosrc_source_residence_ready &&
                autosrc_baro_flow_switch_ready &&
                (autosrc_xy_speed_cms <= AUTOSRC_FLOW_SWITCH_MAX_XY_SPEED_CMS) &&
                (flightmode == &mode_loiter);

            const bool emergency_flow_gate =
                gps_failed &&
                autosrc_flow_air_ready &&
                (flightmode == &mode_loiter);

            if (emergency_flow_gate) {
                // Existing emergency degradation path: bypass normal altitude,
                // speed, residence and retry gates.
                autosrc_select_source(AUTOSRC_FLOW_SOURCE_SET, "GPS lost");
                autosrc_transition_start_ms = onekey_now_ms;
                autosrc_flow_nav_since_ms = 0U;
                autosrc_flow_air_since_ms = 0U;
                autosrc_state = AutoSourceState::FLOW_RECOVERY;
            } else if (normal_flow_gate) {
                autosrc_select_source(AUTOSRC_FLOW_SOURCE_SET, "low-alt Flow");
                autosrc_transition_start_ms = onekey_now_ms;
                autosrc_flow_nav_since_ms = 0U;
                autosrc_flow_handover_gap_since_ms = 0U;
                autosrc_flow_bad_since_ms = 0U;
                autosrc_state = AutoSourceState::FLOW_HANDOVER;
            }
        }

        if (autosrc_state == AutoSourceState::FLOW_RECOVERY) {
            nav_filter_status recovery_filter = inertial_nav.get_filter_status();
            const bool recovery_flow_nav_ok =
                (AP::ahrs().get_posvelyaw_source_set() ==
                 AUTOSRC_FLOW_SOURCE_SET) &&
                recovery_filter.flags.horiz_pos_rel &&
                !recovery_filter.flags.const_pos_mode;
            if (recovery_flow_nav_ok) {
                if (flightmode == &mode_loiter) {
                    loiter_nav->init_target();
                }
                autosrc_state = AutoSourceState::FLOW_ACTIVE;
                autosrc_last_successful_handover_ms = onekey_now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "AutoSrc Flow recovery complete");
            }
        }
    }

    // Compact diagnostic line is intentionally periodic so flight logs show
    // why a source was or was not selected without flooding MAVLink.
    if ((onekey_now_ms - autosrc_last_diag_ms) >= AUTOSRC_DIAG_PERIOD_MS) {
        autosrc_last_diag_ms = onekey_now_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "AS S%u st%u G%uN%u C%u F%u T%u P%u%u q%u r%ld b%.0f",
                      unsigned(AP::ahrs().get_posvelyaw_source_set() + 1U),
                      unsigned(autosrc_state),
                      unsigned(autosrc_gps_raw_ready),
                      unsigned(autosrc_gps_nav_ready),
                      unsigned(autosrc_gps_retry_ready),
                      unsigned(autosrc_flow_ground_ready || autosrc_flow_air_ready),
                      unsigned(autosrc_takeoff_ready()),
                      unsigned(autosrc_filter.flags.horiz_pos_abs),
                      unsigned(autosrc_filter.flags.horiz_pos_rel),
                      unsigned(optflow.quality()),
                      long(autosrc_range_valid ? autosrc_range_cm : -1),
                      double(autosrc_baro_ground_ref_valid ? autosrc_baro_rel_cm : -999.0f));
    }
#endif

    // ---------- one-key takeoff / landing ----------
    if (onekey_takeoff_state == OneKeyTakeoffState::WAIT_POSITION) {
        if (motors->armed() || arming.is_armed()) {
            onekey_reset_takeoff_state();
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO cancelled: unexpectedly armed");
        } else if (!ap.land_complete) {
            onekey_reset_takeoff_state();
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO cancelled: not landed");
        } else if (failsafe.radio || !onekey_rc_input_fresh()) {
            onekey_reset_takeoff_state();
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO abort: RC lost while waiting");
        } else if ((onekey_now_ms - onekey_takeoff_phase_start_ms) >=
                   ONEKEY_POSITION_WAIT_TIMEOUT_MS) {
            onekey_reset_takeoff_state();
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO timeout: no position");
        } else if (autosrc_takeoff_ready() && position_ok()) {
            if (!set_mode(Mode::Number::LOITER, ModeReason::RC_COMMAND)) {
                onekey_reset_takeoff_state();
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO denied: Loiter unavailable");
            } else if (!arming.arm(AP_Arming::Method::AUXSWITCH, true)) {
                onekey_reset_takeoff_state();
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO denied: arming failed");
            } else {
                onekey_takeoff_phase_start_ms = onekey_now_ms;
                onekey_takeoff_state = OneKeyTakeoffState::WAIT_SPOOL;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "OneKey position ready; waiting motor spool");
            }
        }
    } else if (onekey_takeoff_state == OneKeyTakeoffState::WAIT_SPOOL) {
        // The mode or arming state changed before takeoff began: cancel the
        // one-key sequence. If still safely landed, leave no armed vehicle
        // behind after an interrupted automatic sequence.
        if (!motors->armed()) {
            onekey_reset_takeoff_state();
        } else if (flightmode != &mode_loiter) {
            if (ap.land_complete) {
                (void)arming.disarm(AP_Arming::Method::AUXSWITCH, false);
            }
            onekey_reset_takeoff_state();
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO cancelled: mode changed");
        } else if ((onekey_now_ms - onekey_takeoff_phase_start_ms) >=
                   ONEKEY_SPOOL_READY_TIMEOUT_MS) {
            if (ap.land_complete) {
                (void)arming.disarm(AP_Arming::Method::AUXSWITCH, false);
            }
            onekey_reset_takeoff_state();
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "OneKey TO abort: motor spool timeout");
        } else if (!ap.in_arming_delay &&
                   motors->get_interlock() &&
                   (motors->get_spool_state() == AP_Motors::SpoolState::THROTTLE_UNLIMITED)) {
            if (failsafe.radio || !onekey_rc_input_fresh()) {
                if (ap.land_complete) {
                    (void)arming.disarm(AP_Arming::Method::AUXSWITCH, false);
                }
                onekey_reset_takeoff_state();
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO abort: RC lost before takeoff");
            } else if (!autosrc_takeoff_ready() || !position_ok()) {
                if (ap.land_complete) {
                    (void)arming.disarm(AP_Arming::Method::AUXSWITCH, false);
                }
                onekey_reset_takeoff_state();
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO abort: source/position lost before takeoff");
            } else if (!ap.land_complete) {
                onekey_reset_takeoff_state();
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO cancelled: no longer landed");
            } else {
                // Motor spool is genuinely ready. Hold normal idle for one
                // second before starting Takeoff so all four ESC/motors have
                // time to establish a repeatable running state.
                onekey_takeoff_state = OneKeyTakeoffState::WAIT_IDLE_HOLD;
                onekey_takeoff_phase_start_ms = onekey_now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "OneKey motor spool ready; idle hold 1s");
            }
        }
    } else if (onekey_takeoff_state == OneKeyTakeoffState::WAIT_IDLE_HOLD) {
        if (!motors->armed()) {
            onekey_reset_takeoff_state();
        } else if (flightmode != &mode_loiter) {
            if (ap.land_complete) {
                (void)arming.disarm(AP_Arming::Method::AUXSWITCH, false);
            }
            onekey_reset_takeoff_state();
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO cancelled: mode changed during idle");
        } else if (failsafe.radio || !onekey_rc_input_fresh()) {
            if (ap.land_complete) {
                (void)arming.disarm(AP_Arming::Method::AUXSWITCH, false);
            }
            onekey_reset_takeoff_state();
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO abort: RC lost during idle");
        } else if (!autosrc_takeoff_ready() || !position_ok()) {
            if (ap.land_complete) {
                (void)arming.disarm(AP_Arming::Method::AUXSWITCH, false);
            }
            onekey_reset_takeoff_state();
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO abort: source/position lost during idle");
        } else if (!ap.land_complete) {
            onekey_reset_takeoff_state();
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO cancelled: no longer landed");
        } else if (!motors->get_interlock() ||
                   (motors->get_spool_state() != AP_Motors::SpoolState::THROTTLE_UNLIMITED)) {
            (void)arming.disarm(AP_Arming::Method::AUXSWITCH, false);
            onekey_reset_takeoff_state();
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "OneKey TO abort: motor spool lost during idle");
        } else if ((onekey_now_ms - onekey_takeoff_phase_start_ms) >= ONEKEY_IDLE_HOLD_MS) {
            if (!mode_loiter.do_user_takeoff_relative(onekey_pending_takeoff_alt_cm, true)) {
                (void)arming.disarm(AP_Arming::Method::AUXSWITCH, false);
                onekey_reset_takeoff_state();
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "OneKey TO failed: takeoff start");
            } else {
                onekey_takeoff_state = OneKeyTakeoffState::WAIT_LIFTOFF;
                onekey_takeoff_phase_start_ms = onekey_now_ms;
                onekey_liftoff_above_since_ms = 0U;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "OneKey idle complete; takeoff %.0fcm",
                              double(onekey_pending_takeoff_alt_cm));
            }
        }
    } else if (onekey_takeoff_state == OneKeyTakeoffState::WAIT_LIFTOFF) {
        if (!motors->armed()) {
            onekey_reset_takeoff_state();
        } else if (flightmode != &mode_loiter) {
            // A deliberate pilot mode change after takeoff has started owns
            // the aircraft from this point; never let the watchdog disarm it.
            onekey_reset_takeoff_state();
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "OneKey TO monitor cancelled: mode changed");
        } else {
            int32_t range_alt_cm = 0;
            const bool range_valid = get_rangefinder_height_interpolated_cm(range_alt_cm);

            // Prefer a physical rangefinder confirmation. The short hold time
            // rejects a single transient sample while remaining responsive.
            const bool range_above_liftoff =
                range_valid && (range_alt_cm >= int32_t(ONEKEY_LIFTOFF_CONFIRM_CM));
            if (range_above_liftoff) {
                if (onekey_liftoff_above_since_ms == 0U) {
                    onekey_liftoff_above_since_ms = onekey_now_ms;
                } else if ((onekey_now_ms - onekey_liftoff_above_since_ms) >=
                           ONEKEY_LIFTOFF_CONFIRM_HOLD_MS) {
                    onekey_reset_takeoff_state();
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "OneKey TO liftoff confirmed");
                }
            } else {
                onekey_liftoff_above_since_ms = 0U;
            }

            if ((onekey_takeoff_state == OneKeyTakeoffState::WAIT_LIFTOFF) &&
                ((onekey_now_ms - onekey_takeoff_phase_start_ms) >=
                 ONEKEY_TAKEOFF_LIFTOFF_TIMEOUT_MS)) {
                // At the 3 s deadline, trust the healthy downward rangefinder:
                // <=8 cm means the vehicle is still physically on/very near
                // the ground. Do not let barometer/EKF-Z drift veto this check.
                // If range is unavailable, never risk an airborne auto-disarm.
                const bool definitely_not_lifted =
                    range_valid &&
                    (range_alt_cm <= int32_t(ONEKEY_LIFTOFF_ABORT_MAX_CM));

                if (definitely_not_lifted) {
                    Mode::takeoff_stop();
                    set_auto_armed(false);
                    const bool disarmed =
                        arming.disarm(AP_Arming::Method::AUXSWITCH, false);
                    onekey_reset_takeoff_state();
                    if (disarmed) {
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                                      "OneKey TO abort: no physical liftoff in 3s");
                    } else {
                        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                                      "OneKey TO abort: disarm failed");
                    }
                } else {
                    onekey_reset_takeoff_state();
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                                  "OneKey TO watchdog inconclusive; no auto-disarm");
                }
            }
        }
    }

    static uint32_t init_time_ms = AP_HAL::millis();
    static uint32_t last_toggle_time_ms = 0;
    static bool is_blinking_stage = false;
    static uint8_t led_state = 1;

    const uint32_t now_ms = AP_HAL::millis();
    if (!is_blinking_stage) {
        if (now_ms - init_time_ms >= 2000U) {
            is_blinking_stage = true;
            last_toggle_time_ms = now_ms;
        }
    } else if (now_ms - last_toggle_time_ms >= 500U) {
        led_state = !led_state;
        hal.gpio->write(MY_CUSTOM_LED_PIN, led_state);
        last_toggle_time_ms = now_ms;
    }

#if GIMBAL_RZ7889_RC7_CONTROL_ENABLED
    gimbal_periodic_hw_pwm_check();
    if (!gimbal_update_auto_home_request()) {
        gimbal_update_manual_request_from_rc7();
    }
#endif
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // Require the self-centering control to be observed at MIDDLE after boot
    // and again after every command. This prevents powering the vehicle with
    // the control held HIGH/LOW from triggering an unintended action.
    static bool center_seen = false;

    if (ch_flag == RC_Channel::AuxSwitchPos::MIDDLE) {
        center_seen = true;
        return;
    }

    if (!center_seen) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey: return switch to center first");
        return;
    }

    // Consume the center qualification immediately. Another command cannot be
    // accepted until the spring-loaded control returns through MIDDLE.
    center_seen = false;

    if (ch_flag == RC_Channel::AuxSwitchPos::HIGH) {
        if (!onekey_rc_input_fresh()) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO denied: RC invalid");
            return;
        }

        // One-key takeoff is intentionally valid only from a disarmed, landed
        // vehicle. An airborne HIGH command can never restart Takeoff.
        if (motors->armed() || arming.is_armed()) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "OneKey TO ignored: vehicle armed");
            return;
        }
        if (!ap.land_complete) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO denied: not landed");
            return;
        }

        const float takeoff_alt_cm =
            constrain_float(float(g.pilot_takeoff_alt.get()),
                            ONEKEY_MIN_TAKEOFF_ALT_CM,
                            ONEKEY_MAX_TAKEOFF_ALT_CM);
        if (g.pilot_takeoff_alt.get() < int16_t(ONEKEY_MIN_TAKEOFF_ALT_CM)) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO denied: takeoff alt too low");
            return;
        }

        // Queue the command instead of demanding an already-valid position on
        // the AUX edge. The automatic source manager may need a short time to
        // establish Flow-relative or GPS-absolute position. All normal Loiter
        // and ArduPilot arming checks are still enforced before motors arm.
        onekey_pending_takeoff_alt_cm = takeoff_alt_cm;
        onekey_takeoff_phase_start_ms = AP_HAL::millis();
        onekey_takeoff_state = OneKeyTakeoffState::WAIT_POSITION;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "OneKey queued; waiting position");
        return;
    }

    if (ch_flag == RC_Channel::AuxSwitchPos::LOW) {
        onekey_reset_takeoff_state();
        Mode::takeoff_stop();

        // Down command is the native LAND mode. Returning the self-centering
        // switch to MIDDLE does not cancel LAND.
        if (!motors->armed()) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "OneKey LAND ignored: disarmed");
            return;
        }

        if (flightmode == &mode_land) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "OneKey LAND: already active");
            return;
        }

#if LOWALT_THROTTLE_LAND_ENABLED
        product_land_stick_lock = true;
        product_land_return_loiter_pending = true;
#endif
        if (!set_mode(Mode::Number::LAND, ModeReason::RC_COMMAND)) {
#if LOWALT_THROTTLE_LAND_ENABLED
            product_land_stick_lock = false;
            product_land_return_loiter_pending = false;
#endif
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey LAND failed");
            return;
        }

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "OneKey LAND started");
    }
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
}
#endif
