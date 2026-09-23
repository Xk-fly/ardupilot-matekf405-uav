#include "Copter.h"

#define MY_CUSTOM_LED_PIN 59

// One-key takeoff / landing on USER_FUNC1 (intended for the self-centering RC7).
// HIGH edge: disarmed+landed -> LOITER -> normal arming checks -> relative
// PILOT_TKOFF_ALT takeoff -> remain in LOITER.
// LOW edge: if armed, enter native LAND.
// MIDDLE only re-arms the edge detector; returning to center never changes mode.
#define ONEKEY_RC_INPUT_TIMEOUT_MS 500U
#define ONEKEY_SPOOL_READY_TIMEOUT_MS 5000U
#define ONEKEY_TAKEOFF_LIFTOFF_TIMEOUT_MS 3000U
#define ONEKEY_LIFTOFF_CONFIRM_CM 12.0f
#define ONEKEY_LIFTOFF_ABORT_MAX_CM 8.0f
#define ONEKEY_LIFTOFF_CONFIRM_HOLD_MS 150U
#define ONEKEY_MIN_TAKEOFF_ALT_CM 10.0f
#define ONEKEY_MAX_TAKEOFF_ALT_CM 1000.0f

namespace {

enum class OneKeyTakeoffState : uint8_t {
    IDLE = 0,
    WAIT_SPOOL,
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
static float onekey_takeoff_start_inertial_z_cm = 0.0f;

static void onekey_reset_takeoff_state()
{
    onekey_takeoff_state = OneKeyTakeoffState::IDLE;
    onekey_takeoff_phase_start_ms = 0U;
    onekey_liftoff_above_since_ms = 0U;
    onekey_pending_takeoff_alt_cm = 0.0f;
    onekey_takeoff_start_inertial_z_cm = 0.0f;
}

} // namespace

// MatekF405 + RZ7889 RC9 hardware-PWM gimbal control.
// Hardware path:
//   M5 / PA15 / TIM2_CH1 / PWM5 -> RZ7889 A1
//   M6 / PA8  / TIM1_CH1 / PWM6 -> RZ7889 B1
// RCOutput owns the timer mode/frequency; SRV_Channels owns all periodic values.
#if defined(HAL_MATEKF405_UAV) && HAL_MATEKF405_UAV
#define GIMBAL_RZ7889_RC9_CONTROL_ENABLED 1
#else
#define GIMBAL_RZ7889_RC9_CONTROL_ENABLED 0
#endif
#define GIMBAL_PWM5_CH 4U
#define GIMBAL_PWM6_CH 5U
#define GIMBAL_PWM_CH_MASK ((1UL << GIMBAL_PWM5_CH) | (1UL << GIMBAL_PWM6_CH))
#define GIMBAL_RC9_INDEX 8U
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

#if GIMBAL_RZ7889_RC9_CONTROL_ENABLED
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

static bool gimbal_read_rc9_pwm(uint16_t &rc_pwm)
{
    if (!rc().has_valid_input()) {
        return false;
    }

    if (RC_Channels::get_valid_channel_count() <= GIMBAL_RC9_INDEX) {
        return false;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - rc().last_input_ms() > GIMBAL_RC_INPUT_TIMEOUT_MS) {
        return false;
    }

    RC_Channel *rc9 = rc().channel(GIMBAL_RC9_INDEX);
    if (rc9 == nullptr) {
        return false;
    }

    const int16_t radio_in = rc9->get_radio_in();
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

static void gimbal_update_manual_request_from_rc9()
{
    uint16_t rc_pwm = GIMBAL_RC_CENTER_PWM;
    if (!gimbal_read_rc9_pwm(rc_pwm)) {
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
        if (!gimbal_read_rc9_pwm(rc_pwm) || !gimbal_rc_pwm_in_deadzone(rc_pwm)) {
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

#if GIMBAL_RZ7889_RC9_CONTROL_ENABLED
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
#if GIMBAL_RZ7889_RC9_CONTROL_ENABLED
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

    if (onekey_takeoff_state == OneKeyTakeoffState::WAIT_SPOOL) {
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
            // This is deliberately delayed until after the normal ArduPilot
            // arming delay and motor spool-up. Starting Takeoff earlier skips
            // the landed/pre-takeoff branch that requests motor spool-up.
            if (!ap.land_complete) {
                onekey_reset_takeoff_state();
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO cancelled: no longer landed");
            } else if (!mode_loiter.do_user_takeoff_relative(onekey_pending_takeoff_alt_cm, true)) {
                (void)arming.disarm(AP_Arming::Method::AUXSWITCH, false);
                onekey_reset_takeoff_state();
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "OneKey TO failed: takeoff start");
            } else {
                onekey_takeoff_state = OneKeyTakeoffState::WAIT_LIFTOFF;
                onekey_takeoff_phase_start_ms = onekey_now_ms;
                onekey_liftoff_above_since_ms = 0U;
                onekey_takeoff_start_inertial_z_cm = inertial_nav.get_position_z_up_cm();
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "OneKey spool ready; takeoff %.0fcm",
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
            const float inertial_delta_cm =
                inertial_nav.get_position_z_up_cm() - onekey_takeoff_start_inertial_z_cm;

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
                // Only auto-disarm when sensors positively say the vehicle is
                // still on/very near the ground. If range is unavailable or
                // the vehicle may already be airborne, fail safe by keeping
                // it armed and merely ending this watchdog.
                const bool definitely_not_lifted =
                    range_valid &&
                    (range_alt_cm <= int32_t(ONEKEY_LIFTOFF_ABORT_MAX_CM)) &&
                    (inertial_delta_cm < 15.0f);

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

#if GIMBAL_RZ7889_RC9_CONTROL_ENABLED
    gimbal_periodic_hw_pwm_check();
    if (!gimbal_update_auto_home_request()) {
        gimbal_update_manual_request_from_rc9();
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

        // LOITER must already have a valid absolute or relative position
        // estimate. Never bypass ArduPilot's position checks.
        if (!position_ok()) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO denied: no position");
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

        // Mode first, then arm. If LOITER cannot initialise, do not arm.
        if (!set_mode(Mode::Number::LOITER, ModeReason::RC_COMMAND)) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO denied: Loiter unavailable");
            return;
        }

        // Keep all normal ArduPilot arming checks enabled.
        if (!arming.arm(AP_Arming::Method::AUXSWITCH, true)) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "OneKey TO denied: arming failed");
            return;
        }

        // Do NOT start Takeoff in this aux-switch callback. The motors need
        // ArduPilot's normal arming delay and landed/pre-takeoff loop to reach
        // THROTTLE_UNLIMITED first. userhook_50Hz() starts Takeoff only after
        // the motor spool state is genuinely ready.
        onekey_pending_takeoff_alt_cm = takeoff_alt_cm;
        onekey_takeoff_phase_start_ms = AP_HAL::millis();
        onekey_takeoff_state = OneKeyTakeoffState::WAIT_SPOOL;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "OneKey armed; waiting motor spool");
        return;
    }

    if (ch_flag == RC_Channel::AuxSwitchPos::LOW) {
        onekey_reset_takeoff_state();
        Mode::takeoff_stop();
        set_auto_armed(false);

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

        if (!set_mode(Mode::Number::LAND, ModeReason::RC_COMMAND)) {
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
