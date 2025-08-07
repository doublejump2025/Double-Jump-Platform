#include "Copter.h"
#include "ModeHybrid.h"
#include <AP_HAL/AP_HAL.h>

#if MODE_HYBRID_ENABLED

// Constants
namespace {
    //constexpr float PILOT_VELZ_MAX = 250.0f;           // Maximum pilot vertical velocity in cm/s
    constexpr float MIN_BATTERY_VOLTAGE = 10.0f;       // Minimum battery voltage for jump
    constexpr float MAX_JUMP_ANGLE_DEG = 30.0f;        // Maximum angle for safe jump
    constexpr uint32_t MIN_GROUND_TIME_MS = 1000;      // Minimum time on ground before jump
    constexpr uint32_t DOUBLE_TAP_WINDOW_MS = 500;     // Time window for double tap detection
    constexpr float FIRST_JUMP_THRUST = 0.7f;          // First jump thrust level
    constexpr float SECOND_JUMP_THRUST = 0.9f;         // Second jump thrust level
    constexpr uint32_t FIRST_JUMP_PREP_MS = 300;       // First jump preparation time
    constexpr uint32_t FIRST_JUMP_TIME_MS = 200;       // First jump duration
    constexpr uint32_t PAUSE_BETWEEN_MS = 100;         // Pause between jumps
    constexpr uint32_t SECOND_JUMP_PREP_MS = 200;      // Second jump preparation time
    constexpr uint32_t SECOND_JUMP_TIME_MS = 400;      // Second jump duration
    constexpr uint32_t TRANSITION_TIME_MS = 200;       // Transition to hover time
}

bool ModeHybrid::init(bool ignore_checks)
{
    // Initialize state
    _is_flight_mode = false;  // Start in ground mode
    _jump_state = JumpState::NOT_JUMPING;
    _mode_switch_time = AP_HAL::millis();
    _jump_phase_start_time = 0;
    
    // Initialize double-tap detection
    _ch6_prev_high = false;
    _first_tap_time = 0;
    _waiting_for_second_tap = false;
    
    // Initialize ground movement
    _ground_throttle = 0.0f;
    _ground_steering = 0.0f;
    
    // Get hybrid motors instance and verify it exists
    AP_Motors_Hybrid* hybrid_motors = get_hybrid_motors();
    if (hybrid_motors == nullptr) {
        gcs().send_text(MAV_SEVERITY_ERROR, "Hybrid mode requires HYBRID motor configuration");
        return false;
    }
    
    // Initialize hybrid motors in ground mode
    hybrid_motors->set_flight_mode(false);
    hybrid_motors->set_throttle_and_steering(0.0f, 0.0f);
    
    // Set motors to ground idle for ground mode
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
    
    // Send status message
    gcs().send_text(MAV_SEVERITY_INFO, "Hybrid mode initialized - GROUND mode");
    
    return true;
}

void ModeHybrid::run()
{
    // Handle mode switching between ground and flight
    handle_mode_switching();
    
    // Handle jump sequence if in flight mode
    if (_is_flight_mode) {
        handle_jump_sequence();
        handle_flight_mode();
    } else {
        handle_ground_mode();
    }
    
    // Update LEDs to show current state
    update_leds();
    
    // Output to motors
    motors->output();
}

// Helper function to get RC channel safely
RC_Channel* ModeHybrid::get_channel_safe(uint8_t channel_num)
{
    // RC channels are typically 0-15 (16 channels)
    if (channel_num >= 16) {
        return nullptr;
    }
    return RC_Channels::rc_channel(channel_num);
}
void ModeHybrid::handle_mode_switching()
{
    // CH7 controls mode switching (Ground/Flight)
    RC_Channel* ch7 = get_channel_safe(6); // Channel 7 is index 6
    bool ch7_high = (ch7 != nullptr) ? ch7->get_pwm() > 1800 : false;
    
    if (ch7_high && !_is_flight_mode) {
        // Switch to flight mode
        if (is_safe_to_switch_to_flight()) {
            _is_flight_mode = true;
            _mode_switch_time = AP_HAL::millis();
            _jump_state = JumpState::NOT_JUMPING;
            reset_tap_detection();
            
            // Configure motors for flight
            AP_Motors_Hybrid* hybrid_motors = get_hybrid_motors();
            if (hybrid_motors != nullptr) {
                hybrid_motors->set_flight_mode(true);
            }
            
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
            pos_control->set_alt_target_to_current_alt();
            
            gcs().send_text(MAV_SEVERITY_INFO, "Switched to FLIGHT mode");
        }
    }
    else if (!ch7_high && _is_flight_mode) {
        // Switch to ground mode (only if not jumping)
        if (_jump_state == JumpState::NOT_JUMPING || _jump_state == JumpState::HOVERING) {
            _is_flight_mode = false;
            _mode_switch_time = AP_HAL::millis();
            _jump_state = JumpState::NOT_JUMPING;
            reset_tap_detection();
            
            // Configure motors for ground
            AP_Motors_Hybrid* hybrid_motors = get_hybrid_motors();
            if (hybrid_motors != nullptr) {
                hybrid_motors->set_flight_mode(false);
                hybrid_motors->set_throttle_and_steering(0.0f, 0.0f);
            }
            
            motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
            
            gcs().send_text(MAV_SEVERITY_INFO, "Switched to GROUND mode");
        }
    }
}

// ===== GROUND MODE =====

void ModeHybrid::handle_ground_mode()
{
    // Simple tank drive calculation
    calculate_ground_movement();
    
    // Send commands to hybrid motors
    AP_Motors_Hybrid* hybrid_motors = get_hybrid_motors();
    if (hybrid_motors != nullptr) {
        hybrid_motors->set_throttle_and_steering(_ground_throttle, _ground_steering);
    }
}

void ModeHybrid::calculate_ground_movement()
{
    // Tank drive: CH2 = forward/back, CH1 = left/right steering
    _ground_throttle = get_ground_throttle();
    _ground_steering = get_ground_steering();
}

float ModeHybrid::get_ground_throttle()
{
    // Forward/backward from pitch stick (channel 2)
    if (copter.failsafe.radio || !copter.ap.rc_receiver_present) {
        return 0.0f;
    }
    
    float throttle = copter.channel_pitch->norm_input(); // -1 to +1
    
    // Apply deadzone
    const float DEADZONE = 0.05f;
    if (fabsf(throttle) < DEADZONE) {
        throttle = 0.0f;
    }
    
    return constrain_float(throttle, -1.0f, 1.0f);
}

float ModeHybrid::get_ground_steering()
{
    // Left/right from roll stick (channel 1)
    if (copter.failsafe.radio || !copter.ap.rc_receiver_present) {
        return 0.0f;
    }
    
    float steering = copter.channel_roll->norm_input(); // -1 to +1
    
    // Apply deadzone
    const float DEADZONE = 0.05f;
    if (fabsf(steering) < DEADZONE) {
        steering = 0.0f;
    }
    
    return constrain_float(steering, -1.0f, 1.0f);
}

// ===== FLIGHT MODE =====

void ModeHybrid::handle_flight_mode()
{
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    
    // Only handle normal flight if not in jump sequence
    if (_jump_state == JumpState::NOT_JUMPING) {
        calculate_flight_movement();
    }
}

void ModeHybrid::calculate_flight_movement()
{
    // Normal flight mode controls
    if (_jump_state != JumpState::NOT_JUMPING) {
        return; // Jump sequence controls the flight
    }
    
    if (copter.failsafe.radio || !copter.ap.rc_receiver_present) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0);
        attitude_control->set_throttle_out(0.5f, true, copter.g.throttle_filt);
        return;
    }
    
    // Get pilot inputs
    float target_roll = copter.channel_roll->get_control_in();
    float target_pitch = copter.channel_pitch->get_control_in();
    float target_yaw_rate = get_pilot_desired_yaw_rate(copter.channel_yaw->norm_input_dz());
    float target_throttle = copter.channel_throttle->get_control_in() / 1000.0f;
    
    // Constrain throttle
    target_throttle = constrain_float(target_throttle, 0.0f, 1.0f);
    
    // Send to attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(
        target_roll, target_pitch, target_yaw_rate);
    attitude_control->set_throttle_out(target_throttle, true, copter.g.throttle_filt);
}

// ===== JUMP SEQUENCE =====

void ModeHybrid::handle_jump_sequence()
{
    if (!_is_flight_mode) {
        return;
    }
    
    RC_Channel* ch6 = get_channel_safe(5); // Channel 6 is index 5
    bool ch6_high = (ch6 != nullptr) ? ch6->get_pwm() > 1800 : false;
    uint32_t now = AP_HAL::millis();
    
    if (_jump_state == JumpState::NOT_JUMPING) {
        detect_double_tap(ch6_high, now);
    }
    
    execute_auto_jump_sequence(now);
}

void ModeHybrid::detect_double_tap(bool ch6_high, uint32_t now)
{
    // Detect rising edge
    if (ch6_high && !_ch6_prev_high) {
        // Rising edge detected
        if (!_waiting_for_second_tap) {
            // First tap
            _first_tap_time = now;
            _waiting_for_second_tap = true;
            gcs().send_text(MAV_SEVERITY_INFO, "First tap detected");
        }
        else {
            // Potential second tap
            if (now - _first_tap_time < DOUBLE_TAP_WINDOW_MS) {
                // Valid double tap!
                if (is_safe_to_jump()) {
                    start_auto_double_jump();
                } else {
                    gcs().send_text(MAV_SEVERITY_WARNING, "Jump blocked - unsafe conditions!");
                    reset_tap_detection();
                }
            }
            else {
                // Too late, treat as new first tap
                _first_tap_time = now;
                _waiting_for_second_tap = false;
            }
        }
    }
    
    // Check for timeout
    if (_waiting_for_second_tap && (now - _first_tap_time > DOUBLE_TAP_WINDOW_MS)) {
        reset_tap_detection();
    }
    
    _ch6_prev_high = ch6_high;
}

void ModeHybrid::start_auto_double_jump()
{
    _jump_state = JumpState::FIRST_JUMP_PREP;
    _jump_phase_start_time = AP_HAL::millis();
    reset_tap_detection();
    
    // Save current altitude for reference
    pos_control->set_alt_target_to_current_alt();
    
    gcs().send_text(MAV_SEVERITY_INFO, "AUTO DOUBLE JUMP STARTED!");
}

void ModeHybrid::execute_auto_jump_sequence(uint32_t now)
{
    uint32_t elapsed = now - _jump_phase_start_time;
    
    switch (_jump_state) {
        case JumpState::NOT_JUMPING:
            // Nothing to do
            break;
            
        case JumpState::FIRST_JUMP_PREP:
            if (elapsed < FIRST_JUMP_PREP_MS) {
                // Ramp up thrust
                float progress = float(elapsed) / float(FIRST_JUMP_PREP_MS);
                float thrust = progress * FIRST_JUMP_THRUST;
                set_jump_thrust(thrust);
            } else {
                advance_to_next_phase(JumpState::FIRST_JUMP_THRUST, "FIRST JUMP!");
            }
            break;
            
        case JumpState::FIRST_JUMP_THRUST:
            if (elapsed < FIRST_JUMP_TIME_MS) {
                set_jump_thrust(FIRST_JUMP_THRUST);
            } else {
                advance_to_next_phase(JumpState::PAUSE_BETWEEN_JUMPS, "Pause between jumps");
            }
            break;
            
        case JumpState::PAUSE_BETWEEN_JUMPS:
            if (elapsed < PAUSE_BETWEEN_MS) {
                set_jump_thrust(0.5f); // Maintain some thrust
            } else {
                advance_to_next_phase(JumpState::SECOND_JUMP_PREP, "Second jump prep");
            }
            break;
            
        case JumpState::SECOND_JUMP_PREP:
            if (elapsed < SECOND_JUMP_PREP_MS) {
                // Ramp up from hover to second jump thrust
                float progress = float(elapsed) / float(SECOND_JUMP_PREP_MS);
                float thrust = 0.5f + (progress * (SECOND_JUMP_THRUST - 0.5f));
                set_jump_thrust(thrust);
            } else {
                advance_to_next_phase(JumpState::SECOND_JUMP_THRUST, "SECOND JUMP!");
            }
            break;
            
        case JumpState::SECOND_JUMP_THRUST:
            if (elapsed < SECOND_JUMP_TIME_MS) {
                set_jump_thrust(SECOND_JUMP_THRUST);
            } else {
                advance_to_next_phase(JumpState::TRANSITION_TO_ALTHOLD, "Transitioning to hover");
            }
            break;
            
        case JumpState::TRANSITION_TO_ALTHOLD:
            if (elapsed < TRANSITION_TIME_MS) {
                // Smooth transition from jump thrust to hover
                float progress = float(elapsed) / float(TRANSITION_TIME_MS);
                float thrust = SECOND_JUMP_THRUST * (1.0f - progress) + 0.5f * progress;
                set_jump_thrust(thrust);
            } else {
                // Jump complete, switch to hovering
                _jump_state = JumpState::HOVERING;
                pos_control->set_alt_target_to_current_alt();
                gcs().send_text(MAV_SEVERITY_INFO, "JUMP COMPLETE - Now hovering!");
            }
            break;
            
        case JumpState::HOVERING:
            maintain_althold();
            break;
    }
}

void ModeHybrid::advance_to_next_phase(JumpState next_state, const char* message)
{
    _jump_state = next_state;
    _jump_phase_start_time = AP_HAL::millis();
    gcs().send_text(MAV_SEVERITY_INFO, message);
}

void ModeHybrid::set_jump_thrust(float thrust_level)
{
    // Keep attitude level during jump
    attitude_control->set_throttle_out(thrust_level, true, copter.g.throttle_filt);
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0);
}

void ModeHybrid::reset_tap_detection()
{
    _waiting_for_second_tap = false;
    _first_tap_time = 0;
}

void ModeHybrid::maintain_althold()
{
    // Get pilot inputs for position control
    float target_roll = 0, target_pitch = 0;
    
    if (!copter.failsafe.radio && copter.ap.rc_receiver_present) {
        target_roll = copter.channel_roll->get_control_in();
        target_pitch = copter.channel_pitch->get_control_in();
    }
    
    // Get pilot desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(copter.channel_yaw->norm_input_dz());
    
    // Get pilot desired climb rate
    float pilot_climb_rate = get_pilot_desired_climb_rate_cms();
    
    // Send attitude commands
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(
        target_roll, target_pitch, target_yaw_rate);
    
    // Update altitude controller
    pos_control->set_alt_target_from_climb_rate_ff(pilot_climb_rate, G_Dt, false);
    pos_control->update_z_controller();
}

// ===== SAFETY FUNCTIONS =====

bool ModeHybrid::is_safe_to_jump()
{
    return check_battery_level() && 
           check_vehicle_attitude() && 
           check_ground_contact() && 
           check_system_health();
}

bool ModeHybrid::is_safe_to_switch_to_flight()
{
    return check_battery_level() && 
           check_system_health() && 
           !copter.failsafe.radio;
}

bool ModeHybrid::check_battery_level()
{
    const AP_BattMonitor &battery = copter.battery;
    
    if (!battery.healthy()) {
        return false;
    }
    
    float battery_voltage = battery.voltage();
    return battery_voltage > MIN_BATTERY_VOLTAGE && !battery.has_failsafed();
}

bool ModeHybrid::check_vehicle_attitude()
{
    // Check if vehicle is relatively level
    float roll_deg = degrees(copter.ahrs.roll);
    float pitch_deg = degrees(copter.ahrs.pitch);
    
    return (fabsf(roll_deg) < MAX_JUMP_ANGLE_DEG && 
            fabsf(pitch_deg) < MAX_JUMP_ANGLE_DEG);
}

bool ModeHybrid::check_ground_contact()
{
    // Make sure we've been in flight mode for a minimum time
    uint32_t time_since_mode_switch = AP_HAL::millis() - _mode_switch_time;
    return time_since_mode_switch > MIN_GROUND_TIME_MS;
}

bool ModeHybrid::check_system_health()
{
    return copter.motors->armed() && 
           !copter.failsafe.battery && 
           !copter.failsafe.radio &&
           !copter.failsafe.gcs &&
           copter.ahrs.healthy();
}

float ModeHybrid::get_pilot_desired_climb_rate_cms()
{
    if (copter.failsafe.radio || !copter.ap.rc_receiver_present) {
        return 0.0f;
    }
    
    float throttle_control = copter.channel_throttle->get_control_in();
    
    // Check deadzone
    if (fabsf(throttle_control) < copter.g.throttle_deadzone) {
        return 0.0f;
    }
    
    // Scale throttle to climb rate
    float climb_rate = throttle_control * PILOT_VELZ_MAX / 1000.0f;
    
    return constrain_float(climb_rate, -PILOT_VELZ_MAX, PILOT_VELZ_MAX);
}

void ModeHybrid::update_leds()
{
    // Update notification LEDs based on current state
    if (_is_flight_mode) {
        if (_jump_state != JumpState::NOT_JUMPING) {
            // During jump sequence
            AP_Notify::flags.autopilot_mode = true;
            AP_Notify::flags.armed = true;
        } else {
            // Normal flight mode
            AP_Notify::flags.autopilot_mode = true;
            AP_Notify::flags.armed = copter.motors->armed();
        }
    } else {
        // Ground mode
        AP_Notify::flags.autopilot_mode = false;
        AP_Notify::flags.armed = copter.motors->armed();
    }
}

void ModeHybrid::abort_jump()
{
    _jump_state = JumpState::NOT_JUMPING;
    reset_tap_detection();
    pos_control->set_alt_target_to_current_alt();
    gcs().send_text(MAV_SEVERITY_WARNING, "Jump sequence aborted!");
}

AP_Motors_Hybrid* ModeHybrid::get_hybrid_motors()
{
    if (copter.motors == nullptr) {
        return nullptr;
    }
    
    AP_Motors_Hybrid* hybrid_motors = dynamic_cast<AP_Motors_Hybrid*>(copter.motors);
    return hybrid_motors;
}

#endif // MODE_HYBRID_ENABLED