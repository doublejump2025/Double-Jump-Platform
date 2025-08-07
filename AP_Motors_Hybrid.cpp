#include "AP_Motors_Hybrid.h"
#include <AP_HAL/AP_HAL.h>

// Use numeric values instead of ENABLED to avoid dependency issues
#ifndef MODE_HYBRID_ENABLED
#define MODE_HYBRID_ENABLED 0
#endif

#if MODE_HYBRID_ENABLED

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Motors_Hybrid::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),
    AP_GROUPEND
};

AP_Motors_Hybrid::AP_Motors_Hybrid(uint16_t speed_hz) :
    AP_MotorsMulticopter(speed_hz),
    _flight_mode(false),
    _throttle_ground(0.0f),
    _steering_ground(0.0f)
{
    for (uint8_t i = 0; i < 8; i++) {
        _actuator_out[i] = 0.0f;
        motor_enabled[i] = false;
        _test_order[i] = i + 1;  // Simple test order
    }
}

void AP_Motors_Hybrid::setup_flight_motors()
{
    // Motor 1: Front Right (CW)
    _flight_factor[0] = {1.0f, 1.0f, 1.0f, 1.0f};
    motor_enabled[0] = true;

    // Motor 2: Rear Left (CW)
    _flight_factor[1] = {1.0f, -1.0f, 1.0f, 1.0f};
    motor_enabled[1] = true;

    // Motor 3: Front Left (CCW)
    _flight_factor[2] = {-1.0f, 1.0f, -1.0f, 1.0f};
    motor_enabled[2] = true;

    // Motor 4: Rear Right (CCW)
    _flight_factor[3] = {-1.0f, -1.0f, -1.0f, 1.0f};
    motor_enabled[3] = true;
}

void AP_Motors_Hybrid::setup_ground_motors()
{
    // Enable ground motors 5-8
    motor_enabled[4] = true;
    motor_enabled[5] = true;
    motor_enabled[6] = true;
    motor_enabled[7] = true;
}

void AP_Motors_Hybrid::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // Call parent init first
    AP_MotorsMulticopter::init(frame_class, frame_type);
    
    // Setup our motors
    setup_flight_motors();
    setup_ground_motors();
    
    // Set initialization as successful
    set_initialised_ok(true);
}

void AP_Motors_Hybrid::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // Don't change if armed
    if (armed()) {
        return;
    }
    
    // Reinitialize
    init(frame_class, frame_type);
}

void AP_Motors_Hybrid::set_throttle_and_steering(float throttle, float steering)
{
    _throttle_ground = constrain_float(throttle, -1.0f, 1.0f);
    _steering_ground = constrain_float(steering, -1.0f, 1.0f);
}

void AP_Motors_Hybrid::calculate_flight_output()
{
    float roll_thrust = get_roll() * 0.5f;
    float pitch_thrust = get_pitch() * 0.5f;
    float yaw_thrust = get_yaw() * 0.5f;
    float throttle_thrust = get_throttle();

    for (uint8_t i = 0; i < 4; i++) {
        _actuator_out[i] = (_flight_factor[i].throttle * throttle_thrust) +
                          (_flight_factor[i].roll * roll_thrust) +
                          (_flight_factor[i].pitch * pitch_thrust) +
                          (_flight_factor[i].yaw * yaw_thrust);
        _actuator_out[i] = constrain_float(_actuator_out[i], 0.0f, 1.0f);
    }
}

void AP_Motors_Hybrid::calculate_ground_output()
{
    // Simple differential steering
    float left_thrust = _throttle_ground - _steering_ground;
    float right_thrust = _throttle_ground + _steering_ground;
    
    // Constrain
    left_thrust = constrain_float(left_thrust, -1.0f, 1.0f);
    right_thrust = constrain_float(right_thrust, -1.0f, 1.0f);
    
    // Convert to 0-1 range
    float left_motor = (left_thrust + 1.0f) * 0.5f;
    float right_motor = (right_thrust + 1.0f) * 0.5f;

    // Apply to ground motors
    _actuator_out[4] = right_motor;  // Front Right Ground
    _actuator_out[5] = left_motor;   // Rear Left Ground
    _actuator_out[6] = left_motor;   // Front Left Ground
    _actuator_out[7] = right_motor;  // Rear Right Ground
}

void AP_Motors_Hybrid::stop_flight_motors()
{
    for (uint8_t i = 0; i < 4; i++) {
        _actuator_out[i] = 0.0f;
    }
}

void AP_Motors_Hybrid::stop_ground_motors()
{
    for (uint8_t i = 4; i < 8; i++) {
        _actuator_out[i] = 0.0f;
    }
}

float AP_Motors_Hybrid::constrain_motor_output(float output) const
{
    return constrain_float(output, 0.0f, 1.0f);
}

void AP_Motors_Hybrid::output_to_motors()
{
    // Choose mode and calculate outputs
    if (_flight_mode) {
        calculate_flight_output();
        stop_ground_motors();
    } else {
        calculate_ground_output();
        stop_flight_motors();
    }

    // Send PWM to motors
    for (uint8_t i = 0; i < 8; i++) {
        if (motor_enabled[i]) {
            int16_t pwm_min = get_pwm_output_min();
            int16_t pwm_max = get_pwm_output_max();
            int16_t pwm_output = pwm_min + (_actuator_out[i] * (pwm_max - pwm_min));
            rc_write(i, pwm_output);
        }
    }
}

void AP_Motors_Hybrid::output_armed_stabilizing()
{
    update_throttle_filter();
    output_to_motors();
}

void AP_Motors_Hybrid::output_min()
{
    for (uint8_t i = 0; i < 8; i++) {
        if (motor_enabled[i]) {
            rc_write(i, get_pwm_output_min());
        }
    }
}

void AP_Motors_Hybrid::update_throttle_filter()
{
    AP_MotorsMulticopter::update_throttle_filter();
}

void AP_Motors_Hybrid::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // Simple: motor_seq 1-8 maps to motors 0-7
    if (motor_seq >= 1 && motor_seq <= 8) {
        uint8_t motor_index = motor_seq - 1;
        if (motor_enabled[motor_index]) {
            rc_write(motor_index, pwm);
        }
    }
}

uint32_t AP_Motors_Hybrid::get_motor_mask()
{
    uint32_t mask = 0;
    for (uint8_t i = 0; i < 8; i++) {
        if (motor_enabled[i]) {
            mask |= (1U << i);
        }
    }
    return mask;
}

void AP_Motors_Hybrid::init_outputs()
{
    // Call parent class init first
    AP_MotorsMatrix::init_outputs();
    
    // Initialize ground motors (5-8)
    add_motor_raw(4, -1.0f, 1.0f, -1.0f);   // Front left ground motor
    add_motor_raw(5, 1.0f, 1.0f, -1.0f);    // Front right ground motor
    add_motor_raw(6, -1.0f, -1.0f, -1.0f);  // Back left ground motor
    add_motor_raw(7, 1.0f, -1.0f, -1.0f);   // Back right ground motor
    
    _ground_mode = false;
    _ground_steering = 0;
    _ground_throttle = 0;
}

void AP_Motors_Hybrid::set_ground_mode(bool ground_mode)
{
    _ground_mode = ground_mode;
}

void AP_Motors_Hybrid::set_ground_steering(float steering)
{
    _ground_steering = steering;
}

void AP_Motors_Hybrid::set_ground_throttle(float throttle)
{
    _ground_throttle = throttle;
}

void AP_Motors_Hybrid::output_to_ground_motors()
{
    // Stop flight motors
    for (uint8_t i = 0; i < 4; i++) {
        _thrust_rpyt_out[i] = 0;
    }
    
    // Calculate ground motor outputs (tank drive style)
    float left_output = _ground_throttle - _ground_steering;
    float right_output = _ground_throttle + _ground_steering;
    
    // Apply to ground motors (indices 4-7)
    _thrust_rpyt_out[4] = left_output;
    _thrust_rpyt_out[5] = right_output;
    _thrust_rpyt_out[6] = left_output;
    _thrust_rpyt_out[7] = right_output;
    
    // Convert thrust to PWM and output
    for (uint8_t i = 0; i < 8; i++) {
        rc_write(i, output_to_pwm(_thrust_rpyt_out[i]));
    }
}

void AP_Motors_Hybrid::output_to_flight_motors()
{
    // Stop ground motors
    for (uint8_t i = 4; i < 8; i++) {
        _thrust_rpyt_out[i] = 0;
    }
    
    // Let parent class handle flight motor output
    AP_MotorsMatrix::output_to_motors();
}

#endif // MODE_HYBRID_ENABLED