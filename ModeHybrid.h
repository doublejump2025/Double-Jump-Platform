#pragma once
#include "mode.h"

#if MODE_HYBRID_ENABLED == ENABLED

class ModeHybrid : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    
    // mode number must be implemented
    Number mode_number() const override { return Number::HYBRID; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return true; }
    bool allows_arming(uint8_t method) const override { return true; }
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    bool in_guided_mode() const override { return false; }

    // Add missing virtual functions that might be pure virtual in Mode
    const char* name() const override { return "HYBRID"; }
    const char* name4() const override { return "HYBR"; }

    // Hybrid specific functions
    bool is_flight_mode() const { return _is_flight_mode; }
    bool is_jumping() const { return _jump_state != JumpState::NOT_JUMPING; }

protected:

private:

    // Jump sequence states
    enum class JumpState {
        NOT_JUMPING,
        FIRST_JUMP_PREP,
        FIRST_JUMP_THRUST,
        PAUSE_BETWEEN_JUMPS,
        SECOND_JUMP_PREP,
        SECOND_JUMP_THRUST,
        TRANSITION_TO_ALTHOLD,
        HOVERING
    };

    // State variables
    bool _is_flight_mode;
    JumpState _jump_state;
    uint32_t _mode_switch_time;
    uint32_t _jump_phase_start_time;
    
    // Double-tap detection
    bool _ch6_prev_high;
    uint32_t _first_tap_time;
    bool _waiting_for_second_tap;
    
    // Ground movement
    float _ground_throttle;
    float _ground_steering;
    
    // Constants
    static const uint32_t FIRST_JUMP_PREP_MS = 300;
    static const uint32_t FIRST_JUMP_TIME_MS = 200;
    static const uint32_t PAUSE_BETWEEN_JUMPS_MS = 100;
    static const uint32_t SECOND_JUMP_PREP_MS = 200;
    static const uint32_t SECOND_JUMP_TIME_MS = 400;
    static const uint32_t ALTHOLD_TRANSITION_MS = 200;
    
    static constexpr float FIRST_JUMP_POWER = 0.6f;
    static constexpr float SECOND_JUMP_POWER = 0.9f;
    static constexpr float HOVER_THROTTLE = 0.5f;
    
    static const uint32_t DOUBLE_TAP_TIME_MS = 500;
    static const uint32_t MODE_SWITCH_DEBOUNCE_MS = 500;
    
     // Private methods
    void handle_mode_switching();
    void handle_ground_mode();
    void handle_flight_mode();
    void handle_jump_sequence();
    void detect_double_tap(bool ch6_high, uint32_t now);
    void start_auto_double_jump();
    void set_jump_thrust(float thrust_level);
    void update_leds();
    AP_Motors_Hybrid* get_hybrid_motors();
    
    // Helper functions
    RC_Channel* get_channel_safe(uint8_t channel_num);
    void execute_auto_jump_sequence(uint32_t now);
    void advance_to_next_phase(JumpState next_state, const char* message);
    void maintain_althold();
    void reset_tap_detection();
    void abort_jump();
    void calculate_flight_movement();
    void calculate_ground_movement();
    float get_ground_throttle();
    float get_ground_steering();
    
    // Safety functions
    bool is_safe_to_jump();
    bool is_safe_to_switch_to_flight();
    bool check_battery_level();
    bool check_vehicle_attitude();
    bool check_ground_contact();
    bool check_system_health();
    float get_pilot_desired_climb_rate_cms();
};

#endif // MODE_HYBRID_ENABLED