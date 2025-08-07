#pragma once

#include "AP_Motors_Class.h"
#include "AP_MotorsMatrix.h"

// Use numeric values instead of ENABLED to avoid dependency issues
#ifndef MODE_HYBRID_ENABLED
#define MODE_HYBRID_ENABLED 0
#endif

#if MODE_HYBRID_ENABLED

class AP_Motors_Hybrid : public AP_MotorsMatrix {
public:
    AP_Motors_Hybrid(uint16_t loop_rate) :
        AP_MotorsMatrix(loop_rate) {
    }

    // Override any necessary AP_MotorsMatrix virtual methods here
    void init_outputs() override;
    void output_to_motors() override;
    
    // Hybrid specific methods
    void set_ground_mode(bool ground_mode);
    void set_ground_steering(float steering);
    void set_ground_throttle(float throttle);
    
protected:
    bool _ground_mode;
    float _ground_steering;
    float _ground_throttle;
    
    // Additional methods for ground motor control
    void output_to_ground_motors();
    void output_to_flight_motors();
};

#endif // MODE_HYBRID_ENABLED