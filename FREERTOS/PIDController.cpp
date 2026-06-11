#include "PIDController.h"

PIDController::PIDController(float p, float i, float d) 
    : _kp(p), _ki(i), _kd(d), _integral(0), _lastError(0) {}

void PIDController::reset() {
    _integral = 0;
    _lastError = 0;
}
// --- Add this function to the bottom of your PIDController.cpp file ---
void PIDController::setGains(float p, float i, float d) {
    _kp = p;
    _ki = i;
    _kd = d;
    
    // Optional but highly recommended: 
    // Reset the accumulated integral error when tuning live to prevent giant power spikes
    _integral = 0.0; 
    _lastError = 0.0;
}

float PIDController::compute(float target, float current, float dt) {
    if (dt <= 0) return 0;
    if (target == 0 && abs(current) < 0.5) {
        reset();
        return 0;
    }

    float error = target - current;
    
    // Proportional
    float P = _kp * error;

    // Integral (with anti-windup clamping)
    _integral = constrain(_integral + (error * dt), -75, 75);
    float I = _ki * _integral;

    // Derivative
    float D = _kd * (error - _lastError) / dt;

    _lastError = error;
    return P + I + D;
}
