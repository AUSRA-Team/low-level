#include "PIDController.h"

PIDController::PIDController(float p, float i, float d) 
    : _kp(p), _ki(i), _kd(d), _integral(0), _lastError(0) {}

void PIDController::reset() {
    _integral = 0;
    _lastError = 0;
}

void PIDController::setP(float p) { _kp = p; }
void PIDController::setI(float i) { _ki = i; }
void PIDController::setD(float d) { _kd = d; }

float PIDController::compute(float target, float current, float dt) {
    if (dt <= 0) return 0;
    if (target == 0 && abs(current) < 0.5) {
        reset();
        return 0;
    }

    float error = target - current;
    _integral = constrain(_integral + (error * dt), -150, 150);
    float P = _kp * error;
    float I = _ki * _integral;
    float D = _kd * (error - _lastError) / dt;

    _lastError = error;
    return P + I + D;
}