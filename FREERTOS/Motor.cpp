#include "Motor.h"

Motor::Motor(int pinA, int pinB, float p, float i, float d, int minPWM) 
    : _pid(p, i, d), _pinA(pinA), _pinB(pinB), _minPWM(minPWM), _lastTicks(0), _filteredRPM(0) {}

void Motor::setup() {
    pinMode(_pinA, OUTPUT);
    pinMode(_pinB, OUTPUT);
    analogWrite(_pinA, 0);
    analogWrite(_pinB, 0);
}

void Motor::update(float targetRPM, long currentTicks, float dt) {
    // 1. Calculate Velocity
    long deltaTicks = currentTicks - _lastTicks;
    float rawRPM = ((float)deltaTicks / TOTAL_CPR) / (dt / 60.0);
    
    // 2. Apply Low Pass Filter
    _filteredRPM = (LPF_ALPHA * rawRPM) + (1.0 - LPF_ALPHA) * _filteredRPM;

    // 3. PID Math
    float output = _pid.compute(targetRPM, _filteredRPM, dt);

    // 4. Drive Hardware
    drive(output, targetRPM);

    _lastTicks = currentTicks;
}

float Motor::getRPM() {
    return _filteredRPM;
}

// --- Added for Online Parameter Tuning ---
void Motor::updateGains(float p, float i, float d) {
    // NOTE: If your custom "PIDController.h" class uses a different function name 
    // to change parameters (like setTunings() or updateGains()), rename it here.
    _pid.setGains(p, i, d); 
}

void Motor::drive(float output, float target) {
    int raw_pwm = abs((int)output);
    int pwm = 0;

    // If ROS is requesting movement, scale the PID output to kick in right at the threshold
    if (target != 0 && raw_pwm > 2) {
        pwm = map(raw_pwm, 0, 255, _minPWM, 255);
    } else {
        pwm = raw_pwm;
    }

    pwm = constrain(pwm, 0, 255);
    
    // Use the per-motor _minPWM threshold instead of a hardcoded value
    if (target == 0 || pwm < _minPWM) {
        analogWrite(_pinA, 0);
        analogWrite(_pinB, 0);
    } else if (output > 0) {
        analogWrite(_pinA, pwm);
        analogWrite(_pinB, 0);
    } else {
        analogWrite(_pinA, 0);
        analogWrite(_pinB, pwm);
    }
}