#include "Motor.h"

Motor::Motor(int pinA, int pinB, float p, float i, float d) 
    : _pid(p, i, d), _pinA(pinA), _pinB(pinB), _lastTicks(0), _filteredRPM(0) {}

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

void Motor::drive(float output, float target) {
    int pwm = constrain(abs((int)output), 0, 255);
    
    if (fabs(target) < 0.5f || pwm < 5) {
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
