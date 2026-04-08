#ifndef MOTOR_H
#define MOTOR_H

#include "PIDController.h"
#include "Config.h"

class Motor {
public:
    Motor(int pinA, int pinB, float p, float i, float d);
    void setup();
    void update(float targetRPM, long currentTicks, float dt);
    float getRPM();

private:
    PIDController _pid;
    int _pinA, _pinB;
    long _lastTicks;
    float _filteredRPM;
    void drive(float output, float target);
};

#endif
