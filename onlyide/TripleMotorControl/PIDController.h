#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <Arduino.h>

class PIDController {
public:
    PIDController(float p, float i, float d);
    void reset();
    float compute(float target, float current, float dt);

    // Only the declarations here
    void setP(float p);
    void setI(float i);
    void setD(float d);

private:
    float _kp, _ki, _kd;
    float _integral, _lastError;
};

#endif