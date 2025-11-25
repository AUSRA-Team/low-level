#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <CytronMotorDriver.h>

class Motor {
  private:
    float kp=1, ki=1, kd=0;
    float prev_error = 0;
    float integral = 0;
    float rpm_filtered = 0;   // filtered RPM
    float pos = 0;            // position in radians
    float ppr;

    float control_signal = 0;
    long last_count = 0;
    unsigned long last_time = 0;
    
    float target_rpm = 0;
    float rpm = 0;
    float max_pwm = 255;
    float integral_limit = 100

    CytronMD motor;

  public:
    int encoder_count = 0;
    int encA_pin, encB_pin, pwmA_pin, pwmB_pin;

    Motor();  // default constructor

    void init(int encA, int encB, int pwmA, int pwmB, float ppr_in);
    void update(float target_rpm, unsigned long sample_ms);
    void setControlParams(float Kp, float Ki, float Kd);
    
    void set_setpoint(float rpm) { target_rpm = rpm; }
    // getters
    float get_setpoint() { return target_rpm; }
    float get_rpm() { return rpm; }
    float get_position() { return pos; }

    // ISR handler
    void encoderISR();

  private:
    void setMotor(int dir, float pwm);
    
};

#endif
