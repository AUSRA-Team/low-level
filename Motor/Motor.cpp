#include "Motor.h"

Motor::Motor() : motor(PWM_PWM, 0, 0) {} // dummy pins initially

void Motor::init(int encA, int encB, int pwmA, int pwmB, float ppr_in) {
    encA_pin = encA;
    encB_pin = encB;
    pwmA_pin = pwmA;
    pwmB_pin = pwmB;
    ppr = ppr_in;

    pinMode(encA_pin, INPUT);
    pinMode(encB_pin, INPUT);

    motor = CytronMD(PWM_PWM, pwmA_pin, pwmB_pin); // initialize motor driver

    last_time = millis();
    last_count = 0;
}

void Motor::setControlParams(float Kp, float Ki, float Kd) {
    kp = Kp;
    ki = Ki;
    kd = Kd;
}

void Motor::encoderISR() {
    bool b = digitalRead(encB_pin);
    if (b) encoder_count++;
    else encoder_count--;
}

void Motor::update(float target_rpm_setpoint, unsigned long sample_ms) {
    this->target_rpm = target_rpm_setpoint;  // STORE setpoint
    
    unsigned long current_time = millis();
    if (current_time - last_time >= sample_ms) {
        // Atomic read of encoder
        noInterrupts();
        long delta_count = encoder_count - last_count;
        interrupts();
        
        float delta_time = (current_time - last_time) / 1000.0;
        
        // Calculate RPM and store in member variable
        rpm = (delta_count / ppr) * (60.0 / delta_time);
        
        // Proper low-pass filter
        float alpha = 0.1;  // adjust as needed
        rpm_filtered = alpha * rpm + (1.0 - alpha) * rpm_filtered;
        
        pos = encoder_count * 2.0 * PI / ppr;
        
        // PID
        float error = target_rpm - rpm_filtered;
        integral += error * delta_time;
        
        // Anti-windup
        integral = constrain(integral, -integral_limit, integral_limit);
        
        float derivative = (error - prev_error) / delta_time;
        float control_signal = kp * error + ki * integral + kd * derivative;
        
        // Saturation
        float pwm = constrain(fabs(control_signal), 0, max_pwm);
        
        // Direction
        int direction;
        if (control_signal > 0) direction = 1;
        else if (control_signal < 0) direction = -1;
        else direction = 0;
        
        setMotor(direction, pwm);
        
        prev_error = error;
        last_count = encoder_count;
        last_time = current_time;
    }
}

// Fix scope
float Motor::get_position() { 
    return pos; 
}

void Motor::setMotor(int dir, float pwm) {
    if(dir == 1) motor.setSpeed(pwm);
    else if(dir == -1) motor.setSpeed(-pwm);
    else motor.setSpeed(0);
}
float Motor::get_setpoint() { 
    return target_rpm; }
    
float Motor::get_rpm() { 
    return filtered_rpm; }
