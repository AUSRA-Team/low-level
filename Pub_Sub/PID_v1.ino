#include <Arduino.h>
#include <math.h>
// Pin Configuration
static const int EN_A = 12;
static const int EN_B = 14;
static const int M_A = 26;
static const int M_B = 27;
static const int PWM = 25;

// Global Variables
volatile long encoder_count = 0;
float kp=1, ki=1, kd=0;
int direction=0;
float prev_error = 0;
float integral = 0;
float rpm = 0;
float pos = 0;
float v_filt = 0;
float v_prev = 0;
int PPR = 1980;

// Setup All Pins
void setup_pins(){
  pinMode(EN_A,INPUT);
  pinMode(EN_B,INPUT);
  pinMode(M_A,OUTPUT);
  pinMode(M_B,OUTPUT);
  pinMode(PWM,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(EN_A), encoderISR_A, RISING);
}

// Control Motor
void PID(float set_point,float PPR, unsigned long sample_time){
  static long last_count=0;
  static long last_time=0;
  unsigned long current_time = millis();
  static float control_signal = 0;

//PID Control
  if ((current_time-last_time)>=sample_time){
    long current_count = encoder_count;
    long delta_count = current_count - last_count;
    float delta_time = (current_time - last_time)/1000.0;
    rpm = (delta_count/PPR) * (60.0/ delta_time);
    pos = (delta_count/PPR) * 2 * PI;

    v_filt = 0.854*v_filt + 0.0728*rpm + 0.00728 * v_prev;
    v_prev = rpm;
    float error = set_point - v_prev;
    integral += error * delta_time;
    float derivative = (error-prev_error)/ delta_time;
    control_signal = kp * error + ki * integral + kd * derivative ;

    // Update Memory
    prev_error = error;   
    last_count = current_count;
    last_time = current_time;
  

    // Control Drection
    if (control_signal > 0)
      direction = 1;
    else
      direction = -1;
    // PWM and anti-windup
    int pwm_output = (int) fabs(control_signal);
    if (pwm_output > 255)
      pwm_output = 255;
  
    set_motor(direction,pwm_output,PWM,M_A,M_B);
    
    // Printing
    Serial.print("RPM: "); Serial.print(rpm);
    Serial.print(" | Error: "); Serial.print(error);
    Serial.print(" | Output: "); Serial.println(control_signal);

  }
}


// Encoder Interrupt
void IRAM_ATTR encoderISR_A(){
  bool b = digitalRead(EN_B);
  if (b) 
    encoder_count++;
  else
    encoder_count--;
}

// Set Motor
void set_motor(int dir, float pwm_output, int pwm, int in1, int in2){
   // Motor speed
  analogWrite(pwm,pwm_output);

  if(dir == 1){ 
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}
