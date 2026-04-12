#ifndef CONFIG_H
#define CONFIG_H
// Adjust this value: higher = more aggressive, lower = smoother
#define MAX_ACCEL 50.0  // RPM/s^2

// Physical Constants
const float TOTAL_CPR = 1870.0; 
const float LPF_ALPHA = 0.15;
const int SAMPLE_MS = 50;

// Motor 1 - Camera
const int M1_ENC_A = 17; const int M1_ENC_B = 18;
const int M1_IN_A  = 4;  const int M1_IN_B  = 5;

// Motor 2 - Charger
const int M2_ENC_A = 35;  const int M2_ENC_B = 21;
const int M2_IN_A  = 8;  const int M2_IN_B  = 9;

// Motor 3 - Switch
const int M3_ENC_A = 2; const int M3_ENC_B = 1;
const int M3_IN_A  = 6;  const int M3_IN_B  = 7;

#endif