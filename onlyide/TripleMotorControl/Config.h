#ifndef CONFIG_H
#define CONFIG_H

// Physical Constants
const float TOTAL_CPR = 1870.0; 
const float LPF_ALPHA = 0.15;
const int SAMPLE_MS = 50;

// Motor 1 - Camera
const int M1_ENC_A = 17; const int M1_ENC_B = 18;
const int M1_IN_A  = 4;  const int M1_IN_B  = 5;

// Motor 2 - Switch
const int M2_ENC_A = 2;  const int M2_ENC_B = 1;
const int M2_IN_A  = 6;  const int M2_IN_B  = 7;

// Motor 3 - Charger
const int M3_ENC_A = 35; const int M3_ENC_B = 21;
const int M3_IN_A  = 8;  const int M3_IN_B  = 9;

#endif