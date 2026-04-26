#include "Config.h"
#include "Motor.h"

// 1. GLOBAL INSTANTIATIONS (Crucial: This fixes the "not declared" errors)
Motor cameraMotor(M1_IN_A, M1_IN_B, 26.0, 12.0, 0.0);
Motor switchMotor(M2_IN_A, M2_IN_B, 26.0, 12.0, 0.0);
Motor chargerMotor(M3_IN_A, M3_IN_B, 26.0, 12.0, 0.0);

float t1 = 0, t2 = 0, t3 = 0;
volatile long ticks1 = 0, ticks2 = 0, ticks3 = 0;
unsigned long lastTime = 0;

// Interrupt Service Routines
void IRAM_ATTR isr1() { (digitalRead(M1_ENC_A) == digitalRead(M1_ENC_B)) ? ticks1++ : ticks1--; }
void IRAM_ATTR isr2() { (digitalRead(M2_ENC_A) == digitalRead(M2_ENC_B)) ? ticks2++ : ticks2--; }
void IRAM_ATTR isr3() { (digitalRead(M3_ENC_A) == digitalRead(M3_ENC_B)) ? ticks3++ : ticks3--; }

void setup() {
    Serial.begin(115200);

    pinMode(M1_ENC_A, INPUT_PULLUP); pinMode(M1_ENC_B, INPUT_PULLUP);
    pinMode(M2_ENC_A, INPUT_PULLUP); pinMode(M2_ENC_B, INPUT_PULLUP);
    pinMode(M3_ENC_A, INPUT_PULLUP); pinMode(M3_ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), isr1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), isr2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M3_ENC_A), isr3, CHANGE);

    cameraMotor.setup();
    switchMotor.setup();
    chargerMotor.setup();

    lastTime = micros();
    Serial.println("System Ready. Commands: [P/I/D/S][1/2/3/A] [Value]");
}

void loop() {
    // --- MULTI-TARGET SERIAL PARSER ---
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toUpperCase(); 

        int spacePos = input.indexOf(' ');
        if (spacePos != -1) {
            String targets = input.substring(0, spacePos); 
            float value = input.substring(spacePos + 1).toFloat();
            char cmdType = targets[0]; 
            
            bool m1 = (targets.indexOf('1') != -1 || targets.indexOf('A') != -1);
            bool m2 = (targets.indexOf('2') != -1 || targets.indexOf('A') != -1);
            bool m3 = (targets.indexOf('3') != -1 || targets.indexOf('A') != -1);

            if (cmdType == 'P') {
                if (m1) cameraMotor.setP(value);
                if (m2) switchMotor.setP(value);
                if (m3) chargerMotor.setP(value);
            } 
            else if (cmdType == 'I') {
                if (m1) cameraMotor.setI(value);
                if (m2) switchMotor.setI(value);
                if (m3) chargerMotor.setI(value);
            } 
            else if (cmdType == 'D') {
                if (m1) cameraMotor.setD(value);
                if (m2) switchMotor.setD(value);
                if (m3) chargerMotor.setD(value);
            } 
            else if (cmdType == 'S') {
                if (m1) t1 = value;
                if (m2) t2 = value;
                if (m3) t3 = value;
            }
        }
    }

    unsigned long currentTime = micros();
    float dt = (float)(currentTime - lastTime) / 1000000.0;

    if (dt >= (SAMPLE_MS / 1000.0)) {
        noInterrupts();
        long c1 = ticks1; long c2 = ticks2; long c3 = ticks3;
        interrupts();

        cameraMotor.update(t1, c1, dt);
        switchMotor.update(t2, c2, dt);
        chargerMotor.update(t3, c3, dt);

        // Plotter Output
        Serial.print(t1); Serial.print(","); Serial.print(cameraMotor.getRPM()); Serial.print(",");
        Serial.print(t2); Serial.print(","); Serial.print(switchMotor.getRPM()); Serial.print(",");
        Serial.print(t3); Serial.print(","); Serial.println(chargerMotor.getRPM());

        lastTime = currentTime;
    }
}