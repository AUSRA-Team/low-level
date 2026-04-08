#include "Config.h"
#include "Motor.h"

// Instantiate Motors with your tuned gains
Motor cameraMotor(M1_IN_A, M1_IN_B, 26.0, 12.0, 0.0);
Motor switchMotor(M2_IN_A, M2_IN_B, 26.0, 12.0, 0.0);
Motor chargerMotor(M3_IN_A, M3_IN_B, 26.0, 12.0, 0.0);

// Global Targets
float t1 = 0, t2 = 0, t3 = 0;

// Interrupt Variables
volatile long ticks1 = 0, ticks2 = 0, ticks3 = 0;
unsigned long lastTime = 0;

// Interrupt Service Routines
void IRAM_ATTR isr1() { (digitalRead(M1_ENC_A) == digitalRead(M1_ENC_B)) ? ticks1++ : ticks1--; }
void IRAM_ATTR isr2() { (digitalRead(M2_ENC_A) == digitalRead(M2_ENC_B)) ? ticks2++ : ticks2--; }
void IRAM_ATTR isr3() { (digitalRead(M3_ENC_A) == digitalRead(M3_ENC_B)) ? ticks3++ : ticks3--; }

void setup() {
    Serial.begin(115200);

    // Encoder Hardware Setup
    pinMode(M1_ENC_A, INPUT_PULLUP); pinMode(M1_ENC_B, INPUT_PULLUP);
    pinMode(M2_ENC_A, INPUT_PULLUP); pinMode(M2_ENC_B, INPUT_PULLUP);
    pinMode(M3_ENC_A, INPUT_PULLUP); pinMode(M3_ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), isr1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), isr2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M3_ENC_A), isr3, CHANGE);

    // Motor Driver Setup
    cameraMotor.setup();
    switchMotor.setup();
    chargerMotor.setup();

    lastTime = micros();
    Serial.println("System Ready. Commands: '1 100' for Motor 1 only, or '100' for all.");
}

void loop() {
    // --- SELECTIVE SERIAL PARSER ---
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        int spacePos = input.indexOf(' ');
        if (spacePos != -1) {
            // Case: "1 100" (Individual Selection)
            int motorID = input.substring(0, spacePos).toInt();
            float speed = input.substring(spacePos + 1).toFloat();
            
            // Set target for selected, zero for others
            t1 = (motorID == 1) ? speed : 0;
            t2 = (motorID == 2) ? speed : 0;
            t3 = (motorID == 3) ? speed : 0;
        } 
        else if (input.length() > 0) {
            // Case: "150" (All Motors)
            float allSpeed = input.toFloat();
            t1 = t2 = t3 = allSpeed;
        }
    }

    unsigned long currentTime = micros();
    float dt = (float)(currentTime - lastTime) / 1000000.0;

    if (dt >= (SAMPLE_MS / 1000.0)) {
        // Atomic Ticks Capture
        noInterrupts();
        long c1 = ticks1; long c2 = ticks2; long c3 = ticks3;
        interrupts();

        // Update all motor controllers
        cameraMotor.update(t1, c1, dt);
        switchMotor.update(t2, c2, dt);
        chargerMotor.update(t3, c3, dt);

        // Plotter Output: T1, C1, T2, C2, T3, C3
        Serial.print(t1); Serial.print(","); Serial.print(cameraMotor.getRPM()); Serial.print(",");
        Serial.print(t2); Serial.print(","); Serial.print(switchMotor.getRPM()); Serial.print(",");
        Serial.print(t3); Serial.print(","); Serial.println(chargerMotor.getRPM());

        lastTime = currentTime;
    }
}
