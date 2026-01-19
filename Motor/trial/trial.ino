#include "Motor.h"
#include <WiFi.h>
// ========== PIN CONFIGURATION ==========
// Motor 0 pins
#define MOTOR0_ENC_A 2
#define MOTOR0_ENC_B 1
#define MOTOR0_PWM_A 6
#define MOTOR0_PWM_B 7

// Motor 1 pins
#define MOTOR1_ENC_A 35
#define MOTOR1_ENC_B 21
#define MOTOR1_PWM_A 8
#define MOTOR1_PWM_B 9

// Motor 2 pins
#define MOTOR2_ENC_A 17
#define MOTOR2_ENC_B 18
#define MOTOR2_PWM_A 4
#define MOTOR2_PWM_B 5

// Define Network Credentials
char ssid[] = "Omar's IPhone";
char password[] = "12345679";

// Define Agent IP
IPAddress agent_ip(172, 20, 10, 5);
size_t agent_port = 8888;

// Motor specs
#define PPR 935.0  // Pulses per revolution

// ========== GLOBAL OBJECTS ==========
Motor motors[3];

// Test state machine
enum TestState {
    TEST_INIT,
    TEST_ENCODER,
    TEST_DIRECTION,
    TEST_SPEED_STEP,
    TEST_PID_TUNING,
    TEST_COMPLETE
};

TestState currentTest = TEST_INIT;
unsigned long testStartTime = 0;
int testPhase = 0;

// ========== ISR HANDLERS ==========
void IRAM_ATTR motor0_isr() { motors[0].encoderISR(); }
void IRAM_ATTR motor1_isr() { motors[1].encoderISR(); }
void IRAM_ATTR motor2_isr() { motors[2].encoderISR(); }

// ========== SETUP ==========
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    
    Serial.println("\n\n========================================");
    Serial.println("    MOTOR CLASS TEST - ESP32");
    Serial.println("========================================\n");
    
    // Initialize motors
    Serial.println("Initializing motors...");
    motors[0].init(MOTOR0_ENC_A, MOTOR0_ENC_B, MOTOR0_PWM_A, MOTOR0_PWM_B, PPR);
    motors[1].init(MOTOR1_ENC_A, MOTOR1_ENC_B, MOTOR1_PWM_A, MOTOR1_PWM_B, PPR);
    motors[2].init(MOTOR2_ENC_A, MOTOR2_ENC_B, MOTOR2_PWM_A, MOTOR2_PWM_B, PPR);
    
    // Set PID parameters
    motors[0].setControlParams(10, 0, 0);
    motors[1].setControlParams(10, 0, 0);
    motors[2].setControlParams(10, 0, 0);
    
    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(MOTOR0_ENC_A), motor0_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR1_ENC_A), motor1_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR2_ENC_A), motor2_isr, RISING);
    
    Serial.println("✓ Motors initialized");
    Serial.println("✓ Interrupts attached");
    Serial.println("\nStarting tests in 3 seconds...\n");
    delay(3000);
    
    testStartTime = millis();
    currentTest = TEST_ENCODER;
}

// ========== MAIN LOOP ==========
void loop() {
    static unsigned long lastPrint = 0;
    static unsigned long lastUpdate = 0;
    
    // Update motor control at 50Hz
    if (millis() - lastUpdate >= 20) {
        motors[0].update(motors[0].get_setpoint(), 20);
        motors[1].update(motors[1].get_setpoint(), 20);
        motors[2].update(motors[2].get_setpoint(), 20);
        lastUpdate = millis();
    }
    
    // Run current test
    runTest();
    
    // Print status at 5Hz
    if (millis() - lastPrint >= 200) {
        printStatus();
        lastPrint = millis();
    }
}

// ========== TEST EXECUTION ==========
void runTest() {
    unsigned long elapsed = millis() - testStartTime;
    
    switch(currentTest) {
        
        case TEST_ENCODER:
            // Test 1: Encoder reading (manual wheel spin)
            if (testPhase == 0) {
                Serial.println("\n========================================");
                Serial.println("TEST 1: ENCODER READING");
                Serial.println("========================================");
                Serial.println("ACTION: Manually spin each motor wheel");
                Serial.println("EXPECTED: Encoder counts change");
                Serial.println("DURATION: 10 seconds\n");
                testPhase = 1;
            }
            
            if (elapsed > 10000) {
                // Check if encoders moved
                bool allMoved = true;
                for (int i = 0; i < 3; i++) {
                    if (abs(motors[i].encoder_count) < 10) {
                        Serial.printf("⚠ Motor %d encoder seems inactive!\n", i);
                        allMoved = false;
                    }
                }
                
                if (allMoved) {
                    Serial.println("\n✓ TEST 1 PASSED: All encoders working");
                } else {
                    Serial.println("\n✗ TEST 1 FAILED: Check encoder wiring");
                }
                
                // Move to next test
                currentTest = TEST_DIRECTION;
                testStartTime = millis();
                testPhase = 0;
                delay(2000);
            }
            break;
            
        case TEST_DIRECTION:
            // Test 2: Direction control
            if (testPhase == 0) {
                Serial.println("\n========================================");
                Serial.println("TEST 2: DIRECTION CONTROL");
                Serial.println("========================================");
                Serial.println("Testing forward and reverse...\n");
                testPhase = 1;
            }
            
            if (elapsed < 3000) {
                // Forward
                motors[0].set_setpoint(30);
                motors[1].set_setpoint(30);
                motors[2].set_setpoint(30);
                if (testPhase == 1) {
                    Serial.println("Phase: FORWARD (30 RPM)");
                    testPhase = 2;
                }
            } else if (elapsed < 4000) {
                // Stop
                motors[0].set_setpoint(0);
                motors[1].set_setpoint(0);
                motors[2].set_setpoint(0);
            } else if (elapsed < 7000) {
                // Reverse
                motors[0].set_setpoint(-30);
                motors[1].set_setpoint(-30);
                motors[2].set_setpoint(-30);
                if (testPhase == 2) {
                    Serial.println("Phase: REVERSE (-30 RPM)");
                    testPhase = 3;
                }
            } else if (elapsed < 8000) {
                // Stop
                motors[0].set_setpoint(0);
                motors[1].set_setpoint(0);
                motors[2].set_setpoint(0);
            } else {
                Serial.println("\n✓ TEST 2 COMPLETE: Check if motors reversed correctly");
                currentTest = TEST_SPEED_STEP;
                testStartTime = millis();
                testPhase = 0;
                delay(2000);
            }
            break;
            
        case TEST_SPEED_STEP:
            // Test 3: Step response
            if (testPhase == 0) {
                Serial.println("\n========================================");
                Serial.println("TEST 3: STEP RESPONSE");
                Serial.println("========================================");
                Serial.println("Testing speed changes (Motor 0 only)...\n");
                testPhase = 1;
            }
            
            if (elapsed < 4000) {
                motors[0].set_setpoint(50);
                if (testPhase == 1) {
                    Serial.println("Setpoint: 50 RPM");
                    testPhase = 2;
                }
            } else if (elapsed < 8000) {
                motors[0].set_setpoint(100);
                if (testPhase == 2) {
                    Serial.println("Setpoint: 100 RPM");
                    testPhase = 3;
                }
            } else if (elapsed < 12000) {
                motors[0].set_setpoint(150);
                if (testPhase == 3) {
                    Serial.println("Setpoint: 150 RPM");
                    testPhase = 4;
                }
            } else if (elapsed < 13000) {
                motors[0].set_setpoint(0);
            } else {
                Serial.println("\n✓ TEST 3 COMPLETE: Review step response");
                currentTest = TEST_PID_TUNING;
                testStartTime = millis();
                testPhase = 0;
                delay(2000);
            }
            break;
            
        case TEST_PID_TUNING:
            // Test 4: PID performance check
            if (testPhase == 0) {
                Serial.println("\n========================================");
                Serial.println("TEST 4: PID PERFORMANCE");
                Serial.println("========================================");
                Serial.println("Running constant speed test (Motor 0)...");
                Serial.println("TARGET: 80 RPM for 10 seconds\n");
                motors[0].set_setpoint(80);
                testPhase = 1;
            }
            
            if (elapsed > 10000) {
                motors[0].set_setpoint(0);
                
                // Calculate average error (last 5 seconds)
                float avgRPM = motors[0].get_rpm();
                float error = abs(80 - avgRPM);
                
                Serial.printf("\n✓ TEST 4 COMPLETE\n");
                Serial.printf("  Target: 80.0 RPM\n");
                Serial.printf("  Actual: %.2f RPM\n", avgRPM);
                Serial.printf("  Error: %.2f RPM (%.1f%%)\n", error, (error/80)*100);
                
                if (error < 5) {
                    Serial.println("  Result: ✓ EXCELLENT - PID well tuned");
                } else if (error < 10) {
                    Serial.println("  Result: ✓ GOOD - Minor tuning may help");
                } else if (error < 20) {
                    Serial.println("  Result: ⚠ FAIR - Consider PID tuning");
                } else {
                    Serial.println("  Result: ✗ POOR - PID needs tuning");
                }
                
                currentTest = TEST_COMPLETE;
                testStartTime = millis();
                testPhase = 0;
                delay(2000);
            }
            break;
            
        case TEST_COMPLETE:
            if (testPhase == 0) {
                Serial.println("\n========================================");
                Serial.println("ALL TESTS COMPLETE!");
                Serial.println("========================================");
                Serial.println("\nTest Summary:");
                Serial.println("1. Encoder Reading     - Check console output");
                Serial.println("2. Direction Control   - Did motors reverse?");
                Serial.println("3. Step Response       - Smooth transitions?");
                Serial.println("4. PID Performance     - Error acceptable?");
                Serial.println("\nEntering continuous monitoring mode...");
                Serial.println("Set speeds via Serial Monitor:");
                Serial.println("  Format: M<motor> <rpm>");
                Serial.println("  Example: M0 60  (sets motor 0 to 60 RPM)");
                Serial.println("           M1 -30 (sets motor 1 to -30 RPM)");
                Serial.println("           S      (stops all motors)\n");
                testPhase = 1;
            }
            
            // Handle serial commands
            handleSerialCommands();
            break;
    }
}

// ========== STATUS PRINTING ==========
void printStatus() {
    if (currentTest == TEST_COMPLETE && testPhase == 0) return;
    
    Serial.println("----------------------------------------");
    Serial.printf("Time: %lu ms | Test: ", millis());
    
    switch(currentTest) {
        case TEST_ENCODER: Serial.print("ENCODER"); break;
        case TEST_DIRECTION: Serial.print("DIRECTION"); break;
        case TEST_SPEED_STEP: Serial.print("STEP RESPONSE"); break;
        case TEST_PID_TUNING: Serial.print("PID TUNING"); break;
        case TEST_COMPLETE: Serial.print("MONITORING"); break;
        default: Serial.print("UNKNOWN"); break;
    }
    Serial.println();
    
    for (int i = 0; i < 3; i++) {
        Serial.printf("Motor %d | ", i);
        Serial.printf("Enc: %6d | ", motors[i].encoder_count);
        Serial.printf("Pos: %7.2f rad | ", motors[i].get_position());
        Serial.printf("Vel: %6.2f RPM | ", motors[i].get_rpm());
        Serial.printf("Target: %6.2f RPM\n", motors[i].get_setpoint());
    }
}

// ========== SERIAL COMMAND HANDLER ==========
void handleSerialCommands() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        
        if (cmd.startsWith("M") || cmd.startsWith("m")) {
            // Parse: M0 60 or M1 -30
            int motorNum = cmd.substring(1, 2).toInt();
            int rpm = cmd.substring(3).toInt();
            
            if (motorNum >= 0 && motorNum < 3) {
                motors[motorNum].set_setpoint(rpm);
                Serial.printf("✓ Motor %d setpoint: %d RPM\n", motorNum, rpm);
            } else {
                Serial.println("✗ Invalid motor number (0-2)");
            }
        } else if (cmd == "S" || cmd == "s") {
            // Stop all
            for (int i = 0; i < 3; i++) {
                motors[i].set_setpoint(0);
            }
            Serial.println("✓ All motors stopped");
        } else if (cmd == "P" || cmd == "p") {
            // Print detailed info
            printDetailedInfo();
        } else {
            Serial.println("Commands: M<motor> <rpm> | S (stop) | P (info)");
        }
    }
}

// ========== DETAILED INFO ==========
void printDetailedInfo() {
    Serial.println("\n========== DETAILED MOTOR INFO ==========");
    for (int i = 0; i < 3; i++) {
        Serial.printf("\nMotor %d:\n", i);
        Serial.printf("  Encoder Count: %d\n", motors[i].encoder_count);
        Serial.printf("  Position: %.4f rad (%.2f deg)\n", 
                     motors[i].get_position(), 
                     motors[i].get_position() * 180.0 / PI);
        Serial.printf("  Velocity: %.2f RPM (%.2f rad/s)\n", 
                     motors[i].get_rpm(),
                     motors[i].get_rpm() * 2.0 * PI / 60.0);
        Serial.printf("  Setpoint: %.2f RPM\n", motors[i].get_setpoint());
        Serial.printf("  Error: %.2f RPM\n", 
                     motors[i].get_setpoint() - motors[i].get_rpm());
    }
    Serial.println("=========================================\n");
}
