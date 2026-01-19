#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <WiFi.h>

// Use standard ROS messages (no custom message generation needed)
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

#include "Motor.h"

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

#define PPR 935.0  // Pulses per revolution

// Define Network Credentials
char ssid[] = "Abdelrhman tarek";
char password[] = "mtmos12345";

// Define Agent IP
char agent_ip[] = "10.143.62.126";
size_t agent_port = 8888;

// ========== MOTOR OBJECTS ==========
Motor motors[3];

// ========== MICRO-ROS OBJECTS ==========
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Messages
std_msgs__msg__Float32MultiArray motor_state_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

// ========== TIMING ==========
#define CONTROL_RATE_MS 20     // 50 Hz motor control
#define PUBLISH_RATE_MS 50     // 20 Hz publishing
unsigned long last_control_time = 0;
unsigned long last_publish_time = 0;

// ========== LED for status ==========
#define LED_PIN 2
bool led_state = false;

// ========== ERROR HANDLING ==========
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
    while(1) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    }
}

// ========== ISR HANDLERS ==========
void IRAM_ATTR motor0_isr() { motors[0].encoderISR(); }
void IRAM_ATTR motor1_isr() { motors[1].encoderISR(); }
void IRAM_ATTR motor2_isr() { motors[2].encoderISR(); }

// ========== CMD_VEL CALLBACK ==========
void cmdVelCallback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    
    // High-level node calculates and sends motor speeds directly
    // We just map them from the Twist message to motor setpoints
    // Using linear.x, linear.y, linear.z for motor 0, 1, 2 RPM values
    
    float motor0_rpm = msg->linear.x;   // Motor 0 speed in RPM
    float motor1_rpm = msg->linear.y;   // Motor 1 speed in RPM
    float motor2_rpm = msg->linear.z;   // Motor 2 speed in RPM
    
    // Set motor speeds directly
    motors[0].set_setpoint(motor0_rpm);
    motors[1].set_setpoint(motor1_rpm);
    motors[2].set_setpoint(motor2_rpm);
    
    // Blink LED to show command received
    digitalWrite(LED_PIN, HIGH);
}

// ========== TIMER CALLBACK (for publishing) ==========
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        publishMotorStates();
    }
}

// ========== PUBLISH MOTOR STATES ==========
void publishMotorStates() {
    // Populate motor state message array
    // Index 0: motor1_pos, 1: motor1_speed, 2: motor2_pos, 3: motor2_speed, 4: motor3_pos, 5: motor3_speed
    motor_state_msg.data.data[0] = motors[0].get_position();
    motor_state_msg.data.data[1] = motors[0].get_rpm();
    
    motor_state_msg.data.data[2] = motors[1].get_position();
    motor_state_msg.data.data[3] = motors[1].get_rpm();
    
    motor_state_msg.data.data[4] = motors[2].get_position();
    motor_state_msg.data.data[5] = motors[2].get_rpm();
    
    // Publish
    RCSOFTCHECK(rcl_publish(&publisher, &motor_state_msg, NULL));
    
    // Toggle LED briefly
    led_state = !led_state;
    if (led_state) digitalWrite(LED_PIN, LOW);
}

// ========== SETUP ==========
void setup() {
    // Initialize serial for debugging
    Serial.begin(115200);
    delay(2000);

    Serial.println("--------------------------------");
    Serial.println("APP STARTED. Attempting WiFi connection...");
    Serial.print("Target SSID: ");
    Serial.println(ssid);
    Serial.print("Target Agent IP: ");
    Serial.println(agent_ip);
    Serial.println("--------------------------------");

    set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
    
    Serial.println("WiFi Connected! Connecting to micro-ROS Agent...");

    WiFi.setSleep(false);
    // LED setup
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    
    // Wait for serial (optional, for debugging)
    delay(2000);
    Serial.println("Starting micro-ROS Motor Controller...");
    
    // ===== Initialize Motors =====
    Serial.println("Initializing motors...");
    motors[0].init(MOTOR0_ENC_A, MOTOR0_ENC_B, MOTOR0_PWM_A, MOTOR0_PWM_B, PPR);
    motors[1].init(MOTOR1_ENC_A, MOTOR1_ENC_B, MOTOR1_PWM_A, MOTOR1_PWM_B, PPR);
    motors[2].init(MOTOR2_ENC_A, MOTOR2_ENC_B, MOTOR2_PWM_A, MOTOR2_PWM_B, PPR);
    
    // Set PID parameters (TUNE THESE)
    motors[0].setControlParams(10.0, 0.5, 0.0);
    motors[1].setControlParams(10.0, 0.5, 0.0);
    motors[2].setControlParams(10.0, 0.5, 0.0);
    
    // Attach encoder interrupts
    attachInterrupt(digitalPinToInterrupt(MOTOR0_ENC_A), motor0_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR1_ENC_A), motor1_isr, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR2_ENC_A), motor2_isr, RISING);
    
    Serial.println("✓ Motors initialized");
    
    // ===== Setup micro-ROS =====
    Serial.println("Connecting to micro-ROS agent...");
    
    delay(2000);
    
    allocator = rcl_get_default_allocator();
    
    // Create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    // Create node
    RCCHECK(rclc_node_init_default(&node, "motor_controller_node", "", &support));

    //Implement BEST EFFORT QoS (UDP-style)    
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    // Create publisher
    RCCHECK(rclc_publisher_init(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "motor_states",
        &qos_profile));
    
    // Initialize message array
    motor_state_msg.data.capacity = 6;
    motor_state_msg.data.size = 6;
    motor_state_msg.data.data = (float*) malloc(motor_state_msg.data.capacity * sizeof(float));
    
    // Initialize to zero
    for (size_t i = 0; i < motor_state_msg.data.capacity; i++) {
        motor_state_msg.data.data[i] = 0.0;
    }
    
    // Create subscriber
    RCCHECK(rclc_subscription_init(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
        ,&qos_profile));
    
    // Create timer (for publishing at fixed rate)
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(PUBLISH_RATE_MS),
        timer_callback));
    
    // Create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd_vel_msg, &cmdVelCallback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    
    Serial.println("✓ micro-ROS initialized");
    Serial.println("✓ Publishing to: /motor_states");
    Serial.println("✓ Subscribed to: /cmd_vel");
    Serial.println("\nSystem ready!");
    
    digitalWrite(LED_PIN, LOW);
}

// ========== MAIN LOOP ==========
void loop() {
    unsigned long current_time = millis();
    
    // ===== Motor Control Update (50 Hz) =====
    if (current_time - last_control_time >= CONTROL_RATE_MS) {
        motors[0].update(motors[0].get_setpoint(), CONTROL_RATE_MS);
        motors[1].update(motors[1].get_setpoint(), CONTROL_RATE_MS);
        motors[2].update(motors[2].get_setpoint(), CONTROL_RATE_MS);
        
        last_control_time = current_time;
    }
    
    // ===== micro-ROS spin (handle callbacks) =====
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
    
    // Small delay to prevent watchdog issues
    delay(1);
}