#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <string.h>
#include <rmw_microros/rmw_microros.h>
// Standard ROS 2 Messages
#include <std_msgs/msg/float64_multi_array.h>
#include <sensor_msgs/msg/joint_state.h>
#include "Config.h"
#include "Motor.h"

// --- Robot Namespace (received from Jetson before micro-ROS starts) ---
static char g_ns[32] = "";

// --- micro-ROS Objects ---
rcl_subscription_t subscriber;
std_msgs__msg__Float64MultiArray msg_sub;
rcl_publisher_t publisher;
sensor_msgs__msg__JointState msg_pub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// --- FreeRTOS Handles ---
TaskHandle_t MicroROSTaskHandle = NULL;
TaskHandle_t PIDTaskHandle = NULL;
SemaphoreHandle_t xSharedMemoryMutex = NULL;

// --- Thread-Safe Shared Memory Structure ---
struct SharedRobotData {
    // Targets moving from Core 0 (ROS 2) to Core 1 (PID Engine)
    float cmd_t1 = 0.0;
    float cmd_t2 = 0.0;
    float cmd_t3 = 0.0;

    // Telemetry moving from Core 1 (PID Engine) to Core 0 (ROS 2)
    double joint_positions[3] = {0.0, 0.0, 0.0};
    double joint_velocities[3] = {0.0, 0.0, 0.0};
} robotData;

// Motor Instances
Motor cameraMotor(M1_IN_A, M1_IN_B, 10.0, 10.0, 0.01, 30);
Motor switchMotor(M2_IN_A, M2_IN_B, 10.0, 10.0, 0.01, 30);
Motor chargerMotor(M3_IN_A, M3_IN_B, 10.0, 10.0, 0.01, 30);

volatile long ticks1 = 0, ticks2 = 0, ticks3 = 0;
const float RADS_TO_RPM = 60.0 / (2.0 * PI);
const float RPM_TO_RADS = (2.0 * PI) / 60.0;

// --- Forward Declarations of FreeRTOS Tasks ---
void microROSTask(void * parameter);
void pidControlTask(void * parameter);

// --- Error Handling with Auto-Reset ---
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
    delay(2000);
    ESP.restart();
}

// --- Interrupt Service Routines ---
void IRAM_ATTR isr1() { (digitalRead(M1_ENC_A) == digitalRead(M1_ENC_B)) ? ticks1++ : ticks1--; }
void IRAM_ATTR isr2() { (digitalRead(M2_ENC_A) == digitalRead(M2_ENC_B)) ? ticks2++ : ticks2--; }
void IRAM_ATTR isr3() { (digitalRead(M3_ENC_A) == digitalRead(M3_ENC_B)) ? ticks3++ : ticks3--; }

// --- micro-ROS Subscription Callback ---
void subscription_callback(const void * msgin) {
    const std_msgs__msg__Float64MultiArray * msg = (const std_msgs__msg__Float64MultiArray *)msgin;
    if (msg->data.size >= 3) {
        // Safely deposit velocity targets into shared memory using Mutex lock
        if (xSemaphoreTake(xSharedMemoryMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            robotData.cmd_t1 = msg->data.data[0] * RADS_TO_RPM;
            robotData.cmd_t2 = msg->data.data[1] * RADS_TO_RPM;
            robotData.cmd_t3 = msg->data.data[2] * RADS_TO_RPM;
            xSemaphoreGive(xSharedMemoryMutex);
        }
    }
}

void setup() {
    Serial.begin(115200);
    // Wait explicitly for the "NAMESPACE:" prefix, ignoring XRCE-DDS garbage
    String ns = "";
    while (true) {
      if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        
        // Only accept strings that start with our prefix
        if (line.startsWith("ns:")) {
            ns = line.substring(3); // Extract everything after "ns"
            ns.trim();
            break; // Break the loop and start Micro-ROS
        }
      }
    }
    
    strncpy(g_ns, ns.c_str(), sizeof(g_ns) - 1);
    g_ns[sizeof(g_ns) - 1] = '\0';
    
    set_microros_transports();

    // Initialize Encoder Pins
    pinMode(M1_ENC_A, INPUT_PULLUP); pinMode(M1_ENC_B, INPUT_PULLUP);
    pinMode(M2_ENC_A, INPUT_PULLUP); pinMode(M2_ENC_B, INPUT_PULLUP);
    pinMode(M3_ENC_A, INPUT_PULLUP); pinMode(M3_ENC_B, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), isr1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), isr2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(M3_ENC_A), isr3, CHANGE);

    // Initialize Hardware Motors
    cameraMotor.setup(); switchMotor.setup(); chargerMotor.setup();
    delay(2000);

    allocator = rcl_get_default_allocator();

    // Initialize micro-ROS Support Structure & Node
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_base_controller", g_ns, &support));
    
    // Initial clock synchronization hook
    rmw_uros_sync_session(1000); 

    // Sensor data profile (Best-effort delivery configuration)
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;

    // 1. Setup Velocity Subscription
    RCCHECK(rclc_subscription_init(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "joint_group_velocity_controller/commands",
        &qos_profile));

    static double data_buffer[3];
    msg_sub.data.capacity = 3;
    msg_sub.data.data = data_buffer;

    // 2. Setup Joint State Publisher
    RCCHECK(rclc_publisher_init(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states",
        &qos_profile));

    // Configure Joint Names Mapping Sequence
    static char * joint_names_ptrs[3] = {(char*)"ausra_joint_1", (char*)"ausra_joint_2", (char*)"ausra_joint_3"};
    static rosidl_runtime_c__String name_sequence[3];
    msg_pub.name.capacity = 3;
    msg_pub.name.size = 3;
    msg_pub.name.data = name_sequence;

    for(int i = 0; i < 3; i++) {
        msg_pub.name.data[i].data = joint_names_ptrs[i];
        msg_pub.name.data[i].size = strlen(joint_names_ptrs[i]);
        msg_pub.name.data[i].capacity = strlen(joint_names_ptrs[i]) + 1;
    }

    // Configure Data Capacity Buffers for Publisher Arrays
    static double vel_buffer[3] = {0,0,0};
    msg_pub.velocity.capacity = 3; msg_pub.velocity.size = 3; msg_pub.velocity.data = vel_buffer;

    static double pos_buffer[3] = {0,0,0};
    msg_pub.position.capacity = 3; msg_pub.position.size = 3; msg_pub.position.data = pos_buffer;

    msg_pub.effort.capacity = 0;

    // 3. Setup Executor with exactly 1 handle (Only tracking the motion velocity command sub)
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));

    // Create FreeRTOS Shared Memory Mutex
    xSharedMemoryMutex = xSemaphoreCreateMutex();

    // Spawn Multithreaded Core Ecosystem Tasks
    xTaskCreatePinnedToCore(
        microROSTask,        // Function name
        "microROSTask",      // Task string description
        10000,               // Stack size allocations (bytes)
        NULL,                // Function parameter pointer
        1,                   // Task Priority level 
        &MicroROSTaskHandle, // Handle reference object
        0                    // Pin task operation to Core 0 (Handles Networking/Comms)
    );

    xTaskCreatePinnedToCore(
        pidControlTask,      // Function name
        "pidControlTask",    // Task string description
        5000,                // Stack size allocation
        NULL,                // Function parameter pointer
        5,                   // Task Priority level (High to guarantee tight loops)
        &PIDTaskHandle,      // Handle reference object
        1                    // Pin task operation to Core 1 (Isolated High-Freq Math Engine)
    );
}

// Main Arduino loop is blocked; execution is fully driven by FreeRTOS scheduler tasks
void loop() {
    vTaskDelay(portMAX_DELAY); 
}

// ============================================================================
// TASK 1: micro-ROS Thread Execution Block (Core 0)
// ============================================================================
void microROSTask(void * parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPublishPeriod = pdMS_TO_TICKS(20); // 50 Hz reporting rate
    unsigned long lastPingTime = millis();
    int64_t last_published_time_ns = 0;
    int missed_pings = 0;
    for(;;) {
        // Check for incoming velocity target data arrays from ROS 2
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0));

        // Periodic Agent Diagnostics & Continuous Clock Alignment (Every 1 second)
        if (millis() - lastPingTime >= 1000) {
            lastPingTime = millis();
            if (rmw_uros_ping_agent(50, 2) == RMW_RET_OK) {
                // Continuous background synchronization to defeat ESP32 oscillator drift
                rmw_uros_sync_session(10);
                missed_pings = 0; // Reset missed ping counter on successful response 
            } else {
                missed_pings++;
                if(missed_pings >=3) {
                    // If we miss 5 consecutive pings, we assume the connection is lost and reset
                    ESP.restart();
                }
            }
        }

        // Safely extract engine states computed on Core 1 using the Mutex
        if (xSemaphoreTake(xSharedMemoryMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            msg_pub.velocity.data[0] = robotData.joint_velocities[0];
            msg_pub.velocity.data[1] = robotData.joint_velocities[1];
            msg_pub.velocity.data[2] = robotData.joint_velocities[2];

            msg_pub.position.data[0] = robotData.joint_positions[0];
            msg_pub.position.data[1] = robotData.joint_positions[1];
            msg_pub.position.data[2] = robotData.joint_positions[2];
            xSemaphoreGive(xSharedMemoryMutex);
        }

        // Apply Time Stamp with Monotonic Guard to block negative time-jumps across sync actions
        int64_t time_ns = rmw_uros_epoch_nanos();
        if (time_ns <= last_published_time_ns) {
            time_ns = last_published_time_ns + 1000000LL; 
        }
        last_published_time_ns = time_ns;

        msg_pub.header.stamp.sec = time_ns / 1000000000LL;
        msg_pub.header.stamp.nanosec = time_ns % 1000000000LL;

        // Ship joint states down the transport line to Jetson Master Agent
        rcl_publish(&publisher, &msg_pub, NULL);

        // Precise deterministic delay pacing
        vTaskDelayUntil(&xLastWakeTime, xPublishPeriod);
    }
}

// ============================================================================
// TASK 2: High-Frequency PID Engine Loop (Core 1)
// ============================================================================
void pidControlTask(void * parameter) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPIDPeriod = pdMS_TO_TICKS(SAMPLE_MS); 
    float dt = (float)SAMPLE_MS / 1000.0;

    float local_cmd_t1 = 0, local_cmd_t2 = 0, local_cmd_t3 = 0;
    float t1 = 0, t2 = 0, t3 = 0; // Internal speed setpoints tracking acceleration ramps

    for(;;) {
        // Fetch raw target values dropped inside memory registers by Core 0
        if (xSemaphoreTake(xSharedMemoryMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            local_cmd_t1 = robotData.cmd_t1;
            local_cmd_t2 = robotData.cmd_t2;
            local_cmd_t3 = robotData.cmd_t3;
            xSemaphoreGive(xSharedMemoryMutex);
        }

        // Target Acceleration Ramping Calculations
        float diff1 = local_cmd_t1 - t1;
        float diff2 = local_cmd_t2 - t2;
        float diff3 = local_cmd_t3 - t3;
        float max_diff = fmaxf(fabs(diff1), fmaxf(fabs(diff2), fabs(diff3)));

        if (max_diff > 0.01) {
            float max_change = MAX_ACCEL * dt;
            float scale = 1.0;
            if (max_diff > max_change) {
                scale = max_change / max_diff;
            }
            t1 += diff1 * scale;
            t2 += diff2 * scale;
            t3 += diff3 * scale;
        } else {
            t1 = local_cmd_t1; t2 = local_cmd_t2; t3 = local_cmd_t3;
        }

        // Snapshot encoder ticks atomically safely
        noInterrupts();
        long c1 = ticks1; long c2 = ticks2; long c3 = ticks3;
        interrupts();

        // Run Class PID controller calculations
        cameraMotor.update(t1, c1, dt);
        switchMotor.update(t2, c2, dt);
        chargerMotor.update(t3, c3, dt);

        // Store generated data frames back to shared memory struct using the Mutex
        if (xSemaphoreTake(xSharedMemoryMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            robotData.joint_velocities[0] = cameraMotor.getRPM() * RPM_TO_RADS;
            robotData.joint_velocities[1] = switchMotor.getRPM() * RPM_TO_RADS;
            robotData.joint_velocities[2] = chargerMotor.getRPM() * RPM_TO_RADS;

            double tick_to_rad = (2.0 * PI) / TOTAL_CPR;
            robotData.joint_positions[0] = (double)c1 * tick_to_rad;
            robotData.joint_positions[1] = (double)c2 * tick_to_rad;
            robotData.joint_positions[2] = (double)c3 * tick_to_rad;
            xSemaphoreGive(xSharedMemoryMutex);
        }

        // Strict deterministic loop block until next execution window
        vTaskDelayUntil(&xLastWakeTime, xPIDPeriod);
    }
}