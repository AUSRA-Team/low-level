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

// --- micro-ROS Objects ---
rcl_subscription_t subscriber;
std_msgs__msg__Float64MultiArray msg_sub;
rcl_publisher_t publisher;
sensor_msgs__msg__JointState msg_pub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Motor Instances
Motor cameraMotor(M1_IN_A, M1_IN_B, 26.0, 12.0, 0.0);
Motor switchMotor(M2_IN_A, M2_IN_B, 26.0, 12.0, 0.0);
Motor chargerMotor(M3_IN_A, M3_IN_B, 26.0, 12.0, 0.0);

float cmd_t1 = 0, cmd_t2 = 0, cmd_t3 = 0; // Targets from ROS
float t1 = 0, t2 = 0, t3 = 0;             // Ramped setpoints for PID
volatile long ticks1 = 0, ticks2 = 0, ticks3 = 0;
unsigned long lastTime = 0;
unsigned long lastPingTime = 0;

const float RADS_TO_RPM = 60.0 / (2.0 * PI);
const float RPM_TO_RADS = (2.0 * PI) / 60.0;

// --- Error Handling with Auto-Reset ---
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){ 
  delay(2000); 
  ESP.restart(); 
}

// --- Interrupts ---
void IRAM_ATTR isr1() { (digitalRead(M1_ENC_A) == digitalRead(M1_ENC_B)) ? ticks1++ : ticks1--; }
void IRAM_ATTR isr2() { (digitalRead(M2_ENC_A) == digitalRead(M2_ENC_B)) ? ticks2++ : ticks2--; }
void IRAM_ATTR isr3() { (digitalRead(M3_ENC_A) == digitalRead(M3_ENC_B)) ? ticks3++ : ticks3--; }

void subscription_callback(const void * msgin) {
  const std_msgs__msg__Float64MultiArray * msg = (const std_msgs__msg__Float64MultiArray *)msgin;
  if (msg->data.size >= 3) {
    cmd_t1 = msg->data.data[0] * RADS_TO_RPM;
    cmd_t2 = msg->data.data[1] * RADS_TO_RPM;
    cmd_t3 = msg->data.data[2] * RADS_TO_RPM;
  }
}

void setup() {
  set_microros_transports();

  pinMode(M1_ENC_A, INPUT_PULLUP); pinMode(M1_ENC_B, INPUT_PULLUP);
  pinMode(M2_ENC_A, INPUT_PULLUP); pinMode(M2_ENC_B, INPUT_PULLUP);
  pinMode(M3_ENC_A, INPUT_PULLUP); pinMode(M3_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), isr2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(M3_ENC_A), isr3, CHANGE);

  cameraMotor.setup(); switchMotor.setup(); chargerMotor.setup();

  delay(2000);
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_base_controller", "", &support));

  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data; 

  RCCHECK(rclc_subscription_init(
    &subscriber, 
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
    "joint_group_velocity_controller/commands",
    &qos_profile)); 

  static double data_buffer[3];
  msg_sub.data.capacity = 3;
  msg_sub.data.data = data_buffer;

  RCCHECK(rclc_publisher_init(
    &publisher, 
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_states",
    &qos_profile)); 

  static char * joint_names_ptrs[3] = {(char*)"ausrabot_joint_1", (char*)"ausrabot_joint_2", (char*)"ausrabot_joint_3"};
  static rosidl_runtime_c__String name_sequence[3];
  msg_pub.name.capacity = 3;
  msg_pub.name.size = 3;
  msg_pub.name.data = name_sequence;

  for(int i = 0; i < 3; i++) {
    msg_pub.name.data[i].data = joint_names_ptrs[i];
    msg_pub.name.data[i].size = strlen(joint_names_ptrs[i]);
    msg_pub.name.data[i].capacity = strlen(joint_names_ptrs[i]) + 1;
  }

  static double vel_buffer[3] = {0,0,0};
  msg_pub.velocity.capacity = 3; msg_pub.velocity.size = 3; msg_pub.velocity.data = vel_buffer;

  static double pos_buffer[3] = {0,0,0};
  msg_pub.position.capacity = 3; msg_pub.position.size = 3; msg_pub.position.data = pos_buffer;

  msg_pub.effort.capacity = 0; 

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));

  lastTime = micros();
  lastPingTime = micros();
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0)));

  unsigned long currentTime = micros();
  
  if (currentTime - lastPingTime >= 1000000) { 
    lastPingTime = currentTime;
    if (rmw_uros_ping_agent(50, 2) != RMW_RET_OK) {
      error_loop(); 
    }
  }

  float dt = (float)(currentTime - lastTime) / 1000000.0;

  if (dt >= (SAMPLE_MS / 1000.0)) {
    
    // --- SYNCHRONIZED RAMPING LOGIC ---
    float diff1 = cmd_t1 - t1;
    float diff2 = cmd_t2 - t2;
    float diff3 = cmd_t3 - t3;

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
        t1 = cmd_t1; t2 = cmd_t2; t3 = cmd_t3;
    }

    noInterrupts();
    long c1 = ticks1; long c2 = ticks2; long c3 = ticks3;
    interrupts();

    cameraMotor.update(t1, c1, dt);
    switchMotor.update(t2, c2, dt);
    chargerMotor.update(t3, c3, dt);
    
    struct timespec tv;
    clock_gettime(CLOCK_REALTIME, &tv);
    msg_pub.header.stamp.sec = tv.tv_sec;
    msg_pub.header.stamp.nanosec = tv.tv_nsec;

    msg_pub.velocity.data[0] = cameraMotor.getRPM() * RPM_TO_RADS;
    msg_pub.velocity.data[1] = switchMotor.getRPM() * RPM_TO_RADS;
    msg_pub.velocity.data[2] = chargerMotor.getRPM() * RPM_TO_RADS;

    double tick_to_rad = (2.0 * PI) / TOTAL_CPR;
    msg_pub.position.data[0] = (double)c1 * tick_to_rad;
    msg_pub.position.data[1] = (double)c2 * tick_to_rad;
    msg_pub.position.data[2] = (double)c3 * tick_to_rad;

    // Publish the message
    RCSOFTCHECK(rcl_publish(&publisher, &msg_pub, NULL));

    lastTime = currentTime;
  }
}