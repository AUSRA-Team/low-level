#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
#include <geometry_msgs/msg/twist.h>

#define LED_PIN 13
#define MOTOR_PIN 5     // Example motor control pin (PWM)
#define ENCODER_PIN 34  // Example encoder pin

// ROS variables
rcl_publisher_t joint_pub;
rcl_subscription_t cmd_sub;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_timer_t timer;

// Messages
sensor_msgs__msg__JointState joint_msg;
geometry_msgs__msg__Twist cmd_msg;

// Motor control variables
float motor_target_velocity = 0.0;
float motor_position = 0.0;
unsigned long last_update = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ======== Subscriber callback ========
void cmd_vel_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * cmd = (const geometry_msgs__msg__Twist *)msgin;
  motor_target_velocity = cmd->linear.x;  // we only use linear.x
}

// ======== Timer callback for publishing ========
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;

  // Simulated motor position update
  unsigned long now = millis();
  float dt = (now - last_update) / 1000.0;
  last_update = now;

  motor_position += motor_target_velocity * dt;

  // Fill JointState message
  joint_msg.position.data[0] = motor_position;
  joint_msg.velocity.data[0] = motor_target_velocity;
  joint_msg.header.stamp.sec = now / 1000;
  joint_msg.header.stamp.nanosec = (now % 1000) * 1000000;

  RCSOFTCHECK(rcl_publish(&joint_pub, &joint_msg, NULL));
}

// ======== Setup ========
void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_motor_node", "", &support));

  // Publisher: JointState
  RCCHECK(rclc_publisher_init_default(
    &joint_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_states"));

  // Subscriber: cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &cmd_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // Timer (publish every 100 ms)
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Executor (2 handles: 1 timer + 1 subscriber)
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_vel_callback, ON_NEW_DATA));

  // Prepare JointState message arrays
  joint_msg.name.size = 1;
  joint_msg.name.capacity = 1;
  joint_msg.name.data = (rosidl_runtime_c__String*)malloc(sizeof(rosidl_runtime_c__String));
  joint_msg.name.data[0].data = strdup("motor_joint");
  joint_msg.name.data[0].size = strlen("motor_joint");
  joint_msg.name.data[0].capacity = strlen("motor_joint") + 1;

  joint_msg.position.size = 1;
  joint_msg.position.capacity = 1;
  joint_msg.position.data = (double*)malloc(sizeof(double));
  joint_msg.velocity.size = 1;
  joint_msg.velocity.capacity = 1;
  joint_msg.velocity.data = (double*)malloc(sizeof(double));

  last_update = millis();
}

// ======== Loop ========
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
}
