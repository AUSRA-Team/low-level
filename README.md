# Low-Level Control for 3-Wheel Omnidirectional Robot

## Project Description

This repository contains all the low-level control code for the AUSRA team's 3-wheel omnidirectional mobile robot. The system implements PID controllers for three DC motors with encoders, enabling precise motor control and velocity feedback.Using synhcronized ramp input for preventing the drift happens of the Robot before reaching the target velocity.

## System Architecture

**Robot Configuration:**
- 3 DC Motors with Encoders
- Omnidirectional wheel setup (120° arrangement)
- ESP32-S3 microcontroller
- Micro-ROS agent for communication

## Project Structure

### 1. **onlyide** - PID Tuning Module
Live tuning environment for PID controller gains calibration.

**Purpose:**
- Real-time tuning of PID gains (Kp, Ki, Kd) for each motor independently
- Step input response testing and analysis

**Features:**
- Motor-by-motor gain adjustment
- Live feedback monitoring on Serial Plotter
- Step input command generation

### 2. **Tuned_with_communication/CPP_node_template** - Production Controller
Integrated controller with Micro-ROS communication capabilities.

**Purpose:**
- Optimized PID controllers with pre-tuned gains from calibration phase
- Micro-ROS agent integration for ESP32-S3
- Real-time motor command reception and velocity control

**Features:**
- Communication protocol between Micro-ROS agent and ESP32-S3
- Parallel control of all three motors
- Encoder-based velocity feedback
- Synchronized motor coordination for omnidirectional motion (using Ramp input for PID function)

## Workflow

1. **Tuning Phase** → Use `onlyide` to find optimal PID gains for each motor
2. **Integration Phase** → Transfer tuned gains to `Tuned_with_communication/CPP_node_template`
3. **Deployment** → Run on ESP32-S3 with Micro-ROS communication enabled

## Technology Stack

- **Language:** C++
- **Microcontroller:** ESP32-S3
- **Communication:** Micro-ROS
- **Control Strategy:** PID (Proportional-Integral-Derivative)
- **Feedback:** Rotary Encoders

## Getting Started

### Prerequisites
- ESP32-S3 development board
- Arduino IDE
- Micro-ROS libraries and agent setup
- Three DC motors with rotary encoders

### Setup Instructions

1. Clone the repository:
   ```bash
   git clone https://github.com/AUSRA-Team/low-level.git
2. For live Tuning:
   ```bash
   SA 100   #send to all motor speed 100 RPM
   P1 20    #set P gain for motor 1 to 20
3. Setup for Micro-ros agent:
  1) Link Micro-ros repo (https://github.com/micro-ROS/micro_ros_setup/tree/humble)
      ```bash
      mkdir -p ~/microros_ws/src
      cd ~/microros_ws/src
      git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git
   2) Install Dependencies & build tools:
      ```bash
      sudo apt update && rosdep update
      rosdep install --from-paths src --ignore-src -y
      colcon build
      source install/local_setup.bash
   3) Download and Build the Agent:
      ```bash
      ros2 run micro_ros_setup create_agent_ws.sh
      ros2 run micro_ros_setup build_agent.sh
      source install/local_setup.bash
## System Ready to Run

1. Start Micro-ros Agent for Serial connection:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
2. viewing the topics between ros and esp
   ```bash
   ros2 topic list
Topics: 
 
   1- /joint_group_velocity_controller/commands # Array that carries the target speeds come from the omnidriver.
   
   2- /joint states   # Multi array which is the esp publish on it the velocities and positions come from the encoders back to the omnidriver to calculate the odom.

## Wifi_Teleop

### Core Execution Commands

1. **Start WiFi Agent:**
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```

2. **Start Serial Agent** (Alternative):
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
   ```

3. **Start Kinematics Node** (Optional: needed for Teleop):
   ```bash
   python3 omni_controller.py
   ```

4. **Start Keyboard Control:**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

5. **Monitor Topic:**
   ```bash
   ros2 topic echo /joint_group_velocity_controller/commands
   ```

### Mandatory Network Setup

1. **Identify Agent IP:**
   Run the following command to find your laptop's WiFi IP address:
   ```bash
   ip a
   ```

2. **Update Firmware Constants:**
   Update `AGENT_IP`, `SSID`, and `WIFI_PASS` in the ESP32 code.

3. **Match Ports:**
   Ensure the port in the terminal command matches the `AGENT_PORT` in the code (default: `8888`).

4. **Re-flash Requirement:**
   Any change to IP or WiFi credentials requires a new upload to the ESP32.