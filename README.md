# Low-Level Control for 3-Wheel Omnidirectional Robot

## Project Description

This repository contains all the low-level control code for the AUSRA team's 3-wheel omnidirectional mobile robot. The system implements PID controllers for three DC motors with encoders, enabling precise motor control and velocity feedback.

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
- Individual motor characterization and validation

**Features:**
- Motor-by-motor gain adjustment
- Live feedback monitoring
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
- Synchronized motor coordination for omnidirectional motion

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
- Arduino IDE or PlatformIO
- Micro-ROS libraries and agent setup
- Three DC motors with rotary encoders

### Setup Instructions

1. Clone the repository:
   ```bash
   git clone https://github.com/AUSRA-Team/low-level.git
   cd low-level
2. For live Tuning:
   ```bash
   SA 100   #send to all motor speed 100 RPM
   P1 20    #set P gain for motor 1 to 20
