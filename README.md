# Low-Level Control for 3-Wheel Omnidirectional Robot

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Micro-ROS](https://img.shields.io/badge/Micro--ROS-XRCE--DDS-green.svg)](https://micro.ros.org/)
[![Microcontroller](https://img.shields.io/badge/MCU-ESP32--S3-red.svg)](https://www.espressif.com/en/products/socs/esp32-s3)
[![Framework](https://img.shields.io/badge/OS-FreeRTOS-orange.svg)](https://www.freertos.org/)
[![Language](https://img.shields.io/badge/Language-C%2B%2B%20%7C%20Python-lightgrey.svg)](https://isocpp.org/)

This repository contains the complete low-level control firmware, ROS 2 interface nodes, tuning environments, and deployment scripts for the **AUSRA** team's 3-wheel omnidirectional mobile robot platform. 

The system acts as the real-time embedded **"Spine"** of the robot, executing closed-loop PID velocity control, quadrature encoder velocity estimation, synchronized ramp rate acceleration filtering, and Micro-ROS communication over USB-serial or UDP/WiFi.

---

## Electrical System Topology & Real Hardware

### Hierarchical Control Architecture

The platform decouples high-level perception and navigation from time-critical motor control. An **NVIDIA Jetson Orin Nano** serves as the high-level **"Brain"** (handling SLAM, vision, and path planning), communicating via Micro-ROS serial with an **ESP32-S3 Microcontroller** acting as the low-level **"Spine"** (handling inverse kinematics, PID control, and encoder feedback).

![System Electrical Topology](docs/images/system_topology.png)

### Real Robot Assembly Showcase

| Locomotion Base Plate (Exploded View) | Assembled Middle Layer Control Hub |
| :---: | :---: |
| ![Locomotion Base Plate](docs/images/locomotion_base_plate.jpg) | ![Middle Layer Control Hub](docs/images/middle_layer_control_hub.jpg) |
| *Bottom tier housing 3 omni-wheels, JGY-370 motors, and Cytron MDD3A drivers.* | *Middle tier housing ESP32-S3 MCU, signal isolation, and 20V power management.* |

---

## Hardware Specifications & Component Selection

### Components Gallery

| Microcontroller | Motor Driver | DC Gearmotor & Encoder | Omnidirectional Wheel |
| :---: | :---: | :---: | :---: |
| ![ESP32-S3](docs/images/esp32_s3.png) | ![MDD3A Driver](docs/images/mdd3a_motor_driver.png) | ![JGY-370 Motor](docs/images/jgy370_motor.png) | ![Omni Wheel](docs/images/omni_wheel.jpg) |
| **ESP32-S3** Dual-Core | **Cytron MDD3A** 3A MOSFET | **JGY-370** 12V Gearmotor | **58mm** Omnidirectional Wheel |

### Low-Level Microcontroller Selection Matrix

The ESP32-S3 was selected over alternative microcontrollers through a weighted decision matrix evaluating real-time control capability, ROS 2 integration efficiency, and hardware features:

| Evaluation Criteria | Weight | Teensy 4.1 | STM32F411 | ESP32-S3 (Selected) |
| :--- | :---: | :---: | :---: | :---: |
| **Native Micro-ROS Support** | 35% | 30 / 35 | 15 / 35 | **35 / 35** |
| **Dual-Core Task Separation** | 25% | 0 / 25 | 0 / 25 | **25 / 25** |
| **USB-OTG Native Interface** | 20% | 20 / 20 | 15 / 20 | **18 / 20** |
| **Development Ecosystem & Cost** | 20% | 12 / 20 | 18 / 20 | **30 / 20** |
| **Total Evaluation Score** | **100%** | **62 / 100** | **48 / 100** | **108 / 100** |

*Key Driver:* The Xtensa LX7 dual-core architecture allows Core 0 to service the blocking Micro-ROS communication stack while Core 1 executes strict real-time motor control loops without network jitter interference.

### Hardware Pinout Mapping (`Config.h`)

| Motor Channel | Wheel Position | PWM Pin A (`IN_A`) | PWM Pin B (`IN_B`) | Encoder A (`ENC_A`) | Encoder B (`ENC_B`) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Motor 1** | Camera Side (90°) | GPIO 4 | GPIO 5 | GPIO 17 | GPIO 18 |
| **Motor 2** | Charger Side (210°) | GPIO 8 | GPIO 9 | GPIO 35 | GPIO 21 |
| **Motor 3** | Switch Side (330°) | GPIO 6 | GPIO 7 | GPIO 2 | GPIO 1 |

---

## Control Theory & Mathematical Formulation

### 1. Actuator Feedback & Encoder Calibration
The JGY-370 motors include optical quadrature encoders. The firmware implements an event-driven architecture using hardware interrupts tagged with `IRAM_ATTR` (forcing ISR execution directly from internal SRAM for nanosecond latency).

The firmware uses **X2 decoding**, firing interrupts on both edges of Channel A while reading Channel B for direction:
- Halves CPU interrupt load relative to X4 decoding (reducing from ~60,000 to ~30,000 ISR calls/sec per motor at 300 RPM).
- Calibrated resolution: $\mathbf{1997.0\text{ Counts Per Revolution (CPR)}}$ (`TOTAL_CPR = 1997.0`).

### 2. Quantization Filtering (Low-Pass EMA Filter)
Discrete sampling (every 30 ms) produces a single-count staircase quantization error in velocity derivatives. To eliminate derivative spikes and motor humming, an Exponential Moving Average (EMA) filter is applied:

$$\hat{\omega}[k] = \alpha \cdot \omega_{\text{raw}}[k] + (1-\alpha) \cdot \hat{\omega}[k-1]$$

Where $\alpha = 0.45$ (`LPF_ALPHA = 0.45`) balances responsiveness against derivative noise rejection.

### 3. Discrete PID Control & Clamping Anti-Windup
The continuous PID algorithm is discretized using Backward Euler integration with a sampling period of $T_s = 0.03\text{ s}$ ($\approx 33\text{ Hz}$):

$$u[k] = K_p \, e[k] + K_i \cdot \text{clamp}\left(I_{\text{acc}}[k-1] + e[k] \cdot T_s,\ -75,\ +75\right) + K_d \, \frac{e[k] - e[k-1]}{T_s}$$

To prevent **integrator windup** during actuator saturation or wheel obstruction, the integral accumulator state $I_{\text{acc}}$ is hard-bounded to the interval $[-75, +75]$ on every cycle *before* applying $K_i$.

### 4. Dead-Zone Breakaway Compensation
Brushed DC gearmotors exhibit static friction below a minimum duty cycle threshold. To eliminate low-speed deadband, non-zero PID control efforts are remapped above the motor breakaway threshold ($\text{minPWM} = 30$):

$$\text{PWM} = \text{map}\left(|u|,\ 0,\ 255,\ \text{minPWM},\ 255\right)$$

### 5. Synchronized Uniform Ramp Limiting
Applying unconstrained step velocity inputs causes off-axis rotational drift due to uneven surface resistance across wheels. The firmware enforces acceleration slew-rate limiting ($a_{\max} = 50\text{ RPM/s}$):

$$\Delta_{\max} = a_{\max} \cdot \Delta t, \qquad s = \min\left(1,\ \frac{\Delta_{\max}}{\Delta_{\max,\text{req}}}\right)$$

The uniform scale factor $s$ scales all wheel setpoint increments proportionally, preserving the exact directional vector during transient acceleration.

---

## FreeRTOS Dual-Core & Micro-ROS Architecture

### Real-Time Task Partitioning

The production firmware (`FREERTOS/`) executes on FreeRTOS, partitioning workloads across the dual Xtensa LX7 cores to guarantee hard real-time determinism:

![FreeRTOS Dual-Core Architecture](docs/images/rtos_dual_core.png)

- **`microROSTask` (Core 0, Priority 1):** Handles Micro-ROS communication, servicing incoming velocity commands and publishing telemetry at a fixed 50 Hz rate.
- **`pidControlTask` (Core 1, Priority 5):** Executes the strict real-time control loop every 30 ms ($\approx 33\text{ Hz}$).
- Data exchange between tasks is mediated via a mutex-guarded `SharedRobotData` structure, preventing blocking between communication and control loops.

### Micro-ROS Client-Agent Transport

![Micro-ROS Architecture](docs/images/microros_architecture.jpeg)

Micro-ROS enables the ESP32-S3 to participate directly in the ROS 2 graph using Micro-XRCE-DDS with a **<50 KB RAM footprint**. Communication utilizes Best Effort Quality of Service (`rmw_qos_profile_sensor_data`) over USB-CDC serial at 115200 baud.

---

## Empirical Two-Stage PID Tuning Results

Due to evolving chassis weight during hardware bring-up, PID tuning followed a two-stage empirical methodology:

1. **Stage 1 (Free-Air / Unloaded):** Suspended robot testing to tune rise time ($K_p$) and eliminate steady-state error ($K_i$) without chassis inertia.
2. **Stage 2 (On-Ground / Loaded):** Live gain adjustment on the fully assembled 5 kg robot using the `setGains()` Micro-ROS interface to overcome dynamic rolling friction.

### Final Deployed Parameters

$$\mathbf{K_p = 10.0}, \qquad \mathbf{K_i = 10.0}, \qquad \mathbf{K_d = 0.01}$$

- **Integral Accumulator Bounds:** $[-75.0, +75.0]$
- **Breakaway PWM (`minPWM`):** $30$
- **Ramp Acceleration Limit (`MAX_ACCEL`):** $50.0\text{ RPM/s}^2$
- **Low-Pass Filter Alpha (`LPF_ALPHA`):** $0.45$

---

## Project Structure

```
low-level/
├── docs/images/                              # Thesis figures & hardware component images
├── FREERTOS/                                 # Production dual-core FreeRTOS firmware
│   ├── FREERTOS.ino                         # Main dual-core FreeRTOS sketch
│   ├── Config.h                             # Pin assignments & physical parameters
│   ├── Motor.cpp / Motor.h                  # Motor driver logic with dead-zone mapping
│   └── PIDController.cpp / PIDController.h  # LPF-filtered PID controller with anti-windup
├── Tuned_with_Commnunication/                # Serial Micro-ROS Arduino template
│   └── CPP_Node_template/
├── Wifi_Teleop/                             # Wireless UDP Micro-ROS setup
│   └── CPP_Node_template/
├── onlyide/                                  # Interactive PID Gain Calibration firmware
│   └── TripleMotorControl/
├── scripts/
│   └── start_micro_ros_agent.sh             # Automated agent launcher & namespace injector
├── omni_controller.py                        # Inverse kinematics ROS 2 node (/cmd_vel -> motors)
└── README.md                                 # Complete documentation
```

---

## Getting Started & Workflows

### Prerequisites

1. **Host Environment:** Linux OS running **ROS 2 Humble**.
2. **Hardware:** ESP32-S3 board connected via USB (`/dev/ttyACM0`), 3x DC gearmotors with encoders, Cytron MDD3A drivers, and 12V/20V power supply.
3. **Micro-ROS Agent Setup:**
   ```bash
   mkdir -p ~/microros_ws/src
   cd ~/microros_ws/src
   git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git
   cd ~/microros_ws
   sudo apt update && rosdep update
   rosdep install --from-paths src --ignore-src -y
   colcon build && source install/local_setup.bash
   
   ros2 run micro_ros_setup create_agent_ws.sh
   ros2 run micro_ros_setup build_agent.sh
   source install/local_setup.bash
   ```

---

### Workflow 1: Production Deployment (Automated Launch)

To start the serial agent with dynamic hardware-level namespace injection (`ns:<robot_namespace>`):

```bash
chmod +x scripts/start_micro_ros_agent.sh
./scripts/start_micro_ros_agent.sh ausra_1
```

*What the launcher handles:*
- Cleans up stale Micro-ROS processes.
- Launches `micro_ros_agent` at 115200 baud on `/dev/ttyACM0`.
- Transmits `ns:ausra_1` to the ESP32 following DTR reset.
- Verifies XRCE-DDS session establishment.

---

### Workflow 2: Inverse Kinematics & Keyboard Teleoperation

In separate terminals with ROS 2 sourced:

1. **Launch Kinematics Node:**
   ```bash
   python3 omni_controller.py
   ```
2. **Start Keyboard Control:**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
3. **Monitor Telemetry:**
   ```bash
   ros2 topic echo /ausra_1/joint_group_velocity_controller/commands
   ros2 topic echo /ausra_1/joint_states
   ```

---

### Workflow 3: Live Interactive PID Tuning (`onlyide`)

1. Flash `onlyide/TripleMotorControl` to the ESP32-S3.
2. Open Serial Plotter / Monitor at 115200 baud.
3. Send calibration commands:
   ```text
   SA 100   # Command 100 RPM to all 3 motors
   P1 20    # Set Motor 1 Kp = 20
   I1 0.5   # Set Motor 1 Ki = 0.5
   D1 0.1   # Set Motor 1 Kd = 0.1
   ```

---

## ROS 2 Topics Interface

| Topic Name | Message Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Input | Target linear velocity ($v_x, v_y$) and angular rotation ($\omega_z$) |
| `/<namespace>/joint_group_velocity_controller/commands` | `std_msgs/msg/Float64MultiArray` | Internal | Array `[m1_rpm, m2_rpm, m3_rpm]` generated by `omni_controller.py` |
| `/<namespace>/joint_states` | `sensor_msgs/msg/JointState` | Output | Real-time motor positions and filtered RPM feedback from ESP32 |

---

## Troubleshooting

- **Serial Permission Error (`/dev/ttyACM0`):**
  ```bash
  sudo usermod -a -G dialout $USER
  sudo chmod 666 /dev/ttyACM0
  ```
- **Micro-ROS Session Failed:**
  - Verify USB connection and baud rate (115200).
  - Execute `./scripts/start_micro_ros_agent.sh` to allow automatic DTR reset timing.
  - Ensure `esptool` is available (`pip install esptool`).