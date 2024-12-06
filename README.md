# Syringe-Control

**Syringe-Control** is a ROS2 package that enables servo control using joystick inputs. The servo's position is adjusted based on the joystick's movement, with device input communication between a ROS2 node and an Arduino.

## Features

- **Joystick Control**: Map joystick movements to servo positions.
- **Serial Communication**: Communicate seamlessly between ROS2 and Arduino.
- **270-Degree Servo Support**: Designed for 270-degree servo motors.
- **Customizable**: Easily adjust servo speed, increment step, and joystick sensitivity.

---

## Getting Started

This package (along with the serial package) should be located in your ROS2 workspace. For example, if I open my "ros2_ws" I should navigate to "src" and see my two packages in the folder. Ensure this is the case before moving forward.

### Prerequisites

Ensure the following are installed on your system:

- **Hardware**:
  - 270-degree servo motor
  - Arduino (e.g., Arduino Uno)
  - Joystick (e.g., Logitech F310)
  - Power supply for the servo (6V–8.4V)
  - USB cables for Arduino and joystick

- **Software**:
  - **ROS2**: [Install ROS2 Jazzy](https://docs.ros.org/en/foxy/Installation.html) or another compatible distribution.
  - **Arduino IDE**: [Download the Arduino IDE](https://www.arduino.cc/en/software).
  - **Git**: Ensure Git is installed to clone the repository.
  - **Joystick Package**: Install the ROS2 `joy` package:
    ```bash
    sudo apt install ros-jazzy-joy
    ```
  - **Serial Communication Library**: Install the ROS2 `serial` package:
    ```bash
    sudo apt install ros-jazzy-serial
    ```
    YOU MUST HAVE THIS IN YOUR ROS2 WORKSPACE IN ORDER FOR THE CODE TO WORK. 

---

### Repository Structure

├── CMakeLists.txt                    # Build system configuration file
├── launch
│   └── ais_arduino_step_motor_joy_command_receiver.launch  # Launch file for ROS2 nodes
├── package.xml                       # ROS2 package metadata
└── src
    ├── ais_arduino_step_motor_joy_command_receiver
    │   └── ais_arduino_step_motor_joy_command_receiver.ino  # Arduino code for motor control
    └── servo_controller.cpp          # C++ source code for controlling the servo motor


