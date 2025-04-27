# `README_SOFTWARE.md`

# Pollux-AMR Software Report

## Overview
Pollux-AMR software integrates hardware sensor readings, motor control, and optional reinforcement learning into a ROS Noetic-based robotic system running on a Raspberry Pi 4.

## Software Modules
- **hw_publisher.py** – Publishes ultrasonic and IMU sensor data to ROS topics.
- **motor_cmd_node.py** – Listens for motor commands and drives motors accordingly.
- **led_control_node.py** – Controls UV LED strip and status LEDs.
- **led_gyro_node.py** – Manages UV LED based on IMU readings (tilt/unsafe angles).
- **brain_node.py** – (Optional) High-level logic and/or reinforcement learning policy integration.