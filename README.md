# pollux-AMR
![Maintained Badge](https://img.shields.io/badge/status-building-yellow)<br>
An autonomous countertop-cleaning robot with integrated reinforcement learning to prevent falling off cliffs and hitting obstacles.

Pollux-AMR uses sensor fusion (IMU, ultrasonic modules), motor control (stepper drivers), and an OLED display to perform navigation tasks on a Raspberry Pi 4 Model B. The goal is to detect countertop edges (“cliffs”), avoid collisions, and learn from its environment.

## Features
- **Edge (Cliff) Detection**: Downward-facing HC-SR04 ultrasonic sensors to detect surface edges.  
- **Obstacle Avoidance**: Optional additional ultrasonic sensors or cameras for collision prevention.  
- **Reinforcement Learning**: Stable Baselines3 (PyTorch) for policy training.  
- **ROS Integration**: ROS Noetic manages sensor data, control nodes, and RL modules.  
- **OLED Display**: Provides real-time status or debug information.

## Stack Architecture
1. **Python 3.10.7** – Main language for scripting and orchestration.  
2. **Stable Baselines3 (SB3)** – Reinforcement learning library built on PyTorch.  
3. **OpenAI Gymnasium (Gym)** – Standard interface for RL environments and simulations.  
4. **ROS2 (Robot Operating System) Noetic** – Middleware for inter-node communication and data handling.  
5. **Development Environment** – Windows 10/11 using WSL2 (Ubuntu 20.04) for convenience.

## Hardware Components
1. **Robot Base**  
   - 2 motorized wheels, 1 castor wheel, and a sturdy platform.  
2. **Adafruit MPU-6050 (6-DOF)**  
   - Accelerometer + gyroscope for orientation.  
3. **4× HC-SR04 Ultrasonic Sensors**  
   - Positioned downward for cliff detection.  
4. **IPSG Step Motor Driver Boards**  
   - Controls stepper motors (if used instead of DC motors).  
5. **IPSG 1.3" OLED Display**  
   - For real-time status or debug info.  
6. **Raspberry Pi 4 Model B(4GB)**  
   - Core computing platform and ROS node host.  
7. **Optional Cameras**  
   - Stereo or dual Pi Cameras for advanced obstacle detection or visual RL.

## Setup
- **Clone the Repo**
  ```bash
   git clone https://github.com/nhathout/pollux-AMR.git
   cd pollux-AMR
  ```
- **Install Dependencies**
  - ROS2 Noetic: Follow the official ROS installation guide for Ubuntu 20.04.
  - Required packages (assuming ROS2 Noetic installed at ```/opt/ros/noetic``` and you have a ```catkin_ws``` at ```~/catkin_ws```):
    ```bash
    chmod +x setup_env.sh
    ./setup_env.sh
    ```
- **Install Custom Gym Environment**
  - ```bash
    pip install -e gym_pollux
    ```

## Contributing
Please feel free to open GitHub Issues, feature requests, and pull requests :)

## License 
This Project is licensed under the MIT License. You are free to modify and distribute this code under those terms.
