# pollux-AMR
![Maintained Badge](https://img.shields.io/badge/status-stable-yellow)<br>
An autonomous countertop-cleaning robot with integrated reinforcement learning to prevent falling off cliffs and hitting obstacles.

Pollux-AMR uses sensor fusion (IMU, ultrasonic modules), motor control (stepper drivers), and an OLED display to perform navigation tasks on a Raspberry Pi 4 Model B. The goal is to detect countertop edges (“cliffs”), avoid collisions, and learn from its environment.

## Features
- **Edge (Cliff) Detection**: Downward-facing HC-SR04 ultrasonic sensors to detect surface edges.  
- **Obstacle Avoidance**: Optional additional ultrasonic sensors or cameras for collision prevention.  
- **Reinforcement Learning**: Stable Baselines3 (PyTorch) for policy training.  
- **ROS Integration**: ROS Noetic manages sensor data, control nodes, and RL modules.  
- **OLED Display**: Provides real-time status or debug information.

## Stack Architecture
1. **Python 3.10.7** – Main language for scripting and programming.  
2. **Stable Baselines3 (SB3)** – Reinforcement learning library built on PyTorch.  
3. **OpenAI Gymnasium (Gym)** – Standard interface for RL environments and simulations.  
4. **ROS (Robot Operating System) Noetic** – Middleware for inter-node communication and data handling.  
5. **Development Environment** – Was initially Windows 10/11 using WSL2 (Ubuntu 20.04), however now it is SSH from a Mac to the Pi for remote development with the Raspberry Pi flashed with Ubuntu 20.04 LTS (headless).

## Hardware Components
1. **Robot Base**  
   - 2 motorized wheels, 1 castor wheel, and a sturdy platform.  
2. **Adafruit MPU-6050 (6-DOF)**  
   - Accelerometer + gyroscope for orientation.  
3. **4× HC-SR04 Ultrasonic Sensors**  
   - Positioned downward for cliff detection.  
4. **IPSG Step Motor Driver Boards**  
   - Controls stepper motors (if used instead of DC motors).  
5. **IPSG 1.3" OLED Display**  (unconfirmed)
   - For real-time status or debug info.  
6. **Raspberry Pi 4 Model B(4GB)**  
   - Core computing platform and ROS node host.  
7. **Optional Cameras**  (very highly unlikely)
   - Stereo or dual Pi Cameras for advanced obstacle detection or visual RL.

## Setup
- **Clone the Repo**
  ```bash
   git clone https://github.com/nhathout/pollux-AMR.git
   cd pollux-AMR
  ```
- **Install Dependencies**
  - ROS Noetic: Follow the official ROS installation guide for Ubuntu 20.04.
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

## Appendix
![trained](/images/trained.png)
Figure 1: Successful model trained on the Raspberry Pi to prove ability to dynamically train on a Pi 4 Model B.

## License 
This Project is licensed under the MIT License. You are free to modify and distribute this code under those terms.

## Other (Dev) Notes:
- Raspberry Pi flashed with Ubuntu 20.04 LTS (headless)
- ROS Noetic successfully installed
- SSH from a Mac to the Pi for remote development

**Getting Started:**
just turn on the Pi!
(added script to our Pi that runs all the ros nodes below)

Note: use ```journalctl -u pollux-robot.service -f``` for live logging details when SSHing

<br><br>
*Running Everthing (legacy test unless using your own RaspPi)* <br>
IMPORTANT Reminder: whenever SSHing into the Raspberry Pi, to utilize ROS and ROS nodes, you must:<br>
- ```cd catkin_ws/```<br>
- ```catkin_make```<br>
- ```source devel/setup.bash```<br>

1. Terminal 1:<br>
   ```roscore```<br>

2. Terminal 2:<br>
   ```rosrun pollux_amr hw_publisher.py```<br>

3. Terminal 3:<br>
   ```rosrun pollux_amr motor_cmd_node.py```<br>

4. Terminal 4:<br>
   ```rosrun pollux_amr brain_node.py```<br>
<br><br>
*Running ROS (legacy test)*
1. Open a terminal (SSHed into the Pi) and run:<br>
   ```roscore```

2. In another terminal, run the hardware publisher node:<br>
   ```rosrun pollux_amr hw_publisher.py```

   - To check sensor data:<br>
      ```rostopic echo /pollux/imu```<br>
      ```rostopic echo /pollux/ultrasonic```

3. For motors:<br>
   ```rosrun pollux_amr motor_cmd_node.py```

   - To publish a command (move the motors):<br>
      ```rostopic pub /pollux/motor_cmd std_msgs/Int32 "data: 0"``` (0 is an attempt to move forward)

*To run individual HW tests on the Pi without ROS (legacy tests):*
- ```python3 -m pollux-AMR.hardware.tests.test_imu```
- ```python3 -m pollux-AMR.hardware.tests.test_ultrasonic```
- ```python3 -m pollux-AMR.hardware.tests.test_motors``` (crashes/reboots Pi; I think due to too much power consumption by the motors)
