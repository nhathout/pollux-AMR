# pollux-AMR
![Maintained Badge](https://img.shields.io/badge/status-stable-yellow)<br>
An autonomous countertop‑cleaning robot designed to avoid falling off edges (“cliffs”) and colliding with obstacles. Pollux‑AMR uses multiple ultrasonic sensors, an IMU, and ROS Noetic to orchestrate all hardware and software interactions. The system can optionally incorporate reinforcement learning for more advanced navigation or policy‑based behaviors.

## Features
- **5× Ultrasonic Sensors**  
  - **3× Downward‑facing** to detect countertop edges (cliffs).  
  - **2× Forward‑facing** for obstacle avoidance.  
- **IMU‑Based LED Control**  
  - Uses gyro/accelerometer data to enable or disable a UV sanitizing LED strip when tilt or angular velocity is unsafe.  
- **ROS Noetic Integration**  
  - Sensor data is published via dedicated hardware publisher nodes.  
  - Motor commands and LED operations are handled by specialized nodes.  
- **Reinforcement Learning (Optional)**  
  - Can integrate with Stable Baselines3 (PyTorch) if you choose to train/adjust navigation policies.  
- **Headless Ubuntu 20.04 on Raspberry Pi 4**  
  - All ROS nodes run on a Pi 4 Model B, flashed with Ubuntu 20.04 (headless).  
  - SSH in to manage and monitor.  

## Stack Architecture
1. **Python 3.10+** – Main language for scripting and control logic.  
2. **Stable Baselines3 (SB3)** – Reinforcement learning library (optional).  
3. **OpenAI Gymnasium (Gym)** – Standard RL environment interface (optional).  
4. **ROS Noetic** – Message passing and node‑based architecture for sensor/control pipelines.  
5. **Development Environment** – Typically remote (SSH) from a Mac/PC into the Pi (Ubuntu 20.04).

## Hardware Components
1. **Robot Base**  
   - 2 motorized wheels, 1 castor wheel, platform for mounting sensors.  
2. **Adafruit MPU‑6050 (6‑DOF)**  
   - Accelerometer + gyroscope for orientation, tilt, and motion data.  
3. **5× HC‑SR04 Ultrasonic Sensors**  
   - 3x downward for cliff detection (pins assigned in `hw_publisher.py`).  
   - 2x forward for obstacle avoidance (pins assigned in `hw_publisher_2.py`).  
4. **LED Sub‑System**  
   - A UV LED strip for surface sanitation, plus indicator LEDs; controlled via `led_control_node.py` + `led_gyro_node.py`.  
5. **Stepper Motors and Drivers**  
   - Controlled by `motor_cmd_node.py` using a DualMotorController.  
6. **Raspberry Pi 4 Model B (4GB)**  
   - Core computing platform (ARM Cortex‑A72), 4× USB ports, WiFi, GPIO pins for sensors/motors, runs ROS nodes headlessly.

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
- **To change the WiFi network** that the Pi tries to connect to automatically:<br>
  - ```sudo vim /etc/netplan/50-cloud-init.yaml```<br>
  - Change the SSID and password, then apply changes:<br>
  - ```sudo netplan apply```<br>

  - **Verifying Connection:**
       - ```ip addr show wlan0```
       - ```ping google.com```
  

## Getting Started:
just turn on the Pi!
(added script to our Pi that runs all the ros nodes below)

Note: use ```journalctl -u pollux-robot.service -f``` for live logging details when SSHing<br>
OR Run all nodes in the foreground with roslaunch: ```roslaunch pollux_amr all.launch```

<br><br><br>
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
      ```rostopic echo /pollux/ultrasonic_hw```

3. For motors:<br>
   ```rosrun pollux_amr motor_cmd_node.py```

   - To publish a command (move the motors):<br>
      ```rostopic pub /pollux/motor_cmd std_msgs/Int32 "data: 0"``` (0 is an attempt to move forward)

*To run individual HW tests on the Pi without ROS (legacy tests):*
- ```python3 -m pollux-AMR.hardware.tests.test_imu```
- ```python3 -m pollux-AMR.hardware.tests.test_ultrasonic```
- ```python3 -m pollux-AMR.hardware.tests.test_motors``` (crashes/reboots Pi; I think due to too much power consumption by the motors)
