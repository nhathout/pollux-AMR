# pollux-AMR
An autonomous countertop-cleaning robot with integrated reinforcement learning to prevent falling off cliffs and hitting obstacles.

**Stack Architecture:**
1. **Python** for coding & development
   
2. **Stable Baslines3 (SB3)** (built on PyTorch) for the RL library
   
3. **OpenAI Gymnasium (Gym)** for simulation
   
4. **ROS (Robot Operating System)** for facilitating communication between the sensors/hardware, the RL libraries, and the simulation tools.

**Hardware Components:**
1. **Robot Base**: 2 wheels, 1 castor wheel

2. **Sensors**: 4 downward-pointing HC-SR04 ultrasonic sensors for cliff detection
    - (Optional) Stereo camera or two Raspberry Pi camera modules