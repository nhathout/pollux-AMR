# pollux-AMR
An autonomous countertop-cleaning robot with integrated reinforcement learning to prevent falling off cliffs and hitting obstacles.

**Stack Architecture:**
1. **Python** for coding & development
   
2. **Stable Baslines3** (built on PyTorch) or **RLib** for the RL library
   
3. **Gazebo** or **OpenAI Gymnasium** for simulation
   
4. **ROS (Robot Operating System)** for facilitating communication between the sensors/hardware, the RL libraries, and the simulation tools.
