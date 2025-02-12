import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rospy
from hardware_interface import PolluxHardware
from pollux_amr_ros.msg import SensorReadings, ControlCommand

class PolluxRealEnv(gym.Env):
    metadata = {'render_modes': ['human'], 'render_fps': 10}

    def __init__(self, render_mode=None):
        super(PolluxRealEnv, self).__init__()
        
        # Initialize hardware interface
        self.robot = PolluxHardware()
        
        # ROS initialization
        rospy.init_node('gym_env_node', anonymous=True)
        
        # Define action and observation space (match simulation)
        self.action_space = spaces.Discrete(5)
        self.memory_size = 10
        self.observation_space = spaces.Dict({
            'sensors': spaces.Box(low=0.0, high=1.0, shape=(5,), dtype=np.float32),
            'recent_positions': spaces.Box(low=0, high=9, shape=(self.memory_size, 2), dtype=np.int32)
        })

        # Initialize state tracking
        self.current_step = 0
        self.max_steps = 200
        self.visited_positions = set()
        self.loop_memory = []

    def step(self, action):
        # Execute action through ROS
        self._execute_ros_action(action)
        
        # Get observation from hardware
        observation = self._get_observation()
        
        # Calculate reward
        reward = self._calculate_reward(observation)
        
        # Check termination conditions
        terminated = self._check_termination()
        truncated = self.current_step >= self.max_steps
        
        return observation, reward, terminated, truncated, {}

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        
        # Reset hardware state
        self.robot.safe_shutdown()
        self.current_step = 0
        self.visited_positions = set()
        self.loop_memory = []
        
        return self._get_observation(), {}

    def close(self):
        self.robot.safe_shutdown()

    def _execute_ros_action(self, action):
        cmd = ControlCommand()
        cmd.action = action
        self.robot.execute_action(action)

    def _get_observation(self):
        # Get raw sensor data from hardware
        hw_data = self.robot.get_sensor_data()
        
        # Convert to simulation-like observation
        sensors = np.array([
            self._convert_distance(hw_data['distances']['front_left']),
            self._convert_distance(hw_data['distances']['front_right']),
            self._convert_distance(hw_data['distances']['front_center']),  # Add if available
            self._convert_imu(hw_data['imu']['accelerometer']),
            self._convert_imu(hw_data['imu']['gyroscope'])
        ], dtype=np.float32)

        # Recent positions (simulated for compatibility)
        recent_positions = np.zeros((self.memory_size, 2), dtype=np.int32)
        
        return {'sensors': sensors, 'recent_positions': recent_positions}

    def _convert_distance(self, distance_cm):
        # Normalize distance sensor reading (0-100cm to 0-1)
        return np.clip(distance_cm / 100.0, 0.0, 1.0)

    def _convert_imu(self, imu_data):
        # Normalize IMU data (adjust based on your sensor range)
        accel_norm = np.array([
            imu_data['x'] / 16.0,  # Assuming ±16g range
            imu_data['y'] / 16.0,
            imu_data['z'] / 16.0
        ])
        return np.linalg.norm(accel_norm)

    def _calculate_reward(self, observation):
        # Implement similar reward logic as simulation
        reward = 0.0
        
        # Add hardware-specific penalties/bonuses
        if observation['sensors'][2] < 0.1:  # Close to obstacle
            reward -= 1.0
            
        return reward

    def _check_termination(self):
        # Check IMU for tilt (cliff detection)
        accel = self.robot.get_sensor_data()['imu']['accelerometer']
        z_accel = accel['z'] / 16.0  # Normalized
        
        # If robot is tilted beyond 45 degrees
        if abs(z_accel) < 0.707:  # cos(45°) ≈ 0.707
            return True
            
        return False