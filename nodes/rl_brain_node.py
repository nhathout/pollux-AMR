#!/usr/bin/env python3
"""
pollux_rl_trainer_node.py

A self-contained ROS + Gym environment that trains a PPO policy
in real-time on the physical robot. It uses bottom and front ultrasonic
readings to avoid cliffs/obstacles and tries to move around (for coverage).
"""

import rospy
import gym
import numpy as np
import time

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv

from std_msgs.msg import Float32MultiArray, Int32
from sensor_msgs.msg import Imu  # optional if using IMU
from gym import spaces

# ==============================
# Action Definitions
# ==============================
# Must match motor_cmd_node.py:
FORWARD_CMD  = 0
BACKWARD_CMD = 1
STOP_CMD     = 6
SPIN_LEFT    = 4  # or 8 if you prefer "slight spin"
SPIN_RIGHT   = 5  # or 9 if you prefer "slight spin"

# You can pick which discrete actions you want
ACTION_MAP = {
    0: FORWARD_CMD,
    1: BACKWARD_CMD,
    2: SPIN_LEFT,
    3: SPIN_RIGHT,
    4: STOP_CMD
}

# ==============================
# Environment Class
# ==============================
class PolluxRealEnv(gym.Env):
    """
    A custom Gym environment that uses real sensor data (bottom & front ultrasonics)
    and publishes motor commands to the robot in order to do live RL training.

    Observations:
      - bottom_sensors (3) => [dist_bottom_left, dist_bottom_mid, dist_bottom_right]
      - front_sensors (2)  => [dist_front_left, dist_front_right]
    Actions (discrete):
      - 0 => forward
      - 1 => backward
      - 2 => spin left
      - 3 => spin right
      - 4 => stop
    Reward Strategy (example):
      - +0.1 per timestep to encourage movement/coverage
      - -10.0 if we detect a "cliff" (any bottom sensor > cliff threshold)
      - -5.0 if front sensor < obstacle threshold
      - Episode ends ("done") if a cliff is triggered
    """

    def __init__(self):
        super(PolluxRealEnv, self).__init__()

        # ROS parameters / thresholds
        self.cliff_threshold = 15.0     # cm => reading above => "cliff"
        self.obstacle_threshold = 12.0  # cm => reading below => obstacle
        self.rate_hz = 2                # Control frequency (2 Hz => step every ~0.5s)

        # The raw sensor data from ROS
        self.bottom_data = [0.0, 0.0, 0.0]  # [left, mid, right]
        self.front_data  = [0.0, 0.0]       # [left, right]
        self.last_action_time = 0.0

        # Setup observation & action spaces for Gym
        # Observations: 5 floats (bottom3 + front2)
        # We'll limit them to [0, 100] cm or -1 if invalid
        obs_low  = np.array([0, 0, 0, 0, 0], dtype=np.float32)
        obs_high = np.array([100, 100, 100, 100, 100], dtype=np.float32)
        self.observation_space = spaces.Box(low=obs_low, high=obs_high, shape=(5,), dtype=np.float32)

        # Actions: discrete set
        # 0: forward, 1: backward, 2: spin_left, 3: spin_right, 4: stop
        self.action_space = spaces.Discrete(len(ACTION_MAP))

        # Initialize ROS pieces in the environment (subscribers/publishers)
        self.init_ros()

        # We track "episode" in real life with a manual done condition
        self.episode_start_time = time.time()
        self.episode_max_duration = 60.0  # e.g., 60 seconds per episode

    def init_ros(self):
        """
        Initialize ROS subscribers/publishers. We'll do this once.
        """
        # Subscribers (ultrasonic data)
        rospy.Subscriber('/pollux/ultrasonic_hw', Float32MultiArray, self.bottom_callback)
        rospy.Subscriber('/pollux/ultrasonic_2', Float32MultiArray, self.front_callback)
        # IMU optional:
        # rospy.Subscriber('/pollux/imu', Imu, self.imu_callback)

        # Publisher for motor commands
        self.cmd_pub = rospy.Publisher('/pollux/motor_cmd', Int32, queue_size=10)

        # A small pause to ensure subscribers are connected
        rospy.sleep(1.0)

    def bottom_callback(self, msg):
        data = list(msg.data)
        # If sensor returns negative => invalid => clamp to 0
        data = [max(d, 0.0) for d in data]
        self.bottom_data = data

    def front_callback(self, msg):
        data = list(msg.data)
        data = [max(d, 0.0) for d in data]
        self.front_data = data

    # def imu_callback(self, msg):
    #     # If you want to incorporate orientation or angular velocity, parse here
    #     pass

    def step(self, action):
        """
        1. Publish the chosen action to the motors
        2. Wait a short period
        3. Collect new sensor data => compute reward
        4. Return (observation, reward, done, info)
        """
        # Convert discrete action to motor command int
        motor_cmd = ACTION_MAP[action]
        self.cmd_pub.publish(motor_cmd)

        # Sleep to let the robot move & sensors update
        rospy.sleep(1.0 / self.rate_hz)

        # Build observation vector
        obs = self._get_observation()

        # Compute reward
        reward = self._get_reward(obs, action)

        # Check done
        done = self._check_done(obs)
        # or time-based end of episode
        if (time.time() - self.episode_start_time) > self.episode_max_duration:
            done = True

        info = {}
        return obs, reward, done, info

    def reset(self):
        """
        Called at the beginning of each episode. 
        For a real robot, you might decide how to "reset" physically:
          - Possibly stop the motors
          - Put the robot in a known location
          - Or do nothing if you can't physically reset easily
        """
        rospy.loginfo("=== [PolluxRealEnv] Episode RESET ===")
        # Stop the robot
        self.cmd_pub.publish(Int32(STOP_CMD))
        rospy.sleep(0.5)

        # Re-initialize episode
        self.episode_start_time = time.time()
        obs = self._get_observation()
        return obs  # Must return initial observation

    def _get_observation(self):
        """
        Gather the sensor data into a single 5-element float array:
          [bottom_left, bottom_mid, bottom_right, front_left, front_right]
        We'll clamp them to [0, 100] for the sake of our Box space.
        """
        bleft, bmid, bright = self.bottom_data
        fleft, fright       = self.front_data
        # Simple clamp to 100
        arr = np.array([
            min(bleft, 100.0),
            min(bmid,  100.0),
            min(bright,100.0),
            min(fleft, 100.0),
            min(fright,100.0),
        ], dtype=np.float32)
        return arr

    def _get_reward(self, obs, action):
        """
        Example reward function:
          +0.1 per timestep to encourage movement
          -10 if any bottom sensor > cliff_threshold => near cliff
          -5 if any front sensor < obstacle_threshold => near obstacle
          Slight negative if action is STOP (discourage idle)
        Customize as needed.
        """
        reward = 0.1  # base reward each timestep

        bleft, bmid, bright, fleft, fright = obs
        # Cliff penalty
        if (bleft > self.cliff_threshold or
            bmid  > self.cliff_threshold or
            bright> self.cliff_threshold):
            reward -= 10.0

        # Obstacle penalty
        if (fleft < self.obstacle_threshold and fleft > 0) or \
           (fright < self.obstacle_threshold and fright > 0):
            reward -= 5.0

        # Slight negative if we used STOP
        if ACTION_MAP[action] == STOP_CMD:
            reward -= 0.05

        return reward

    def _check_done(self, obs):
        """
        End episode if we detect a cliff (to avoid real accidents).
        Or you can also end if the penalty is severe.
        """
        bleft, bmid, bright, fleft, fright = obs
        # If we see a "cliff", let's end episode
        if (bleft > self.cliff_threshold or
            bmid  > self.cliff_threshold or
            bright> self.cliff_threshold):
            rospy.logwarn("Cliff triggered => Episode done!")
            return True
        return False


# ==============================
# Main: Train PPO on this Env
# ==============================
def main():
    rospy.init_node("rl_brain_node", anonymous=True)

    # Create environment
    env = PolluxRealEnv()

    # If you want a VecEnv for stable baselines:
    vec_env = DummyVecEnv([lambda: env])

    # Create PPO model
    model = PPO(
        policy="MlpPolicy",
        env=vec_env,
        verbose=1,
        device='cpu',  # likely on a Pi
        learning_rate=1e-4,
        n_steps=256,   # adjust as you see fit
        batch_size=32, # adjust for Pi performance
        ent_coef=0.01, # encourage exploration
    )

    rospy.loginfo("Starting real-time PPO training on Pollux...")

    try:
        # Train for a certain # of timesteps or until you terminate
        model.learn(total_timesteps=1000000)  # e.g. 1e6 steps
    except KeyboardInterrupt:
        rospy.loginfo("Training interrupted by user!")

    rospy.loginfo("Saving final model to /models/pollux_rl_model.zip")
    model.save("../models/pollux_rl_model.zip")

    rospy.loginfo("Training done. Spin for a bit or exit.")
    rospy.spin()


if __name__ == "__main__":
    main()