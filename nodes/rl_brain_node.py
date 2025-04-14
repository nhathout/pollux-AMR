#!/usr/bin/env python3
"""
rl_brain_node.py

A self-contained ROS + Gym environment that trains a PPO policy
in real-time on the physical robot, using bottom and front ultrasonic
readings to avoid cliffs/obstacles and tries to cover surface area.

Refined reward:
  - +0.1 each step
  - -10 if a cliff is detected
  - -5 if near an obstacle
  - -0.05 if STOP action is chosen
  - -0.01 if same action is repeated consecutively
  - Additional penalty if sensor readings remain nearly the same
    (suggesting it hasn't moved to a 'new area').
"""

import rospy
import gym
import numpy as np
import time
from collections import deque

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

# Discrete action → motor_cmd
ACTION_MAP = {
    0: FORWARD_CMD,
    1: BACKWARD_CMD,
    2: SPIN_LEFT,
    3: SPIN_RIGHT,
    4: STOP_CMD
}

class PolluxRealEnv(gym.Env):
    """
    A custom Gym environment that uses real sensor data (bottom & front ultrasonics)
    and publishes motor commands to the robot in order to do live RL training.

    Observations: (5)
      [dist_bottom_left, dist_bottom_mid, dist_bottom_right,
       dist_front_left, dist_front_right]

    Actions (discrete, 5):
      0 => forward
      1 => backward
      2 => spin left
      3 => spin right
      4 => stop

    Reward Strategy (refined):
      - +0.1 per timestep
      - -10 if any bottom sensor > cliff_threshold => near cliff
      - -5 if any front sensor < obstacle_threshold => near obstacle
      - -0.05 if STOP action is chosen
      - -0.01 if repeating the same action as last step
      - -0.02 if sensor readings do not change significantly from last step
        => implies it hasn't moved to a new area
      - Episode done if cliff is triggered (to prevent real accidents)
    """

    def __init__(self):
        super(PolluxRealEnv, self).__init__()

        # === ROS thresholds ===
        self.cliff_threshold = 15.0     # cm => reading above => "cliff"
        self.obstacle_threshold = 12.0  # cm => reading below => obstacle
        self.rate_hz = 2                # Control frequency (2 Hz => step every ~0.5s)

        # === Sensor data from ROS ===
        self.bottom_data = [0.0, 0.0, 0.0]  # [left, mid, right]
        self.front_data  = [0.0, 0.0]       # [left, right]

        # === Spaces for Gym ===
        obs_low  = np.array([0, 0, 0, 0, 0], dtype=np.float32)
        obs_high = np.array([100, 100, 100, 100, 100], dtype=np.float32)
        self.observation_space = spaces.Box(low=obs_low, high=obs_high, shape=(5,), dtype=np.float32)
        self.action_space = spaces.Discrete(len(ACTION_MAP))

        # === Setup ROS & environment tracking ===
        self.init_ros()

        self.episode_start_time = time.time()
        self.episode_max_duration = 60.0  # 60 seconds per episode (arbitrary)
        
        # For reward shaping:
        self.last_action = None
        self.last_obs = None
        # Keep a short history of observations to detect repeated states (optional)
        self.obs_history = deque(maxlen=3)

    def init_ros(self):
        """
        Initialize ROS subscribers/publishers.
        """
        rospy.Subscriber('/pollux/ultrasonic_hw', Float32MultiArray, self.bottom_callback)
        rospy.Subscriber('/pollux/ultrasonic_2', Float32MultiArray, self.front_callback)
        # Optional IMU:
        # rospy.Subscriber('/pollux/imu', Imu, self.imu_callback)

        self.cmd_pub = rospy.Publisher('/pollux/motor_cmd', Int32, queue_size=10)
        rospy.sleep(1.0)  # small pause to ensure connections

    def bottom_callback(self, msg):
        data = [max(d, 0.0) for d in msg.data]  # clamp negatives
        if len(data) == 3:
            self.bottom_data = data

    def front_callback(self, msg):
        data = [max(d, 0.0) for d in msg.data]
        if len(data) == 2:
            self.front_data = data

    # def imu_callback(self, msg):
    #     # Incorporate orientation or angular velocity if you want
    #     pass

    def reset(self):
        """
        Called at the beginning of each episode.
        Stop the robot, reset counters, return initial observation.
        """
        rospy.loginfo("=== [PolluxRealEnv] Episode RESET ===")
        self.cmd_pub.publish(Int32(STOP_CMD))
        rospy.sleep(0.5)

        self.episode_start_time = time.time()
        self.last_action = None
        self.last_obs    = None
        self.obs_history.clear()

        obs = self._get_observation()
        return obs

    def step(self, action):
        """
        1. Publish the chosen action
        2. Wait to let robot move
        3. Collect new obs, compute reward
        4. Check done
        """
        motor_cmd = ACTION_MAP[action]
        self.cmd_pub.publish(motor_cmd)

        rospy.sleep(1.0 / self.rate_hz)

        obs = self._get_observation()
        reward = self._get_reward(obs, action)
        done = self._check_done(obs)

        # optional time-based cutoff
        if (time.time() - self.episode_start_time) > self.episode_max_duration:
            done = True

        info = {}
        self.last_action = action
        self.last_obs    = obs
        self.obs_history.append(obs)

        return obs, reward, done, info

    def _get_observation(self):
        """
        Combine bottom & front sensor data into a single array [5].
        Clamp to [0, 100].
        """
        bleft, bmid, bright = self.bottom_data
        fleft, fright = self.front_data

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
        Refined reward:
          +0.1 base each step
          -10 if cliff
          -5 if obstacle
          -0.05 if STOP
          -0.01 if repeating same action
          -0.02 if observation hasn't changed => not covering new area
        """
        reward = 0.1  # base

        bleft, bmid, bright, fleft, fright = obs

        # Cliff penalty
        if (bleft > self.cliff_threshold or
            bmid  > self.cliff_threshold or
            bright> self.cliff_threshold):
            reward -= 10.0

        # Obstacle penalty
        if ((0 < fleft  < self.obstacle_threshold) or
            (0 < fright < self.obstacle_threshold)):
            reward -= 5.0

        # STOP penalty
        if ACTION_MAP[action] == STOP_CMD:
            reward -= 0.05

        # Repeated action penalty
        if self.last_action is not None and action == self.last_action:
            reward -= 0.01

        # If obs didn't change much => penalize (not exploring)
        # We'll compare the new obs to the last obs:
        if self.last_obs is not None:
            diff = np.linalg.norm(obs - self.last_obs)
            if diff < 1.0:
                # means sensors are nearly the same => maybe no movement
                reward -= 0.02

        return reward

    def _check_done(self, obs):
        """
        End episode if a cliff is detected or if we want to avoid real accidents.
        """
        bleft, bmid, bright, fleft, fright = obs
        if (bleft > self.cliff_threshold or
            bmid  > self.cliff_threshold or
            bright> self.cliff_threshold):
            rospy.logwarn("Cliff triggered => Episode done!")
            return True
        return False


def main():
    rospy.init_node("rl_brain_node", anonymous=True)

    env = PolluxRealEnv()
    vec_env = DummyVecEnv([lambda: env])

    model = PPO(
        policy="MlpPolicy",
        env=vec_env,
        verbose=1,
        device='cpu',
        learning_rate=1e-4,
        n_steps=256,
        batch_size=32,
        ent_coef=0.01
    )

    rospy.loginfo("Starting real-time PPO training on Pollux with refined coverage reward...")

    try:
        model.learn(total_timesteps=1000000)  # or until Ctrl+C
    except KeyboardInterrupt:
        rospy.loginfo("Training interrupted by user!")

    # Save final model
    rospy.loginfo("Saving final model to /models/pollux_rl_model.zip")
    model.save("../models/pollux_rl_model.zip")

    rospy.loginfo("Training done. Spin or exit.")
    rospy.spin()

if __name__ == "__main__":
    main()