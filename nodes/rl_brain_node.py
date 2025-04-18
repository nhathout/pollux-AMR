#!/usr/bin/env python3
"""
rl_brain_node.py – v3.1 (IMU‑aware + movement shaping)
=====================================================
Minor tweaks over v3:
* **Forward/Spin bonus** (+0.05) encourages exploratory motion.
* **Stuck vs. move shaping** – big Δ‑obs ⇒ +0.05, tiny Δ‑obs ⇒ ‑0.05.
* **Ctrl‑C exits immediately** – clean save & STOP command.

Usage examples
--------------
Train from scratch, save every 25 k steps:
```bash
rosrun pollux_amr rl_brain_node.py --mode train --timesteps 200000 --save-every 25000
```
Resume training:
```bash
rosrun pollux_amr rl_brain_node.py --mode resume --model ~/models/pollux_rl_model.zip --timesteps 500000
```
Run inference only:
```bash
rosrun pollux_amr rl_brain_node.py --mode infer --model ~/models/pollux_rl_model.zip
```
"""

import argparse
import time
from pathlib import Path

import gym
import numpy as np
import rospy
from gym import spaces
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int32
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor

# ================= ROS / Robot constants =================
BOTTOM_TOPIC = "/pollux/ultrasonic_hw"     # [left, mid, right]
FRONT_TOPIC  = "/pollux/ultrasonic_2"      # [left, right]
IMU_TOPIC    = "/pollux/imu"
CMD_TOPIC    = "/pollux/motor_cmd"

CLIFF_THRESHOLD    = 8.5    # cm – reading **above** ⇒ cliff
OBSTACLE_THRESHOLD = 12.0   # cm – reading **below** ⇒ obstacle
MAX_SENSOR_CM      = 100.0  # normalisation upper‑bound

ACC_LIMIT    = 3.0     # m/s² horizontal → possible fall
BACK_PENALTY = 0.03    # discourage backward moves
FALL_PENALTY = 20.0    # heavy penalty if fall detected

FWD_REWARD       = 0.05   # encourage forward / spins
MOVE_REWARD      = 0.05   # reward large obs change
STUCK_PENALTY    = 0.05   # penalise tiny obs change
MOVE_THRESH_REW  = 1.5    # ||Δobs|| > this ⇒ reward
MOVE_THRESH_PUN  = 0.005  # ||Δobs|| < this ⇒ stuck

CTRL_HZ     = 2    # env step() frequency
EP_MAX_SECS = 90   # per‑episode wall‑clock limit

# Motor commands – must match motor_cmd_node.py
FORWARD_CMD   = 0
BACKWARD_CMD  = 1
STOP_CMD      = 6
SPIN_LEFT_CMD = 4
SPIN_RIGHT_CMD= 5

ACTION_MAP = {
    0: FORWARD_CMD,
    1: BACKWARD_CMD,
    2: SPIN_LEFT_CMD,
    3: SPIN_RIGHT_CMD,
    4: STOP_CMD,
}

# =============== Environment ===============
class PolluxRealEnv(gym.Env):
    """Gym‑style wrapper around real sensors + actuators."""

    metadata = {"render.modes": []}

    def __init__(self):
        super().__init__()

        # --- live data holders
        self.bottom = [0.0, 0.0, 0.0]
        self.front  = [0.0, 0.0]
        self.acc_xy = [0.0, 0.0]

        rospy.Subscriber(BOTTOM_TOPIC, Float32MultiArray, self._bottom_cb, queue_size=5)
        rospy.Subscriber(FRONT_TOPIC,  Float32MultiArray, self._front_cb,  queue_size=5)
        rospy.Subscriber(IMU_TOPIC,    Imu,               self._imu_cb,    queue_size=5)
        self.cmd_pub = rospy.Publisher(CMD_TOPIC, Int32, queue_size=10)
        rospy.sleep(0.4)  # allow publisher & subscribers to connect

        self.observation_space = spaces.Box(low=0.0, high=1.0, shape=(7,), dtype=np.float32)
        self.action_space      = spaces.Discrete(len(ACTION_MAP))

        self.rate           = rospy.Rate(CTRL_HZ)
        self._episode_start = 0.0
        self._last_action   = None
        self._last_obs      = None
        self._last_penalty  = False

    # --- ROS callbacks
    def _bottom_cb(self, msg):
        if len(msg.data) == 3:
            self.bottom = [max(d, 0.0) for d in msg.data]

    def _front_cb(self, msg):
        if len(msg.data) == 2:
            self.front = [max(d, 0.0) for d in msg.data]

    def _imu_cb(self, msg):
        self.acc_xy = [msg.linear_acceleration.x, msg.linear_acceleration.y]

    # --- Gym API
    def reset(self):
        self.cmd_pub.publish(STOP_CMD)
        rospy.sleep(0.2)
        self._episode_start = time.time()
        self._last_action   = None
        self._last_obs      = None
        self._last_penalty  = False
        return self._get_obs()

    def step(self, action: int):
        self.cmd_pub.publish(ACTION_MAP[action])
        self.rate.sleep()

        obs   = self._get_obs()
        reward= self._compute_reward(obs, action)
        done  = self._check_done(obs)

        if time.time() - self._episode_start > EP_MAX_SECS:
            done = True

        self._last_action = action
        self._last_obs    = obs
        return obs, reward, done, {}

    # --- helper functions
    def _get_obs(self):
        b = [min(v, MAX_SENSOR_CM) / MAX_SENSOR_CM for v in self.bottom]
        f = [min(v, MAX_SENSOR_CM) / MAX_SENSOR_CM for v in self.front]
        ax, ay = self.acc_xy
        ax_n = min(abs(ax), ACC_LIMIT) / ACC_LIMIT
        ay_n = min(abs(ay), ACC_LIMIT) / ACC_LIMIT
        return np.array(b + f + [ax_n, ay_n], dtype=np.float32)

    def _compute_reward(self, obs, action):
        base = 0.1
        penalty = 0.0
        bonus   = 0.0

        b_vals = [v * MAX_SENSOR_CM for v in obs[:3]]
        f_vals = [v * MAX_SENSOR_CM for v in obs[3:5]]
        ax_n, ay_n = obs[5:]

        # Penalties for hazards
        if any(v > CLIFF_THRESHOLD for v in b_vals):
            penalty += 10.0; self._last_penalty = True
        elif any(0 < v < OBSTACLE_THRESHOLD for v in f_vals):
            penalty += 5.0;  self._last_penalty = True
        else:
            self._last_penalty = False

        if ax_n >= 1.0 or ay_n >= 1.0:
            penalty += FALL_PENALTY

        # Action shaping
        if ACTION_MAP[action] == STOP_CMD:
            penalty += 0.05
        if ACTION_MAP[action] == BACKWARD_CMD and not self._last_penalty:
            penalty += BACK_PENALTY
        if self._last_action is not None and action == self._last_action:
            penalty += 0.01

        # Movement / exploration shaping
        if self._last_obs is not None:
            diff = np.linalg.norm(obs - self._last_obs)
            if diff > MOVE_THRESH_REW:
                bonus += MOVE_REWARD
            elif diff < MOVE_THRESH_PUN:
                penalty += STUCK_PENALTY

        if ACTION_MAP[action] in {FORWARD_CMD, SPIN_LEFT_CMD, SPIN_RIGHT_CMD}:
            bonus += FWD_REWARD

        return base + bonus - penalty

    def _check_done(self, obs):
        b_vals = [v * MAX_SENSOR_CM for v in obs[:3]]
        ax_n, ay_n = obs[5:]
        if any(v > CLIFF_THRESHOLD for v in b_vals):
            return True
        if ax_n >= 1.0 or ay_n >= 1.0:
            rospy.logwarn("Fall detected – episode ended")
            return True
        return False

    def close(self):
        self.cmd_pub.publish(STOP_CMD)
        super().close()

# =============== Main ===============

def parse_args():
    ap = argparse.ArgumentParser(description="Pollux RL brain node (IMU‑aware)")
    ap.add_argument("--mode", choices=["train", "infer", "resume"], default="infer")
    ap.add_argument("--model", type=str, default="~/models/pollux_rl_model.zip")
    ap.add_argument("--timesteps", type=int, default=50000)  # quick run
    ap.add_argument("--save-every", type=int, default=25000)
    return ap.parse_args()


def main():
    args = parse_args()
    rospy.init