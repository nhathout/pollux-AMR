#!/usr/bin/env python3
"""
rl_brain_node.py ‑ v3 (IMU‑aware)
================================
An on‑robot reinforcement‑learning controller for Pollux‑AMR that can **train** a
PPO policy live, **resume** a previous run, or **run inference only** using a
saved model.

What’s new in **v3**
-------------------
* **IMU integration** – subscribes to `/pollux/imu` and feeds linear‑acceleration
  *x*, *y* into the observation vector.
* **Fall detection** – if |ax| or |ay| > `ACC_LIMIT` (default 3 m/s²) a heavy
  penalty is applied and the episode terminates (simulates falling off a desk).
* **Backward‑motion discouragement** – every BACKWARD action incurs a small
  penalty unless it immediately follows a cliff/obstacle penalty.
* Reward constants pulled up top for easy tuning.
* Existing usage examples retained.

Usage examples
--------------
Train from scratch for ~200 k timesteps, saving every 25 k:
```bash
rosrun pollux_amr rl_brain_node.py --mode train --timesteps 200000 --save-every 25000
```
Run inference only with an existing model:
```bash
rosrun pollux_amr rl_brain_node.py --mode infer --model ~/models/pollux_rl_model.zip
```
Resume training from a checkpoint:
```bash
rosrun pollux_amr rl_brain_node.py --mode resume --model ~/models/pollux_rl_model.zip --timesteps 500000
```
"""

import argparse
import time
from collections import deque
from pathlib import Path

import gym
import numpy as np
import rospy
from gym import spaces
from std_msgs.msg import Float32MultiArray, Int32
from sensor_msgs.msg import Imu

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor

# ================= ROS / Robot constants =================
BOTTOM_TOPIC = "/pollux/ultrasonic_hw"      # [left, mid, right]
FRONT_TOPIC  = "/pollux/ultrasonic_2"       # [left, right]
IMU_TOPIC    = "/pollux/imu"                # custom IMU publisher
CMD_TOPIC    = "/pollux/motor_cmd"

CLIFF_THRESHOLD     = 8.5   # cm – reading **above** ⇒ cliff
OBSTACLE_THRESHOLD  = 12.0   # cm – reading **below** ⇒ obstacle
MAX_SENSOR_CM       = 100.0  # normalisation upper‑bound

ACC_LIMIT  = 3.0   # m/s² horizontal → possible fall
BACK_PENALTY = 0.03  # discourage backward moves
FALL_PENALTY = 20.0  # heavy penalty if fall detected

CTRL_HZ     = 2      # env step() frequency
EP_MAX_SECS = 90.0   # hard timeout per episode

# Motor commands (must align with motor_cmd_node.py)
FORWARD_CMD   = 0
BACKWARD_CMD  = 1
STOP_CMD      = 6
SPIN_LEFT_CMD = 4  # adjust if your driver expects 8
SPIN_RIGHT_CMD= 5  # adjust if your driver expects 9

ACTION_MAP = {
    0: FORWARD_CMD,
    1: BACKWARD_CMD,
    2: SPIN_LEFT_CMD,
    3: SPIN_RIGHT_CMD,
    4: STOP_CMD,
}

# =============== PolluxRealEnv implementation ===============
class PolluxRealEnv(gym.Env):
    """Live Gym environment backed by ROS sensor and IMU data."""

    metadata = {"render.modes": []}

    def __init__(self):
        super().__init__()

        # --- ROS state holders
        self.bottom_data = [0.0, 0.0, 0.0]
        self.front_data  = [0.0, 0.0]
        self.acc_xy      = [0.0, 0.0]   # linear acceleration X, Y

        rospy.Subscriber(BOTTOM_TOPIC, Float32MultiArray, self._bottom_cb, queue_size=5)
        rospy.Subscriber(FRONT_TOPIC,  Float32MultiArray, self._front_cb,  queue_size=5)
        rospy.Subscriber(IMU_TOPIC,    Imu,               self._imu_cb,    queue_size=5)
        self.cmd_pub = rospy.Publisher(CMD_TOPIC, Int32, queue_size=10)
        rospy.sleep(0.5)

        # Observation = 5 ultrasonics + 2 acc = 7
        low  = np.zeros(7, dtype=np.float32)
        high = np.ones(7, dtype=np.float32)
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self.action_space      = spaces.Discrete(len(ACTION_MAP))

        self.rate           = rospy.Rate(CTRL_HZ)
        self._episode_start = time.time()
        self._last_action   = None
        self._last_obs      = None
        self._last_penalty  = False  # did last step incur cliff/obs penalty?

    # ------------------ ROS Callbacks ------------------
    def _bottom_cb(self, msg):
        if len(msg.data) == 3:
            self.bottom_data = [max(d, 0.0) for d in msg.data]

    def _front_cb(self, msg):
        if len(msg.data) == 2:
            self.front_data = [max(d, 0.0) for d in msg.data]

    def _imu_cb(self, msg):
        self.acc_xy = [msg.linear_acceleration.x, msg.linear_acceleration.y]

    # ------------------ Gym API ------------------
    def reset(self):
        self.cmd_pub.publish(STOP_CMD)
        rospy.sleep(0.3)

        self._episode_start = time.time()
        self._last_action   = None
        self._last_obs      = None
        self._last_penalty  = False
        return self._get_obs()

    def step(self, action: int):
        self.cmd_pub.publish(ACTION_MAP[action])
        self.rate.sleep()

        obs    = self._get_obs()
        reward = self._compute_reward(obs, action)
        done   = self._check_done(obs)

        if (time.time() - self._episode_start) > EP_MAX_SECS:
            done = True

        self._last_action = action
        self._last_obs    = obs
        return obs, reward, done, {}

    # ------------------ Helpers ------------------
    def _get_obs(self):
        b = [min(v, MAX_SENSOR_CM) / MAX_SENSOR_CM for v in self.bottom_data]
        f = [min(v, MAX_SENSOR_CM) / MAX_SENSOR_CM for v in self.front_data]
        ax, ay = self.acc_xy
        ax_n = min(abs(ax), ACC_LIMIT) / ACC_LIMIT
        ay_n = min(abs(ay), ACC_LIMIT) / ACC_LIMIT
        return np.array(b + f + [ax_n, ay_n], dtype=np.float32)

    def _compute_reward(self, obs, action):
        base = 0.1
        penalty = 0.0

        b_vals = [v * MAX_SENSOR_CM for v in obs[:3]]
        f_vals = [v * MAX_SENSOR_CM for v in obs[3:5]]
        ax_n, ay_n = obs[5:]

        # Cliff / obstacle penalties
        if any(v > CLIFF_THRESHOLD for v in b_vals):
            penalty += 10.0
            self._last_penalty = True
        elif any(0 < v < OBSTACLE_THRESHOLD for v in f_vals):
            penalty += 5.0
            self._last_penalty = True
        else:
            self._last_penalty = False

        # Fall penalty (high horizontal acceleration)
        if ax_n >= 1.0 or ay_n >= 1.0:
            penalty += FALL_PENALTY

        # Action‑based penalties
        if ACTION_MAP[action] == STOP_CMD:
            penalty += 0.05
        if ACTION_MAP[action] == BACKWARD_CMD and not self._last_penalty:
            penalty += BACK_PENALTY
        if self._last_action is not None and action == self._last_action:
            penalty += 0.01
        if self._last_obs is not None and np.linalg.norm(obs - self._last_obs) < 0.01:
            penalty += 0.02

        return base - penalty

    def _check_done(self, obs):
        b_vals = [v * MAX_SENSOR_CM for v in obs[:3]]
        ax_n, ay_n = obs[5:]
        if any(v > CLIFF_THRESHOLD for v in b_vals):
            return True
        if ax_n >= 1.0 or ay_n >= 1.0:
            rospy.logwarn("Fall detected – episode terminated!")
            return True
        return False

    def close(self):
        self.cmd_pub.publish(STOP_CMD)
        super().close()

# =============== Main entry ===============

def parse_args():
    p = argparse.ArgumentParser(description="RL brain node for Pollux‑AMR (IMU‑aware)")
    p.add_argument("--mode", choices=["train", "infer", "resume"], default="infer")
    p.add_argument("--model", type=str, default="~/models/pollux_rl_model.zip")
    p.add_argument("--timesteps", type=int, default=200_000)
    p.add_argument("--save-every", type=int, default=50_000)
    return p.parse_args()


def main():
    args = parse_args()
    rospy.init_node("rl_brain_node", anonymous=True)

    env = VecMonitor(DummyVecEnv([lambda: PolluxRealEnv()]))
    model_path = Path(Path(args.model).expanduser())

    if args.mode == "train":
        model = PPO("MlpPolicy", env, verbose=1, device="cpu", learning_rate=1e-4,
                    n_steps=512, batch_size=64, ent_coef=0.01)
    elif args.mode in {"infer", "resume"}:
        if not model_path.exists():
            rospy.logerr(f"Model file {model_path} not found! Aborting.")
            return
        model = PPO.load(model_path, env=env, device="cpu")
    else:
        raise ValueError("Unsupported mode")

    rospy.on_shutdown(lambda: env.envs[0].cmd_pub.publish(STOP_CMD))

    if args.mode == "infer":
        rospy.loginfo("Running inference… Press Ctrl+C to exit.")
        try:
            while not rospy.is_shutdown():
                obs = env.reset()
                done = False
                while not done and not rospy.is_shutdown():
                    action, _ = model.predict(obs, deterministic=True)
                    obs, _, done, _ = env