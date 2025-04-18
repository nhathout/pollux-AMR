#!/usr/bin/env python3
"""
rl_brain_node.py ‑ v2
=====================
An on‑robot reinforcement‑learning controller for Pollux‑AMR that can **train** a PPO
policy live, **resume** a previous run, or **run inference only** using a saved model.

Major design notes (compared with the original draft):
----------------------------------------------------
* Added **CLI flags** (`--mode`, `--model`, `--timesteps`, `--save-every`) so you don't
  have to edit the file when switching between training and deployment.
* Wrapped the environment in `VecMonitor` → automatic reward/episode stats + clean
  `model.learn()` interruptions.
* Introduced `rospy.Rate` instead of bare `sleep` for more deterministic timing.
* Robot **always stops** on shutdown (`rospy.on_shutdown`).
* Sensor callbacks now store **timestamps** so stale data is detectable.
* Added configurable **observation normalisation** (min‑max → [0, 1]) for more stable
  learning.
* `step()` returns a Gym‑Kit style `(obs, reward, done, info)`; compatible with
  Stable‑Baselines 3′s Gym‑API expectation.
* Periodic checkpointing during training (`--save-every N` timesteps).
* All ROS parameters (topics, thresholds) are constants at the top for easy tweaking.

Usage examples
--------------
Train from scratch for ~200 k timesteps, saving every 25 k:
```bash
rosrun pollux_amr rl_brain_node.py --mode train --timesteps 200000 --save-every 25000
```
Run inference only with an existing model:
```bash
rosrun pollux_amr rl_brain_node.py --mode infer --model ../models/pollux_rl_model.zip
```
Resume training from a checkpoint:
```bash
rosrun pollux_amr rl_brain_node.py --mode resume --model ../models/pollux_rl_model.zip --timesteps 500000
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

from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor

# ================= ROS / Robot constants =================
BOTTOM_TOPIC  = "/pollux/ultrasonic_hw"     # [left, mid, right]
FRONT_TOPIC   = "/pollux/ultrasonic_2"      # [left, right]
CMD_TOPIC     = "/pollux/motor_cmd"

CLIFF_THRESHOLD      = 15.0   # cm – reading **above** ⇒ cliff
OBSTACLE_THRESHOLD   = 12.0   # cm – reading **below** ⇒ obstacle
MAX_SENSOR_VALUE_CM  = 100.0  # clamp/dist‑normalisation upper bound

CTRL_HZ       = 2            # env step() frequency
EP_MAX_SECS   = 90.0         # stop episode after this many seconds

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
    4: STOP_CMD
}

# =============== PolluxRealEnv implementation ===============
class PolluxRealEnv(gym.Env):
    """Live Gym environment backed by ROS sensor data."""

    metadata = {"render.modes": []}

    def __init__(self):
        super().__init__()

        # ROS – publishers / subscribers
        self.bottom_data = [0.0, 0.0, 0.0]
        self.front_data  = [0.0, 0.0]
        self._bottom_ts  = 0.0
        self._front_ts   = 0.0

        rospy.Subscriber(BOTTOM_TOPIC, Float32MultiArray, self._bottom_cb, queue_size=5)
        rospy.Subscriber(FRONT_TOPIC,  Float32MultiArray, self._front_cb,  queue_size=5)
        self.cmd_pub = rospy.Publisher(CMD_TOPIC, Int32, queue_size=10)
        rospy.sleep(0.5)  # allow time for connections

        # Gym spaces
        low  = np.zeros(5, dtype=np.float32)
        high = np.ones(5, dtype=np.float32)  # normalised to [0,1]
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)
        self.action_space      = spaces.Discrete(len(ACTION_MAP))

        # Episode bookkeeping
        self._episode_start = time.time()
        self._last_action   = None
        self._last_obs      = None
        self._obs_hist      = deque(maxlen=3)
        self.rate           = rospy.Rate(CTRL_HZ)

    # ------------------ ROS Callbacks ------------------
    def _bottom_cb(self, msg):
        data = [max(d, 0.0) for d in msg.data]
        if len(data) == 3:
            self.bottom_data = data
            self._bottom_ts  = time.time()

    def _front_cb(self, msg):
        data = [max(d, 0.0) for d in msg.data]
        if len(data) == 2:
            self.front_data = data
            self._front_ts  = time.time()

    # ------------------ Gym API ------------------
    def reset(self):
        self.cmd_pub.publish(STOP_CMD)
        rospy.sleep(0.3)

        self._episode_start = time.time()
        self._last_action   = None
        self._last_obs      = None
        self._obs_hist.clear()

        return self._get_obs()

    def step(self, action: int):
        # 1 Send command
        self.cmd_pub.publish(ACTION_MAP[action])

        # 2 Wait for next control tick
        self.rate.sleep()

        # 3 Collect observation & compute reward
        obs   = self._get_obs()
        reward= self._compute_reward(obs, action)
        done  = self._check_done(obs)

        # Optional timeout
        if (time.time() - self._episode_start) > EP_MAX_SECS:
            done = True

        info = {}
        self._last_action = action
        self._last_obs    = obs
        self._obs_hist.append(obs)
        return obs, reward, done, info

    # ------------------ Helper methods ------------------
    def _get_obs(self):
        """Return 5‑element normalised observation in [0,1]."""
        b = [min(v, MAX_SENSOR_VALUE_CM) / MAX_SENSOR_VALUE_CM for v in self.bottom_data]
        f = [min(v, MAX_SENSOR_VALUE_CM) / MAX_SENSOR_VALUE_CM for v in self.front_data]
        return np.array(b + f, dtype=np.float32)

    def _compute_reward(self, obs, action):
        base = 0.1
        penalty = 0.0

        # Denormalise for logic
        b_vals = [v * MAX_SENSOR_VALUE_CM for v in obs[:3]]
        f_vals = [v * MAX_SENSOR_VALUE_CM for v in obs[3:]]

        # Cliff & obstacle penalties
        if any(v > CLIFF_THRESHOLD for v in b_vals):
            penalty += 10.0
        if any(0.0 < v < OBSTACLE_THRESHOLD for v in f_vals):
            penalty += 5.0

        # Action penalties
        if ACTION_MAP[action] == STOP_CMD:
            penalty += 0.05
        if self._last_action is not None and action == self._last_action:
            penalty += 0.01

        # Movement / exploration penalty
        if self._last_obs is not None and np.linalg.norm(obs - self._last_obs) < 0.01:
            penalty += 0.02

        return base - penalty

    def _check_done(self, obs):
        b_vals = [v * MAX_SENSOR_VALUE_CM for v in obs[:3]]
        return any(v > CLIFF_THRESHOLD for v in b_vals)

    # ------------------ Housekeeping ------------------
    def close(self):
        self.cmd_pub.publish(STOP_CMD)
        super().close()

# =============== Main entry ===============

def parse_args():
    p = argparse.ArgumentParser(description="RL brain node for Pollux‑AMR")
    p.add_argument("--mode", choices=["train", "infer", "resume"], default="infer",
                   help="Run training, resume training, or inference only")
    p.add_argument("--model", type=str, default="../models/pollux_rl_model.zip",
                   help="Path to save/load the PPO model")
    p.add_argument("--timesteps", type=int, default=200_000,
                   help="Total timesteps to train (only for train/resume)")
    p.add_argument("--save-every", type=int, default=50_000,
                   help="Checkpoint frequency in timesteps (train/resume)")
    return p.parse_args()


def main():
    args = parse_args()
    rospy.init_node("rl_brain_node", anonymous=True)

    env = VecMonitor(DummyVecEnv([lambda: PolluxRealEnv()]))

    model_path = Path(args.model)

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

    # Ensure the robot stops on shutdown
    rospy.on_shutdown(lambda: env.envs[0].cmd_pub.publish(STOP_CMD))

    if args.mode == "infer":
        rospy.loginfo("Running inference… Press Ctrl+C to exit.")
        try:
            while not rospy.is_shutdown():
                obs = env.reset()
                done = False
                while not done and not rospy.is_shutdown():
                    action, _ = model.predict(obs, deterministic=True)
                    obs, _, done, _ = env.step(action)
        except rospy.ROSInterruptException:
            pass

    else:  # train / resume
        total_steps = args.timesteps
        save_every  = args.save_every
        next_ckpt   = save_every
        rospy.loginfo(f"Starting training for {total_steps:,} timesteps…")

        def save_callback(_locals, _globals):
            nonlocal next_ckpt
            num_timesteps = _locals["self"].num_timesteps
            if num_timesteps >= next_ckpt:
                ckpt_path = model_path.with_stem(f"pollux_rl_{num_timesteps//1000}k")
                rospy.loginfo(f"Checkpoint @ {num_timesteps:,} → {ckpt_path}")
                _locals["self"].save(ckpt_path)
                next_ckpt += save_every
            return True

        try:
            model.learn(total_timesteps=total_steps, callback=save_callback)
        except rospy.ROSInterruptException:
            rospy.logwarn("ROS shutdown detected – saving model before exit…")
        finally:
            rospy.loginfo("Final save → %s", model_path)
            model.save(model_path)

if __name__ == "__main__":
    main()