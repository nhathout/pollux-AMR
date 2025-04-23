#!/usr/bin/env python3
"""
rl_brain_node.py – v3.3  (path fix · escape bonus · optional fall penalty)
==========================================================================
* Default model path points to the repo folder:
    ~/catkin_ws/src/pollux-AMR/models/pollux_rl_model.zip
* `--disable-fall-penalty` lets you train safely on a guarded table.

Usage
-----
Train from scratch, checkpoint every 25 k:
    rosrun pollux_amr rl_brain_node.py --mode train --timesteps 200000 --save-every 25000

Resume training from saved model:
    rosrun pollux_amr rl_brain_node.py --mode resume --model ~/catkin_ws/src/pollux-AMR/models/pollux_rl_model.zip --timesteps 10000 --save-every 1000

Run inference only:
    rosrun pollux_amr rl_brain_node.py --mode infer --model ~/catkin_ws/src/pollux-AMR/models/pollux_rl_model.zip
"""
import argparse, time
from pathlib import Path

import gym, numpy as np, rospy
from gym import spaces
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int32
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor

# ---------- ROS topics ----------
BOTTOM_TOPIC = "/pollux/ultrasonic_hw"
FRONT_TOPIC  = "/pollux/ultrasonic_2"
IMU_TOPIC    = "/pollux/imu"
CMD_TOPIC    = "/pollux/motor_cmd"

# ---------- thresholds ----------
CLIFF_THRESHOLD    = 8.5   # cm
OBSTACLE_THRESHOLD = 12.0  # cm
MAX_SENSOR_CM      = 100.0
ACC_LIMIT          = 3.0   # m s⁻² horizontal

# ---------- reward constants ----
CLIFF_PENALTY  = 20.0
BACK_PENALTY   = 0.15
FALL_PENALTY   = 40.0
FWD_REWARD     = 0.08
BASE_REWARD    = 0.05
MOVE_REWARD    = 0.05
STUCK_PENALTY  = 0.05
MOVE_THRESH_REW= 1.5
MOVE_THRESH_PUN= 0.005
ESCAPE_BONUS   = 0.04      # back‑then‑turn/forward

# ---------- timing --------------
CTRL_HZ     = 2
EP_MAX_SECS = 90

# ---------- motor commands ------
FORWARD_CMD   = 0
BACKWARD_CMD  = 1
STOP_CMD      = 6
SPIN_L_CMD    = 4
SPIN_R_CMD    = 5
ACTION_MAP = {0: FORWARD_CMD, 1: BACKWARD_CMD, 2: SPIN_L_CMD,
              3: SPIN_R_CMD, 4: STOP_CMD}

# =====================================================================
class PolluxRealEnv(gym.Env):
    """ROS‑backed Gym environment."""

    metadata = {"render.modes": []}

    def __init__(self, fall_penalty_enabled: bool = True):
        super().__init__()
        self.bottom = [0.0, 0.0, 0.0]
        self.front  = [0.0, 0.0]
        self.acc_xy = [0.0, 0.0]
        self.fall_penalty_enabled = fall_penalty_enabled

        rospy.Subscriber(BOTTOM_TOPIC, Float32MultiArray, self._bottom_cb, 5)
        rospy.Subscriber(FRONT_TOPIC,  Float32MultiArray, self._front_cb,  5)
        rospy.Subscriber(IMU_TOPIC,    Imu,               self._imu_cb,    5)
        self.cmd_pub = rospy.Publisher(CMD_TOPIC, Int32, queue_size=10)
        rospy.sleep(0.3)

        self.observation_space = spaces.Box(0.0, 1.0, shape=(7,), dtype=np.float32)
        self.action_space      = spaces.Discrete(len(ACTION_MAP))

        self.rate = rospy.Rate(CTRL_HZ)
        self._episode_start = 0.0
        self._last_action   = None
        self._last_obs      = None
        self._last_penalty  = False
        self._after_back    = False  # for escape bonus

    # ----------------- ROS callbacks -----------------
    def _bottom_cb(self, msg): self.bottom = [max(d, 0.0) for d in msg.data[:3]]
    def _front_cb(self,  msg): self.front  = [max(d, 0.0) for d in msg.data[:2]]
    def _imu_cb(self,    msg): self.acc_xy = [msg.linear_acceleration.x,
                                              msg.linear_acceleration.y]

    # ----------------- Gym API ----------------------
    def reset(self):
        self.cmd_pub.publish(STOP_CMD)
        rospy.sleep(0.2)
        self._episode_start = time.time()
        self._last_action = self._last_obs = None
        self._last_penalty = False
        self._after_back   = False
        return self._get_obs()

    def step(self, action:int):
        self.cmd_pub.publish(ACTION_MAP[action])
        self.rate.sleep()

        obs   = self._get_obs()
        reward= self._compute_reward(obs, action)
        done  = self._check_done(obs)

        if time.time() - self._episode_start > EP_MAX_SECS:
            done = True

        self._after_back = (ACTION_MAP[action] == BACKWARD_CMD)
        self._last_action, self._last_obs = action, obs
        return obs, reward, done, {}

    # ----------------- helpers ----------------------
    def _get_obs(self):
        norm = lambda v: min(v, MAX_SENSOR_CM) / MAX_SENSOR_CM
        ax_n = min(abs(self.acc_xy[0]), ACC_LIMIT) / ACC_LIMIT
        ay_n = min(abs(self.acc_xy[1]), ACC_LIMIT) / ACC_LIMIT
        return np.array(list(map(norm, self.bottom))
                        + list(map(norm, self.front))
                        + [ax_n, ay_n], dtype=np.float32)

    def _compute_reward(self, obs, action):
        penalty = bonus = 0.0
        b_vals = [v*MAX_SENSOR_CM for v in obs[:3]]
        f_vals = [v*MAX_SENSOR_CM for v in obs[3:5]]
        ax_n, ay_n = obs[5:]

        if any(v > CLIFF_THRESHOLD for v in b_vals):
            penalty += CLIFF_PENALTY; self._last_penalty = True
        elif any(0 < v < OBSTACLE_THRESHOLD for v in f_vals):
            penalty += 5.0;           self._last_penalty = True
        else:
            self._last_penalty = False

        if self.fall_penalty_enabled and (ax_n >= 1.0 or ay_n >= 1.0):
            penalty += FALL_PENALTY

        if ACTION_MAP[action] == STOP_CMD: penalty += 0.05
        if ACTION_MAP[action] == BACKWARD_CMD and not self._last_penalty:
            penalty += BACK_PENALTY
        if self._after_back and ACTION_MAP[action] in {FORWARD_CMD, SPIN_L_CMD, SPIN_R_CMD}:
            bonus += ESCAPE_BONUS
        if self._last_action is not None and action == self._last_action:
            penalty += 0.01

        if self._last_obs is not None:
            diff = np.linalg.norm(obs - self._last_obs)
            bonus += MOVE_REWARD if diff > MOVE_THRESH_REW else 0.0
            penalty += STUCK_PENALTY if diff < MOVE_THRESH_PUN else 0.0

        if ACTION_MAP[action] in {FORWARD_CMD, SPIN_L_CMD, SPIN_R_CMD}:
            bonus += FWD_REWARD

        return BASE_REWARD + bonus - penalty

    def _check_done(self, obs):
        cliff = any(v*MAX_SENSOR_CM > CLIFF_THRESHOLD for v in obs[:3])
        fall  = self.fall_penalty_enabled and (obs[5]>=1.0 or obs[6]>=1.0)
        if cliff: return True
        if fall:
            rospy.logwarn("Fall detected – episode terminated")
            return True
        return False

    def close(self):
        self.cmd_pub.publish(STOP_CMD)
        super().close()

# =====================================================================
def parse_args():
    ap = argparse.ArgumentParser(description="Pollux RL brain node")
    default_model = "~/catkin_ws/src/pollux-AMR/models/pollux_rl_model.zip"
    ap.add_argument("--mode", choices=["train","infer","resume"], default="infer")
    ap.add_argument("--model", type=str, default=default_model)
    ap.add_argument("--timesteps", type=int, default=50000)
    ap.add_argument("--save-every", type=int, default=25000)
    ap.add_argument("--disable-fall-penalty", action="store_true",
                    help="Train on guarded table without IMU fall penalty")
    return ap.parse_args()

# =====================================================================
def main():
    args = parse_args()
    rospy.init