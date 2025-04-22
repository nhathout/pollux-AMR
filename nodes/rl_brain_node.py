#!/usr/bin/env python3
"""
rl_brain_node.py – v3.3 (path fix + safer training toggle)
=========================================================
* **Default model path** now points to
  `~/catkin_ws/src/pollux-AMR/models/pollux_rl_model.zip`, matching the repo.
* Add `--disable-fall-penalty` flag so you can train on a rim‑guarded table
  without punishing IMU jolts.
* Escape bonus (+0.04) for a **back‑then‑turn/forward** sequence encourages
  clearing manoeuvres instead of repeated bump‑and‑reverse loops.
* Python‑3.8–safe checkpoint filenames, unconditional final `model.save()`.
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

# ================= ROS topics & constants =================
BOTTOM_TOPIC  = "/pollux/ultrasonic_hw"
FRONT_TOPIC   = "/pollux/ultrasonic_2"
IMU_TOPIC     = "/pollux/imu"
CMD_TOPIC     = "/pollux/motor_cmd"

CLIFF_THRESHOLD    = 8.5   # cm
OBSTACLE_THRESHOLD = 12.0  # cm
MAX_SENSOR_CM      = 100.0
ACC_LIMIT          = 3.0   # m/s² horizontal

CLIFF_PENALTY   = 20.0
BACK_PENALTY    = 0.15
FALL_PENALTY    = 40.0
FWD_REWARD      = 0.08
BASE_REWARD     = 0.05
MOVE_REWARD     = 0.05
STUCK_PENALTY   = 0.05
MOVE_THRESH_REW = 1.5
MOVE_THRESH_PUN = 0.005
ESCAPE_BONUS    = 0.04

CTRL_HZ     = 2
EP_MAX_SECS = 90

FORWARD_CMD   = 0
BACKWARD_CMD  = 1
STOP_CMD      = 6
SPIN_LEFT_CMD = 4
SPIN_RIGHT_CMD= 5
ACTION_MAP = {0: FORWARD_CMD, 1: BACKWARD_CMD, 2: SPIN_LEFT_CMD,
              3: SPIN_RIGHT_CMD, 4: STOP_CMD}

# ================= Environment ============================
class PolluxRealEnv(gym.Env):
    metadata = {"render.modes": []}

    def __init__(self, fall_penalty_enabled: bool = True):
        super().__init__()
        self.bottom = [0.0, 0.0, 0.0]
        self.front  = [0.0, 0.0]
        self.acc_xy = [0.0, 0.0]
        self.fall_penalty_enabled = fall_penalty_enabled

        rospy.Subscriber(BOTTOM_TOPIC, Float32MultiArray, self._bottom_cb, queue_size=5)
        rospy.Subscriber(FRONT_TOPIC,  Float32MultiArray, self._front_cb,  queue_size=5)
        rospy.Subscriber(IMU_TOPIC,    Imu,               self._imu_cb,    queue_size=5)
        self.cmd_pub = rospy.Publisher(CMD_TOPIC, Int32, queue_size=10)
        rospy.sleep(0.3)

        self.observation_space = spaces.Box(0.0, 1.0, shape=(7,), dtype=np.float32)
        self.action_space      = spaces.Discrete(len(ACTION_MAP))

        self.rate = rospy.Rate(CTRL_HZ)
        self._episode_start = 0.0
        self._last_action   = None
        self._last_obs      = None
        self._last_penalty  = False
        self._after_back    = False

    # ---------- callbacks ----------
    def _bottom_cb(self, msg):
        if len(msg.data) == 3:
            self.bottom = [max(d, 0.0) for d in msg.data]

    def _front_cb(self, msg):
        if len(msg.data) == 2:
            self.front = [max(d, 0.0) for d in msg.data]

    def _imu_cb(self, msg):
        self.acc_xy = [msg.linear_acceleration.x, msg.linear_acceleration.y]

    # ---------- Gym API ----------
    def reset(self):
        self.cmd_pub.publish(STOP_CMD)
        rospy.sleep(0.2)
        self._episode_start = time.time()
        self._last_action = self._last_obs = None
        self._last_penalty = False
        self._after_back  = False
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

    # ---------- helpers ----------
    def _get_obs(self):
        norm = lambda v: min(v, MAX_SENSOR_CM) / MAX_SENSOR_CM
        b = list(map(norm, self.bottom))
        f = list(map(norm, self.front))
        ax_n = min(abs(self.acc_xy[0]), ACC_LIMIT) / ACC_LIMIT
        ay_n = min(abs(self.acc_xy[1]), ACC_LIMIT) / ACC_LIMIT
        return np.array(b + f + [ax_n, ay_n], dtype=np.float32)

    def _compute_reward(self, obs, action):
        penalty = bonus = 0.0
        b_vals = [v * MAX_SENSOR_CM for v in obs[:3]]
        f_vals = [v * MAX_SENSOR_CM for v in obs[3:5]]
        ax_n, ay_n = obs[5:]

        if any(v > CLIFF_THRESHOLD for v in b_vals):
            penalty += CLIFF_PENALTY; self._last_penalty = True
        elif any(0 < v < OBSTACLE_THRESHOLD for v in f_vals):
            penalty += 5.0; self._last_penalty = True
        else:
            self._last_penalty = False

        if self.fall_penalty_enabled and (ax_n >= 1.0 or ay_n >= 1.0):
            penalty += FALL_PENALTY

        if ACTION_MAP[action] == STOP_CMD:
            penalty += 0.05
        if ACTION_MAP[action] == BACKWARD_CMD and not self._last_penalty:
            penalty += BACK_PENALTY
        if self._after_back and ACTION_MAP[action] in {FORWARD_CMD, SPIN_LEFT_CMD, SPIN_RIGHT_CMD}:
            bonus += ESCAPE_BONUS
        if self._last_action is not None and action == self._last_action:
            penalty += 0.01

        if self._last_obs is not None:
            diff = np.linalg.norm(obs - self._last_obs)
            if diff > MOVE_THRESH_REW:
                bonus += MOVE_REWARD
            elif diff < MOVE_THRESH_PUN:
                penalty += STUCK_PENALTY

        if ACTION_MAP[action] in {FORWARD_CMD, SPIN_LEFT_CMD, SPIN_RIGHT_CMD}:
            bonus += FWD_REWARD

        return BASE_REWARD + bonus - penalty

    def _check_done(self, obs):
        if any(v * MAX_SENSOR_CM > CLIFF_THRESHOLD for v in obs[:3]):
            return True
        if self.fall_penalty_enabled and (obs[5] >= 1.0 or obs[6] >= 1.0):
            rospy.logwarn("Fall detected – episode ended")
            return True
        return False

    def close(self):
        self.cmd_pub.publish(STOP_CMD)
        super().close()

# ================= arg‑parser =================

def parse_args():
    p = argparse.ArgumentParser(description="Pollux RL brain node (IMU‑aware)")
    p.add_argument("--mode", choices=["train", "infer", "resume"], default="infer")
    default_path = "~/catkin_ws/src/pollux-AMR/models/pollux_rl_model.zip"
    p.add_argument("--model", type=str, default=default_path)
    p.add_argument("--timesteps", type=int, default=50000)
    p.add_argument("--save-every", type=int, default=25000)
    p.add_argument("--disable-fall-penalty", action="store_true",
                   help="Train without IMU fall penalty (use guarded table)")
    return p.parse_args()

# ===================== main ==================

def main():
    args = parse_args()
    rospy.init_node("rl_brain_node", anonymous=True)

    fall_penalty = not args.disable_fall_penalty
    env = VecMonitor(DummyVecEnv([lambda: PolluxRealEnv(fall_penalty)]))

    model_path = Path(Path(args.model).expanduser())
    model_path.parent.mkdir(parents=True, exist_ok=True)

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
        rospy.loginfo("Running inference… press Ctrl+C to exit.")
        try:
            while not rospy.is_shutdown():
                obs, done = env.reset(), False
                while not done and not rospy.is_shutdown():
                    action, _ = model.predict(obs, deterministic=True)
                    obs, _, done, _ = env.step(action)
        except KeyboardInterrupt:
            pass
        finally:
            env.close()
        return

    # -------- TRAIN / RESUME --------
    next_ckpt = args.save_every

    def save_callback(_locals, _globals):
        nonlocal next_ckpt
        steps = _locals["self"].num_timesteps
        if steps >= next_ckpt:
            ckpt_file = model_path.parent / f"{model_path.stem}_{steps//1000}k.zip"
            rospy.loginfo(f"Checkpoint @ {steps:,} → {ckpt_file}")
            _locals["self"].save(ckpt_file)
            next_ckpt += args.save_every
        return True

    try:
        rospy.loginfo(f"Starting training for {args.timesteps:,} timesteps…")
        model.learn(total_timesteps=args.timesteps, callback=save_callback)
    except KeyboardInterrupt:
        rospy.logwarn("Training interrupted – saving model before exit.")
    finally:
        final_file = model_path.expanduser()
        rospy.loginfo(f"Final save → {final_file}")
        model.save(final_file)
        env.close()

if __name__ == "__main__":
    main()