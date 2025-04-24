#!/usr/bin/env python3
"""
rl_brain_node.py – v3.5

• Adds 180-degree spin (motor-cmd 7) as a sixth RL action.  
• Bonus is awarded only when the agent backs off a hazard and then spins
  left, right, **or** 180 °.  
• Blind reversing penalty increased; cliff penalty still largest.  
• Learning-rate raised to 1 e-3 for quicker learning.

commands
----------------
Train from scratch:

    rosrun pollux_amr rl_brain_node.py --mode train --timesteps 200000 

Resume training on the saved model:

    rosrun pollux_amr rl_brain_node.py --mode resume --model ~/catkin_ws/src/pollux-AMR/models/pollux_rl_model.zip --timesteps 15000

Inference only:

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

# ───── ROS topics ─────
BOTTOM_TOPIC = "/pollux/ultrasonic_hw"
FRONT_TOPIC  = "/pollux/ultrasonic_2"
IMU_TOPIC    = "/pollux/imu"
CMD_TOPIC    = "/pollux/motor_cmd"

# ───── thresholds ─────
CLIFF_THRESHOLD    = 10.0      # cm
OBSTACLE_THRESHOLD = 12.0     # cm
MAX_SENSOR_CM      = 100.0
ACC_LIMIT          = 3.0      # m s⁻² horizontal

# ───── reward constants ─────
CLIFF_PENALTY      = 50.0
BACK_PENALTY       = 0.35
FALL_PENALTY       = 40.0
FWD_REWARD         = 0.08
SPIN_ESCAPE_BONUS  = 6.0
BASE_REWARD        = 0.05
MOVE_REWARD        = 0.05
STUCK_PENALTY      = 0.05
MOVE_THRESH_REW    = 1.5
MOVE_THRESH_PUN    = 0.005

# ───── timing ─────
CTRL_HZ     = 2
EP_MAX_SECS = 90

# ───── motor cmd codes & RL actions ─────
FORWARD_CMD, BACKWARD_CMD, STOP_CMD  = 0, 1, 6
SPIN_L_CMD,  SPIN_R_CMD,  ROT180_CMD = 4, 5, 7

ACTION_MAP = {
    0: FORWARD_CMD,
    1: BACKWARD_CMD,
    2: SPIN_L_CMD,
    3: SPIN_R_CMD,
    4: STOP_CMD,
    5: ROT180_CMD
}

# =====================================================================
class PolluxRealEnv(gym.Env):
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
        self.action_space      = spaces.Discrete(6)   # actions 0-5

        self.rate = rospy.Rate(CTRL_HZ)
        self._episode_start   = 0.0
        self._last_action     = None
        self._last_obs        = None
        self._last_penalty    = False
        self._after_hazard_bk = False

    # ───── callbacks ─────
    def _bottom_cb(self, msg): self.bottom = [max(d, 0.0) for d in msg.data[:3]]
    def _front_cb(self,  msg): self.front  = [max(d, 0.0) for d in msg.data[:2]]
    def _imu_cb(self,    msg): self.acc_xy = [msg.linear_acceleration.x,
                                              msg.linear_acceleration.y]

    # ───── Gym API ─────
    def reset(self):
        self.cmd_pub.publish(STOP_CMD)
        rospy.sleep(0.2)
        self._episode_start   = time.time()
        self._last_action     = self._last_obs = None
        self._last_penalty    = False
        self._after_hazard_bk = False
        return self._get_obs()

    def step(self, action: int):
        self.cmd_pub.publish(ACTION_MAP[action])
        self.rate.sleep()

        obs = self._get_obs()
        reward = self._compute_reward(action, obs)
        done = self._check_done(obs) \
               or (time.time() - self._episode_start > EP_MAX_SECS)

        # track escape pattern
        if ACTION_MAP[action] == BACKWARD_CMD and self._last_penalty:
            self._after_hazard_bk = True
        elif ACTION_MAP[action] in {SPIN_L_CMD, SPIN_R_CMD, ROT180_CMD} \
                and self._after_hazard_bk:
            pass
        else:
            self._after_hazard_bk = False

        self._last_action, self._last_obs = action, obs
        return obs, reward, done, {}

    # ───── helpers ─────
    def _get_obs(self):
        norm = lambda v: min(v, MAX_SENSOR_CM) / MAX_SENSOR_CM
        ax_n = min(abs(self.acc_xy[0]), ACC_LIMIT) / ACC_LIMIT
        ay_n = min(abs(self.acc_xy[1]), ACC_LIMIT) / ACC_LIMIT
        return np.array([*map(norm, self.bottom),
                         *map(norm, self.front),
                         ax_n, ay_n], dtype=np.float32)

    def _compute_reward(self, action: int, obs):
        b_vals, f_vals = obs[:3]*MAX_SENSOR_CM, obs[3:5]*MAX_SENSOR_CM
        ax_n, ay_n = obs[5:]

        penalty = bonus = 0.0

        # hazards
        if any(v > CLIFF_THRESHOLD for v in b_vals):
            penalty += CLIFF_PENALTY; self._last_penalty = True
        elif any(0 < v < OBSTACLE_THRESHOLD for v in f_vals):
            penalty += 5.0;           self._last_penalty = True
        else:
            self._last_penalty = False

        if self.fall_penalty_enabled and (ax_n >= 1.0 or ay_n >= 1.0):
            penalty += FALL_PENALTY

        # action costs / bonuses
        if ACTION_MAP[action] == STOP_CMD:
            penalty += 0.05
        if ACTION_MAP[action] == BACKWARD_CMD and not self._last_penalty:
            penalty += BACK_PENALTY
        if self._after_hazard_bk and ACTION_MAP[action] in \
                {SPIN_L_CMD, SPIN_R_CMD, ROT180_CMD}:
            bonus += SPIN_ESCAPE_BONUS
        if self._last_action is not None and action == self._last_action:
            penalty += 0.01

        # movement delta
        if self._last_obs is not None:
            diff = np.linalg.norm(obs - self._last_obs)
            bonus   += MOVE_REWARD   if diff > MOVE_THRESH_REW else 0.0
            penalty += STUCK_PENALTY if diff < MOVE_THRESH_PUN else 0.0

        if ACTION_MAP[action] in {FORWARD_CMD, SPIN_L_CMD, SPIN_R_CMD, ROT180_CMD}:
            bonus += FWD_REWARD

        return BASE_REWARD + bonus - penalty

    def _check_done(self, obs):
        cliff = any(obs[i]*MAX_SENSOR_CM > CLIFF_THRESHOLD for i in range(3))
        fall  = self.fall_penalty_enabled and (obs[5] >= 1.0 or obs[6] >= 1.0)
        if cliff: return True
        if fall:
            rospy.logwarn("Fall detected – episode terminated"); return True
        return False

    def close(self):
        self.cmd_pub.publish(STOP_CMD)

# =====================================================================
def parse_args():
    p = argparse.ArgumentParser(description="Pollux RL brain node")
    default = "~/catkin_ws/src/pollux-AMR/models/pollux_rl_model.zip"
    p.add_argument("--mode", choices=["train","infer","resume"], default="infer")
    p.add_argument("--model", type=str, default=default)
    p.add_argument("--timesteps", type=int, default=50000)
    p.add_argument("--save-every", type=int, default=25000)
    p.add_argument("--disable-fall-penalty", action="store_true")
    return p.parse_args()

# =====================================================================
def main():
    args = parse_args()
    rospy.init_node("rl_brain_node", anonymous=True)

    env = VecMonitor(DummyVecEnv([lambda: PolluxRealEnv(
        fall_penalty_enabled=not args.disable_fall_penalty)]))

    model_path = Path(args.model).expanduser()
    model_path.parent.mkdir(parents=True, exist_ok=True)

    if args.mode == "train":
        model = PPO("MlpPolicy", env, learning_rate=1e-3, n_steps=512,
                    batch_size=64, ent_coef=0.01, verbose=1, device="cpu")
    elif args.mode in {"resume","infer"}:
        if not model_path.exists():
            rospy.logerr(f"Model file {model_path} not found!"); return
        model = PPO.load(model_path, env=env, device="cpu")
    else:
        raise ValueError("Unsupported mode")

    rospy.on_shutdown(lambda: env.envs[0].cmd_pub.publish(STOP_CMD))

    # ── inference only ──
    if args.mode == "infer":
        rospy.loginfo("Inference – Ctrl+C to exit")
        try:
            while not rospy.is_shutdown():
                obs, done = env.reset(), False
                while not done and not rospy.is_shutdown():
                    action,_ = model.predict(obs, deterministic=True)
                    obs,_,done,_ = env.step(action)
        except KeyboardInterrupt:
            pass
        finally:
            env.close()
        return

    # ── train / resume ──
    next_ckpt = args.save_every
    def save_cb(_l,_g):
        nonlocal next_ckpt
        steps = _l["self"].num_timesteps
        if steps >= next_ckpt:
            ckpt = model_path.parent / f"{model_path.stem}_{steps//1000}k.zip"
            rospy.loginfo(f"Checkpoint @ {steps:,} → {ckpt}")
            _l["self"].save(ckpt); next_ckpt += args.save_every
        return True

    try:
        rospy.loginfo(f"Training for {args.timesteps:,} steps …")
        model.learn(total_timesteps=args.timesteps, callback=save_cb)
    except KeyboardInterrupt:
        rospy.logwarn("Interrupted – saving model …")
    finally:
        rospy.loginfo(f"Final save → {model_path}")
        model.save(model_path); env.close()

if __name__ == "__main__":
    main()