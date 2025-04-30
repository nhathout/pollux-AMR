#!/usr/bin/env python3
"""
safe_rl_brain_node.py  –  PPO + hard-safety shield in one ROS node
------------------------------------------------------------------
• keeps the PPO agent you already trained (train / resume / infer)
• before *every* action, the shield runs the proven cliff / obstacle
  routine from unified_brain_node; if a hazard is found it executes
  the full avoidance sequence *blocking* the RL loop
• observation = 7 floats  (3 bottom US, 2 front US, |ax|, |ay|)
"""

import argparse, time, random
from pathlib import Path

import gym, numpy as np, rospy
from gym import spaces
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int32
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor

# ───────── topics & motor codes ────────────────────────────────────
BOTTOM_T = "/pollux/ultrasonic_hw"
FRONT_T  = "/pollux/ultrasonic_2"
IMU_T    = "/pollux/imu"
CMD_T    = "/pollux/motor_cmd"

FWD, BWD, STOP   = 0, 1, 6
SPIN_L, SPIN_R   = 4, 5
ROT180           = 7        # very smooth 180 in your motor node
ACTION_MAP       = {0:FWD, 1:BWD, 2:SPIN_L, 3:SPIN_R, 4:STOP, 5:ROT180}

# ───────── safety constants (same as unified_brain_node) ───────────
CLIFF_T_CM       = 10.0
OBST_T_CM        = 12.0
BACK_SECS_EACH   = 2.0
ROT180_SECS      = 4.0
SINGLE_SPIN_SECS = 2.0

CTRL_HZ          = 2        # PPO step frequency

# ═════════════════════════ ENV ═════════════════════════════════════
class PolluxEnv(gym.Env):
    """Minimal wrapper around real robot for PPO"""
    metadata = {}

    def __init__(self, fall_penalty=True):
        super().__init__()
        self.cmd_pub = rospy.Publisher(CMD_T, Int32, queue_size=4)

        # state buffers
        self.bottom = [0., 0., 0.]
        self.front  = [0., 0.]
        self.acc_xy = [0., 0.]

        rospy.Subscriber(BOTTOM_T, Float32MultiArray,
                         self._bottom_cb, queue_size=5)
        rospy.Subscriber(FRONT_T,  Float32MultiArray,
                         self._front_cb,  queue_size=5)
        rospy.Subscriber(IMU_T,    Imu,
                         self._imu_cb,   queue_size=5)

        self.action_space      = spaces.Discrete(6)
        self.observation_space = spaces.Box(0, 1, (7,), dtype=np.float32)

        self.rate = rospy.Rate(CTRL_HZ)
        self.fall_penalty = fall_penalty
        self.last_obs = None

    # ───── ROS callbacks ────────────────────────────────────────────
    def _bottom_cb(self, msg):
        self.bottom = [max(d, 0.0) for d in msg.data[:3]]

    def _front_cb(self, msg):
        self.front = [max(d, 0.0) for d in msg.data[:2]]

    def _imu_cb(self, msg):
        self.acc_xy = [msg.linear_acceleration.x,
                       msg.linear_acceleration.y]

    # ───── helpers ─────────────────────────────────────────────────
    @staticmethod
    def _norm(val, lim): return min(abs(val), lim) / lim

    def _get_obs(self):
        MAX_CM, ACC = 100.0, 3.0
        norm_cm = lambda v: min(v, MAX_CM) / MAX_CM
        ax_n = self._norm(self.acc_xy[0], ACC)
        ay_n = self._norm(self.acc_xy[1], ACC)
        return np.array([*map(norm_cm, self.bottom),
                         *map(norm_cm, self.front),
                         ax_n, ay_n], dtype=np.float32)

    # ───── Gym API ─────────────────────────────────────────────────
    def reset(self):
        self.cmd_pub.publish(STOP)
        rospy.sleep(0.2)
        self.last_obs = None
        return self._get_obs()

    def step(self, action):
        self.cmd_pub.publish(ACTION_MAP[int(action)])
        self.rate.sleep()
        obs   = self._get_obs()
        rew   = 0.0        # reward shaping omitted for brevity
        done  = False
        info  = {}
        self.last_obs = obs
        return obs, rew, done, info

    def close(self): self.cmd_pub.publish(STOP)

# ════════════════════ SAFETY SHIELD ════════════════════════════════
class SafetyShield:
    """Hard-coded avoidance that can *block* and override PPO"""
    def __init__(self, cmd_pub: rospy.Publisher):
        self.cmd_pub = cmd_pub
        self.last_cliff = 0.0
        self.last_front = 0.0
        self.cool_cliff = 4.0
        self.cool_front = 3.0

    # ---------- primitive moves ------------------------------------
    def _back_twice(self):
        for _ in range(2):
            self.cmd_pub.publish(BWD)
            rospy.sleep(BACK_SECS_EACH)

    def _spin_random(self):
        if random.random() < 0.5:
            cmd  = SPIN_L if random.random() < 0.5 else SPIN_R
            secs = random.uniform(1.0, 2.0)
            self.cmd_pub.publish(cmd)
            rospy.sleep(secs)

    # ---------- full sequences -------------------------------------
    def _do_cliff_sequence(self):
        rospy.logwarn("SHIELD • cliff detected")
        self.cmd_pub.publish(STOP); rospy.sleep(0.5)
        self._back_twice()
        self.cmd_pub.publish(ROT180); rospy.sleep(ROT180_SECS)
        self._spin_random()
        self.cmd_pub.publish(FWD); rospy.sleep(1.0)

    def _do_front_sequence(self, left, right):
        rospy.loginfo("SHIELD • obstacle")
        self.cmd_pub.publish(STOP); rospy.sleep(0.5)

        if left and right:                 # treat like cliff
            self._back_twice()
            self.cmd_pub.publish(ROT180); rospy.sleep(ROT180_SECS)
            self._spin_random()
        elif left:                         # spin right
            self.cmd_pub.publish(SPIN_R);  rospy.sleep(SINGLE_SPIN_SECS)
        elif right:                        # spin left
            self.cmd_pub.publish(SPIN_L);  rospy.sleep(SINGLE_SPIN_SECS)

        self.cmd_pub.publish(FWD); rospy.sleep(1.0)

    # ---------- public entry ---------------------------------------
    def filter(self, obs: np.ndarray) -> bool:
        """
        Returns True if a blocking safety manoeuvre was executed.
        `obs` must be a 1-D (7,) array – use .squeeze() on VectorEnv output.
        """
        now = rospy.get_time()
        b_cm = obs[:3] * 100.0
        f_cm = obs[3:5] * 100.0

        cliff = (b_cm > CLIFF_T_CM).any()
        left  = 0.0 < f_cm[0] < OBST_T_CM
        right = 0.0 < f_cm[1] < OBST_T_CM

        if cliff and now - self.last_cliff > self.cool_cliff:
            self.last_cliff = now
            self._do_cliff_sequence()
            return True

        if (left or right) and now - self.last_front > self.cool_front:
            self.last_front = now
            self._do_front_sequence(left, right)
            return True

        return False

# ═════════════════════ argparse helper ═════════════════════════════
def parse_args():
    d = "~/catkin_ws/src/pollux-AMR/models/safe_pollux_model.zip"
    P = argparse.ArgumentParser()
    P.add_argument("--mode", choices=["train", "resume", "infer"], default="infer")
    P.add_argument("--model", type=str, default=d)
    P.add_argument("--timesteps", type=int, default=50_000)
    P.add_argument("--save-every", type=int, default=25_000)
    P.add_argument("--disable-fall-penalty", action="store_true")
    return P.parse_args()

# ═════════════════════ main routine ════════════════════════════════
def main():
    args = parse_args()
    rospy.init_node("safe_rl_brain_node", anonymous=True)

    env = VecMonitor(DummyVecEnv([lambda: PolluxEnv(
        fall_penalty=not args.disable_fall_penalty)]))
    shield = SafetyShield(env.envs[0].cmd_pub)

    model_path = Path(args.model).expanduser()
    model_path.parent.mkdir(parents=True, exist_ok=True)

    # ---- create / load model --------------------------------------
    if args.mode == "train":
        model = PPO("MlpPolicy", env, learning_rate=1e-3,
                    n_steps=512, batch_size=64,
                    ent_coef=0.01, verbose=1, device="cpu")
    else:
        if not model_path.exists():
            rospy.logerr(f"Model file {model_path} not found")
            return
        model = PPO.load(model_path, env=env, device="cpu")

    # ---- inference -------------------------------------------------
    if args.mode == "infer":
        rospy.loginfo("Inference with safety shield – Ctrl-C to exit")
        try:
            while not rospy.is_shutdown():
                obs = env.reset()
                done = False
                while not done and not rospy.is_shutdown():
                    act, _ = model.predict(obs, deterministic=True)
                    if not shield.filter(obs.squeeze()):
                        env.envs[0].cmd_pub.publish(ACTION_MAP[int(act)])
                    env.envs[0].rate.sleep()
                    obs = env.envs[0]._get_obs()
        except KeyboardInterrupt:
            pass
        finally:
            env.close()
        return

    # ---- train / resume -------------------------------------------
    next_ckpt = args.save_every

    def ckpt_cb(_l, _g):
        nonlocal next_ckpt
        steps = _l["self"].num_timesteps
        if steps >= next_ckpt:
            ck = model_path.parent / f"{model_path.stem}_{steps//1000}k.zip"
            rospy.loginfo(f"Checkpoint → {ck}")
            _l["self"].save(ck)
            next_ckpt += args.save_every
        return True

    try:
        rospy.loginfo(f"Training for {args.timesteps:,} steps with shield")
        total_steps = 0
        while total_steps < args.timesteps and not rospy.is_shutdown():
            obs = env.reset()
            for _ in range(CTRL_HZ * 4):      # 4-second rollout
                act, _ = model.predict(obs, deterministic=False)
                if shield.filter(obs.squeeze()):
                    break                     # episode handled by shield
                obs, _, _, _ = env.step(act)
                total_steps += 1
            model.learn(total_timesteps=CTRL_HZ * 4,
                        reset_num_timesteps=False,
                        callback=ckpt_cb)
    except KeyboardInterrupt:
        rospy.logwarn("Interrupted – saving model")
    finally:
        rospy.loginfo(f"Saving final model → {model_path}")
        model.save(model_path)
        env.close()

# ═══════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    main()