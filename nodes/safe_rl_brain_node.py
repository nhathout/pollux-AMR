#!/usr/bin/env python3
"""
safe_rl_brain_node.py  –  PPO + hard safety-shield in one ROS node
------------------------------------------------------------------
• PPO actions are intercepted by a proven cliff/obstacle routine
  (taken from unified_brain_node).  The shield performs its full,
  blocking manoeuvre whenever a hazard is detected.
• Observation = 7 floats  →  3 bottom US, 2 front US, |ax|, |ay|
• Command set identical to previous projects.

usage
-----
rosrun pollux_amr safe_rl_brain_node.py --mode train  --timesteps 15000
rosrun pollux_amr safe_rl_brain_node.py --mode resume --model <ckpt>
rosrun pollux_amr safe_rl_brain_node.py --mode infer  --model <ckpt>
"""

import argparse, time, random
from pathlib import Path

import gym, numpy as np, rospy
from gym import spaces
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int32
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor

# ─── topics & motor codes ──────────────────────────────────────────
BOTTOM_T = "/pollux/ultrasonic_hw"
FRONT_T  = "/pollux/ultrasonic_2"
IMU_T    = "/pollux/imu"
CMD_T    = "/pollux/motor_cmd"

FWD, BWD, STOP   = 0, 1, 6
SPIN_L, SPIN_R   = 4, 5
ROT180           = 7                 # smooth 180 in your motor node
ACTION_MAP       = {0: FWD, 1: BWD, 2: SPIN_L, 3: SPIN_R, 4: STOP, 5: ROT180}

# ─── safety constants (mirrors unified_brain_node) ────────────────
CLIFF_T_CM       = 10.0
OBST_T_CM        = 12.0
BACK_SECS_EACH   = 2.0
ROT180_SECS      = 4.0
SINGLE_SPIN_SECS = 2.0
CTRL_HZ          = 2                 # PPO control frequency

# ═════════════════════════ ENV  ════════════════════════════════════
class PolluxEnv(gym.Env):
    """Minimal real-robot environment for PPO."""
    metadata = {}

    def __init__(self, fall_penalty: bool = True):
        super().__init__()
        self.cmd_pub = rospy.Publisher(CMD_T, Int32, queue_size=4)

        # sensor buffers
        self.bottom = np.zeros(3, np.float32)
        self.front  = np.zeros(2, np.float32)
        self.acc_xy = [0.0, 0.0]

        rospy.Subscriber(BOTTOM_T, Float32MultiArray,
                         self._bottom_cb, queue_size=5)
        rospy.Subscriber(FRONT_T,  Float32MultiArray,
                         self._front_cb,  queue_size=5)
        rospy.Subscriber(IMU_T,    Imu,
                         self._imu_cb,    queue_size=5)

        self.action_space      = spaces.Discrete(6)
        self.observation_space = spaces.Box(0, 1, (7,), dtype=np.float32)

        self.rate = rospy.Rate(CTRL_HZ)
        self.fall_penalty = fall_penalty

    # ── ROS callbacks ───────────────────────────────────────────────
    def _bottom_cb(self, msg):
        self.bottom[:] = np.maximum(msg.data[:3], 0.0)

    def _front_cb(self, msg):
        self.front[:]  = np.maximum(msg.data[:2], 0.0)

    def _imu_cb(self, msg):
        self.acc_xy = [msg.linear_acceleration.x,
                       msg.linear_acceleration.y]

    # ── helpers ────────────────────────────────────────────────────
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

    # ── Gym API ────────────────────────────────────────────────────
    def reset(self):
        self.cmd_pub.publish(STOP)
        rospy.sleep(0.2)
        return self._get_obs()

    def step(self, action):
        self.cmd_pub.publish(ACTION_MAP[int(action)])
        self.rate.sleep()
        obs  = self._get_obs()
        rew  = 0.0
        done = False          # never auto-done – shield handles hazards
        return obs, rew, done, {}

    def close(self):
        self.cmd_pub.publish(STOP)

# ═════════════════ SAFETY SHIELD ═══════════════════════════════════
class SafetyShield:
    """Blocking cliff / obstacle handler (adapted from unified_brain_node)."""
    def __init__(self, cmd_pub: rospy.Publisher):
        self.cmd_pub = cmd_pub
        self.last_cliff = 0.0
        self.last_front = 0.0
        self.cool_cliff = 4.0      # s
        self.cool_front = 3.0

    # -------- primitives -------------------------------------------
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

    # -------- full sequences ---------------------------------------
    def _cliff_sequence(self):
        rospy.logwarn("SHIELD • cliff")
        self.cmd_pub.publish(STOP);  rospy.sleep(0.5)
        self._back_twice()
        self.cmd_pub.publish(ROT180); rospy.sleep(ROT180_SECS)
        self._spin_random()
        self.cmd_pub.publish(FWD);   rospy.sleep(1.0)

    def _front_sequence(self, left: bool, right: bool):
        rospy.loginfo("SHIELD • obstacle")
        self.cmd_pub.publish(STOP);  rospy.sleep(0.5)
        if left and right:
            self._back_twice()
            self.cmd_pub.publish(ROT180); rospy.sleep(ROT180_SECS)
            self._spin_random()
        elif left:
            self.cmd_pub.publish(SPIN_R);  rospy.sleep(SINGLE_SPIN_SECS)
        elif right:
            self.cmd_pub.publish(SPIN_L);  rospy.sleep(SINGLE_SPIN_SECS)
        self.cmd_pub.publish(FWD);   rospy.sleep(1.0)

    # -------- external entry ---------------------------------------
    def filter(self, obs_flat: np.ndarray) -> bool:
        """Return True if a blocking manoeuvre was executed."""
        now = rospy.get_time()
        b_cm = obs_flat[:3] * 100.0
        f_cm = obs_flat[3:5] * 100.0

        cliff = (b_cm > CLIFF_T_CM).any()
        left  = 0.0 < f_cm[0] < OBST_T_CM
        right = 0.0 < f_cm[1] < OBST_T_CM

        if cliff and now - self.last_cliff > self.cool_cliff:
            self.last_cliff = now
            self._cliff_sequence()
            return True

        if (left or right) and now - self.last_front > self.cool_front:
            self.last_front = now
            self._front_sequence(left, right)
            return True

        return False

# ═════════════════ argparse / main ═════════════════════════════════
def parse_args():
    default = "~/catkin_ws/src/pollux-AMR/models/safe_pollux_model.zip"
    P = argparse.ArgumentParser()
    P.add_argument("--mode", choices=["train", "resume", "infer"], default="infer")
    P.add_argument("--model", type=str, default=default)
    P.add_argument("--timesteps", type=int, default=50_000)
    P.add_argument("--save-every", type=int, default=25_000)
    P.add_argument("--disable-fall-penalty", action="store_true")
    return P.parse_args()

def main():
    args = parse_args()
    rospy.init_node("safe_rl_brain_node", anonymous=True)

    env = VecMonitor(DummyVecEnv([lambda: PolluxEnv(
        fall_penalty=not args.disable_fall_penalty)]))
    shield = SafetyShield(env.envs[0].cmd_pub)

    model_path = Path(args.model).expanduser()
    model_path.parent.mkdir(parents=True, exist_ok=True)

    if args.mode == "train":
        model = PPO("MlpPolicy", env, learning_rate=1e-3,
                    n_steps=512, batch_size=64,
                    ent_coef=0.01, verbose=1, device="cpu")
    else:
        if not model_path.exists():
            rospy.logerr(f"Model {model_path} not found"); return
        model = PPO.load(model_path, env=env, device="cpu")

    # ── inference mode ──────────────────────────────────────────────
    if args.mode == "infer":
        rospy.loginfo("Inference with safety shield – Ctrl-C to exit")
        try:
            while not rospy.is_shutdown():
                obs = env.reset()
                while not rospy.is_shutdown():
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

    # ── training / resume ──────────────────────────────────────────
    next_ckpt = args.save_every

    def ckpt_cb(_l, _g):
        nonlocal next_ckpt
        steps = _l["self"].num_timesteps
        if steps >= next_ckpt:
            ck = model_path.parent / f"{model_path.stem}_{steps//1000}k.zip"
            rospy.loginfo(f"Checkpoint → {ck}")
            _l["self"].save(ck);  next_ckpt += args.save_every
        return True

    try:
        rospy.loginfo(f"Training for {args.timesteps:,} steps with shield")
        total = 0
        while total < args.timesteps and not rospy.is_shutdown():
            obs = env.reset()
            for _ in range(CTRL_HZ * 4):                # 4-second roll-out
                act, _ = model.predict(obs, deterministic=False)
                if shield.filter(obs.squeeze()):
                    break                               # episode handled
                obs, _, _, _ = env.step(act)
                total += 1
            model.learn(total_timesteps=CTRL_HZ * 4,
                        reset_num_timesteps=False,
                        callback=ckpt_cb)
    except KeyboardInterrupt:
        rospy.logwarn("Interrupted – saving model …")
    finally:
        rospy.loginfo(f"Saving final model → {model_path}")
        model.save(model_path); env.close()

# ───────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    main()