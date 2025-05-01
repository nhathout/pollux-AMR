#!/usr/bin/env python3
"""
safe_rl_brain_node.py  –  PPO + blocking safety shield
Learn exploration / coverage, while never letting the robot drive off
a cliff or ram a wall.  Fully self-contained ROS node.

CLI examples
------------
# inference only (shield on, no learning)
rosrun pollux_amr safe_rl_brain_node.py --mode infer

# train for 200 k steps (checkpoints every 25 k)
rosrun pollux_amr safe_rl_brain_node.py --mode train --timesteps 200000
"""
# ───────────────────────────────────────────────────────────────────
import argparse, random, time
from pathlib import Path

import gym, numpy as np, rospy
from gym import spaces
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int32
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor

# ─── topics & motor codes ─────────────────────────────────────────
BOTTOM_T = "/pollux/ultrasonic_hw"
FRONT_T  = "/pollux/ultrasonic_2"
IMU_T    = "/pollux/imu"
CMD_T    = "/pollux/motor_cmd"

FWD, BWD, STOP       = 0, 1, 6
SPIN_L, SPIN_R, ROT  = 4, 5, 7                 # 7 = 180° rotate
ACTION_MAP = {0: FWD, 1: BWD, 2: SPIN_L, 3: SPIN_R, 4: STOP, 5: ROT}

# ─── shield params (copied from unified_brain_node) ───────────────
CLIFF_CM, OBST_CM = 10.0, 12.0
BACK_SEC, ROT_SEC = 2.0, 4.0
SPIN_SEC          = 2.0
CTRL_HZ           = 2                              # agent control rate

# ═════════════════ Gym ENV ════════════════════════════════════════
class PolluxEnv(gym.Env):
    metadata = {}
    MAX_CM, ACC_LIM = 100.0, 3.0                   # normalisation

    def __init__(self):
        super().__init__()
        self.cmd_pub = rospy.Publisher(CMD_T, Int32, queue_size=4)

        self.bottom = np.zeros(3, np.float32)
        self.front  = np.zeros(2, np.float32)
        self.acc_xy = [0.0, 0.0]

        # ---------- correct subscriber signatures ----------
        rospy.Subscriber(BOTTOM_T, Float32MultiArray, self._bottom_cb,
                         queue_size=5)
        rospy.Subscriber(FRONT_T,  Float32MultiArray, self._front_cb,
                         queue_size=5)
        rospy.Subscriber(IMU_T,    Imu,               self._imu_cb,
                         queue_size=5)

        self.action_space      = spaces.Discrete(6)
        self.observation_space = spaces.Box(0, 1, (7,), np.float32)
        self.rate = rospy.Rate(CTRL_HZ)

        self.prev_obs = None
        self.prev_act = None

    # ── callbacks (accept *args to be future-proof) ────────────────
    def _bottom_cb(self, msg, *args):
        self.bottom[:] = np.maximum(msg.data[:3], 0.0)

    def _front_cb(self, msg, *args):
        self.front[:]  = np.maximum(msg.data[:2], 0.0)

    def _imu_cb(self, msg, *args):
        self.acc_xy = [msg.linear_acceleration.x,
                       msg.linear_acceleration.y]

    # ── helpers ────────────────────────────────────────────────────
    def _get_obs(self):
        to_u = lambda v: min(v, self.MAX_CM) / self.MAX_CM
        ax_u = min(abs(self.acc_xy[0]), self.ACC_LIM) / self.ACC_LIM
        ay_u = min(abs(self.acc_xy[1]), self.ACC_LIM) / self.ACC_LIM
        return np.array([*map(to_u, self.bottom),
                         *map(to_u, self.front),
                         ax_u, ay_u], np.float32)

    def _reward(self, act, obs):
        b_cm, f_cm = obs[:3]*self.MAX_CM, obs[3:5]*self.MAX_CM
        cliff = (b_cm > CLIFF_CM).any()
        left, right = 0 < f_cm[0] < OBST_CM, 0 < f_cm[1] < OBST_CM
        both = left and right

        r = 0.0
        if cliff:  r -= 5.0
        elif both: r -= 2.0
        if left  and ACTION_MAP[act] == SPIN_R: r += 1.0
        if right and ACTION_MAP[act] == SPIN_L: r += 1.0
        if ACTION_MAP[act] == FWD and self.prev_act == ROT: r += 3.0

        if self.prev_obs is not None:
            diff = np.linalg.norm(obs - self.prev_obs)
            r += 0.5 * diff
            if diff < 0.01: r -= 0.05

        if ACTION_MAP[act] == BWD and not (cliff or both): r -= 0.2
        if self.prev_act is not None and act == self.prev_act: r -= 0.1
        return r

    # ── Gym API ────────────────────────────────────────────────────
    def reset(self):
        self.cmd_pub.publish(STOP); rospy.sleep(0.2)
        self.prev_obs = self.prev_act = None
        return self._get_obs()

    def step(self, act):
        self.cmd_pub.publish(ACTION_MAP[int(act)])
        self.rate.sleep()
        obs = self._get_obs()
        rew = self._reward(act, obs)
        self.prev_obs, self.prev_act = obs, act
        return obs, rew, False, {}

    def close(self): self.cmd_pub.publish(STOP)

# ═══════════════════ SAFETY SHIELD ════════════════════════════════
class Shield:
    def __init__(self, pub):
        self.pub = pub
        self.last_cliff = self.last_front = 0.0

    # primitives
    def _back_twice(self):
        for _ in range(2):
            self.pub.publish(BWD); rospy.sleep(BACK_SEC)

    def _rand_spin(self):
        if random.random() < 0.5:
            self.pub.publish(SPIN_L if random.random() < 0.5 else SPIN_R)
            rospy.sleep(random.uniform(1.0, 2.0))

    # sequences
    def _cliff(self):
        self.pub.publish(STOP); rospy.sleep(0.5)
        self._back_twice()
        self.pub.publish(ROT);  rospy.sleep(ROT_SEC)
        self._rand_spin()
        self.pub.publish(FWD);  rospy.sleep(1.0)

    def _obst(self, left, right):
        self.pub.publish(STOP); rospy.sleep(0.5)
        if left and right:
            self._back_twice(); self.pub.publish(ROT); rospy.sleep(ROT_SEC)
            self._rand_spin()
        elif left:
            self.pub.publish(SPIN_R); rospy.sleep(SPIN_SEC)
        elif right:
            self.pub.publish(SPIN_L); rospy.sleep(SPIN_SEC)
        self.pub.publish(FWD); rospy.sleep(1.0)

    # main entry
    def filter(self, obs):
        obs   = np.asarray(obs).squeeze()
        b_cm  = obs[:3] * 100.0
        f_cm  = obs[3:5] * 100.0
        cliff = (b_cm > CLIFF_CM).any()
        left  = 0.0 < f_cm[0] < OBST_CM
        right = 0.0 < f_cm[1] < OBST_CM
        now   = rospy.get_time()

        if cliff and now - self.last_cliff > 4.0:
            self.last_cliff = now; self._cliff(); return True
        if (left or right) and now - self.last_front > 3.0:
            self.last_front = now; self._obst(left, right); return True
        return False

# ═════════════════ argparse / main ════════════════════════════════
def _args():
    d = "~/catkin_ws/src/pollux-AMR/models/safe_pollux_model.zip"
    P = argparse.ArgumentParser()
    P.add_argument("--mode", choices=["train", "resume", "infer"],
                   default="infer")
    P.add_argument("--model", type=str, default=d)
    P.add_argument("--timesteps", type=int, default=50_000)
    P.add_argument("--save-every", type=int, default=25_000)
    return P.parse_args()

def main():
    args = _args()
    rospy.init_node("safe_rl_brain_node", anonymous=True)

    env    = VecMonitor(DummyVecEnv([lambda: PolluxEnv()]))
    shield = Shield(env.envs[0].cmd_pub)

    model_p = Path(args.model).expanduser()
    model_p.parent.mkdir(parents=True, exist_ok=True)

    if args.mode == "train":
        model = PPO("MlpPolicy", env,
                    learning_rate=1e-3, n_steps=512,
                    batch_size=64, ent_coef=0.01,
                    verbose=1, device="cpu")
    else:
        if not model_p.exists():
            rospy.logerr(f"Model {model_p} not found"); return
        model = PPO.load(model_p, env=env, device="cpu")

    # ─── inference ────────────────────────────────────────────────
    if args.mode == "infer":
        rospy.loginfo("Inference (shield active)  –  Ctrl-C to quit")
        try:
            while not rospy.is_shutdown():
                obs = env.reset()
                while not rospy.is_shutdown():
                    act, _ = model.predict(obs, deterministic=True)
                    if not shield.filter(obs):
                        env.envs[0].cmd_pub.publish(ACTION_MAP[int(act)])
                    env.envs[0].rate.sleep()
                    obs = env.envs[0]._get_obs()
        except KeyboardInterrupt:
            pass
        finally:
            env.close()
        return

    # ─── train / resume ───────────────────────────────────────────
    next_ck = args.save_every
    def ckpt(locals_, _globals):
        nonlocal next_ck
        steps = locals_["self"].num_timesteps
        if steps >= next_ck:
            f = model_p.parent / f"{model_p.stem}_{steps//1000}k.zip"
            rospy.loginfo(f"Checkpoint → {f}")
            locals_["self"].save(f); next_ck += args.save_every
        return True

    try:
        total = 0
        rospy.loginfo(f"Training for {args.timesteps:,} steps")
        while total < args.timesteps and not rospy.is_shutdown():
            obs = env.reset()
            for _ in range(CTRL_HZ * 4):                   # ~4 s rollout
                act, _ = model.predict(obs, deterministic=False)
                if shield.filter(obs): break
                obs, _, _, _ = env.step(act); total += 1
            model.learn(total_timesteps=CTRL_HZ * 4,
                        reset_num_timesteps=False,
                        callback=ckpt)
    except KeyboardInterrupt:
        rospy.logwarn("Interrupted – saving model")
    finally:
        rospy.loginfo(f"Saving final model → {model_p}")
        model.save(model_p); env.close()

# ───────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    main()