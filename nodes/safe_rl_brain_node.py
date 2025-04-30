#!/usr/bin/env python3
"""
safe_rl_brain_node.py
────────────────────────────────────────────────────────────────────
 ▸ PPO agent (Stable-Baselines3) that learns coverage behaviour
 ▸ HARD safety-shield copied from unified_brain_node
 ▸ Reward:
     +0.50 forward-progress (larger Δ-position proxy)
     +1.00 correct single-side avoidance
     +3.00 successful cliff / dual-obstacle routine
     −5.00 cliff hit     −2.00 dual-obstacle hit
     −0.20 blind reverse −0.10 repeated action
 ▸ Observation = 7 floats  (3 bottom US, 2 front US, |ax|,|ay|)
"""

import argparse, random, time
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

FWD,BWD,STOP        = 0,1,6
SPIN_L,SPIN_R,ROT18 = 4,5,7
ACTION_MAP          = {0:FWD,1:BWD,2:SPIN_L,3:SPIN_R,4:STOP,5:ROT18}

# ───────── safety thresholds ───────────────────────────────────────
CLIFF_CM  , OBST_CM  = 10. , 12.
BACK_SEC  , ROT_SEC  = 2.0 , 4.0
SPIN_SEC               = 2.0
CTRL_HZ                = 2        # agent frequency

# ═════════════════════  ENV  ═══════════════════════════════════════
class PolluxEnv(gym.Env):
    metadata = {}
    MAX_CM, ACC_LIM = 100.0, 3.0

    def __init__(self):
        super().__init__()
        self.cmd_pub = rospy.Publisher(CMD_T, Int32, queue_size=4)

        self.bottom = np.zeros(3,dtype=np.float32)
        self.front  = np.zeros(2,dtype=np.float32)
        self.acc_xy = [0.,0.]

        rospy.Subscriber(BOTTOM_T, Float32MultiArray,
                         self._bottom_cb, queue_size=5)
        rospy.Subscriber(FRONT_T,  Float32MultiArray,
                         self._front_cb,  queue_size=5)
        rospy.Subscriber(IMU_T,    Imu,
                         self._imu_cb,    queue_size=5)

        self.action_space      = spaces.Discrete(6)
        self.observation_space = spaces.Box(0,1,(7,),np.float32)

        self.rate = rospy.Rate(CTRL_HZ)
        self.prev_obs = None
        self.prev_act = None

    # ---- callbacks ------------------------------------------------
    def _bottom_cb(self,msg): self.bottom[:] = np.maximum(msg.data[:3],0.0)
    def _front_cb(self,msg):  self.front[:]  = np.maximum(msg.data[:2],0.0)
    def _imu_cb(self,msg):
        self.acc_xy = [msg.linear_acceleration.x,
                       msg.linear_acceleration.y]

    # ---- helpers --------------------------------------------------
    def _get_obs(self):
        to_unit = lambda v: min(v, self.MAX_CM)/self.MAX_CM
        ax_n = min(abs(self.acc_xy[0]), self.ACC_LIM)/self.ACC_LIM
        ay_n = min(abs(self.acc_xy[1]), self.ACC_LIM)/self.ACC_LIM
        return np.array([*map(to_unit,self.bottom),
                         *map(to_unit,self.front),
                         ax_n, ay_n],dtype=np.float32)

    # ---- reward shaping (coverage proxy) --------------------------
    def _reward(self, act, obs):
        b_cm, f_cm = obs[:3]*self.MAX_CM, obs[3:5]*self.MAX_CM
        cliff_hit  = (b_cm > CLIFF_CM).any()
        obst_hit   = ((0<f_cm[0]<OBST_CM) or (0<f_cm[1]<OBST_CM))

        rew = 0.0
        if cliff_hit:      rew -= 5.0
        elif obst_hit and f_cm[0]<OBST_CM and f_cm[1]<OBST_CM:
            rew -= 2.0

        # correct single-side avoidance bonus
        if   f_cm[0]<OBST_CM and ACTION_MAP[act]==SPIN_R: rew += 1.0
        elif f_cm[1]<OBST_CM and ACTION_MAP[act]==SPIN_L: rew += 1.0

        # winner bonus when shield handled hazard (detected via STOP→FWD diff)
        if ACTION_MAP[act]==FWD and self.prev_act==ROT18: rew += 3.0

        # coverage proxy = big state change
        if self.prev_obs is not None:
            diff = np.linalg.norm(obs-self.prev_obs)
            rew += 0.5*diff            # encourage motion
            if diff<0.01: rew -= 0.05  # discourage oscillation

        # small living cost
        if ACTION_MAP[act]==BWD and not (cliff_hit or obst_hit):
            rew -= 0.2
        if self.prev_act is not None and act==self.prev_act:
            rew -= 0.1

        return rew

    # ---- Gym API --------------------------------------------------
    def reset(self):
        self.cmd_pub.publish(STOP); rospy.sleep(0.2)
        self.prev_obs = None; self.prev_act=None
        return self._get_obs()

    def step(self,action):
        self.cmd_pub.publish(ACTION_MAP[int(action)])
        self.rate.sleep()
        obs  = self._get_obs()
        rew  = self._reward(action,obs)
        done = False
        self.prev_obs,self.prev_act = obs,action
        return obs,rew,done,{}

    def close(self): self.cmd_pub.publish(STOP)

# ═══════════════ SAFETY SHIELD ═════════════════════════════════════
class Shield:
    def __init__(self, pub):
        self.pub = pub
        self.last_cliff = self.last_front = 0.0

    def _back_twice(self):
        for _ in range(2):
            self.pub.publish(BWD); rospy.sleep(BACK_SEC)

    def _rand_spin(self):
        if random.random()<.5:
            cmd = SPIN_L if random.random()<.5 else SPIN_R
            self.pub.publish(cmd); rospy.sleep(random.uniform(1,2))

    def _cliff_seq(self):
        self.pub.publish(STOP); rospy.sleep(0.5)
        self._back_twice()
        self.pub.publish(ROT18); rospy.sleep(ROT_SEC)
        self._rand_spin(); self.pub.publish(FWD); rospy.sleep(1.0)

    def _obs_seq(self,left,right):
        self.pub.publish(STOP); rospy.sleep(0.5)
        if left and right:
            self._back_twice(); self.pub.publish(ROT18); rospy.sleep(ROT_SEC)
            self._rand_spin()
        elif left:
            self.pub.publish(SPIN_R); rospy.sleep(SPIN_SEC)
        elif right:
            self.pub.publish(SPIN_L); rospy.sleep(SPIN_SEC)
        self.pub.publish(FWD); rospy.sleep(1.0)

    def filter(self,obs)->bool:
        obs=np.asarray(obs).squeeze()
        b_cm, f_cm = obs[:3]*100, obs[3:5]*100
        left,right = (0<f_cm[0]<OBST_CM),(0<f_cm[1]<OBST_CM)
        cliff      = (b_cm>CLIFF_CM).any()
        now        = rospy.get_time()

        if cliff and now-self.last_cliff>4:
            self.last_cliff=now; self._cliff_seq(); return True
        if (left or right) and now-self.last_front>3:
            self.last_front=now; self._obs_seq(left,right); return True
        return False

# ═══════════════ argparse / main ═══════════════════════════════════
def parse():
    d="~/catkin_ws/src/pollux-AMR/models/safe_pollux_model.zip"
    p=argparse.ArgumentParser()
    p.add_argument("--mode",choices=["train","resume","infer"],default="infer")
    p.add_argument("--model",type=str,default=d)
    p.add_argument("--timesteps",type=int,default=50_000)
    p.add_argument("--save-every",type=int,default=25_000)
    return p.parse_args()

def main():
    args=parse(); rospy.init_node("safe_rl_brain",anonymous=True)

    env = VecMonitor(DummyVecEnv([lambda: PolluxEnv()]))
    shield = Shield(env.envs[0].cmd_pub)

    path=Path(args.model).expanduser(); path.parent.mkdir(parents=True,exist_ok=True)
    if args.mode=="train":
        model=PPO("MlpPolicy",env,learning_rate=1e-3,n_steps=512,
                  batch_size=64,ent_coef=0.01,verbose=1,device="cpu")
    else:
        if not path.exists(): rospy.logerr(f"model {path} missing"); return
        model=PPO.load(path,env=env,device="cpu")

    # ── inference ──
    if args.mode=="infer":
        rospy.loginfo("Inference – Ctrl-C to quit")
        try:
            while not rospy.is_shutdown():
                obs=env.reset()
                while not rospy.is_shutdown():
                    act,_=model.predict(obs,deterministic=True)
                    if not shield.filter(obs): env.envs[0].cmd_pub.publish(ACTION_MAP[int(act)])
                    env.envs[0].rate.sleep(); obs=env.envs[0]._get_obs()
        except KeyboardInterrupt: pass
        finally: env.close(); return

    # ── train / resume ──
    nxt=args.save_every
    def ck(_l,_g):
        nonlocal nxt; st=_l["self"].num_timesteps
        if st>=nxt:
            ckpt=path.parent/f"{path.stem}_{st//1000}k.zip"
            rospy.loginfo(f"checkpoint → {ckpt}"); _l["self"].save(ckpt); nxt+=args.save_every
        return True

    try:
        total=0; rospy.loginfo(f"Training {args.timesteps:,} steps (shield on)")
        while total<args.timesteps and not rospy.is_shutdown():
            obs=env.reset()
            for _ in range(CTRL_HZ*4):         # roll out 4 s
                act,_=model.predict(obs,deterministic=False)
                if shield.filter(obs): break
                obs,_,_,_=env.step(act); total+=1
            model.learn(total_timesteps=CTRL_HZ*4,reset_num_timesteps=False,callback=ck)
    except KeyboardInterrupt: rospy.logwarn("Interrupted – saving")
    finally:
        model.save(path); env.close()

# ───────────────────────────────────────────────────────────────────
if __name__=="__main__": main()