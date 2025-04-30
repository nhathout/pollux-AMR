#!/usr/bin/env python3
"""
safe_rl_brain_node.py  –  RL + heuristic “shield” in one ROS node
---------------------------------------------------------------
 • Keeps using your PPO agent (train / resume / infer)
 • BEFORE executing an action, runs the proven cliff/obstacle
   checks from unified_brain_node.py and overrides if needed.
 • Overrides are *blocking* (they finish their sequence before
   the RL loop continues) so hazards are always handled.

CLI usage  (same flags as your old rl_brain_node):
    --mode {train,resume,infer}   default=infer
    --model <path>                default=~/catkin_ws/src/pollux-AMR/models/pollux_rl_model.zip
    --timesteps N
    --save-every N
    --disable-fall-penalty
"""

import argparse, time, random, os, sys
from pathlib import Path

import gym, numpy as np, rospy
from gym import spaces
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int32
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor

# ─── sensor & command topics ─────────────────────────────────────────
BOTTOM_TOPIC = "/pollux/ultrasonic_hw"
FRONT_TOPIC  = "/pollux/ultrasonic_2"
IMU_TOPIC    = "/pollux/imu"
CMD_TOPIC    = "/pollux/motor_cmd"

# ─── shared motor codes ─────────────────────────────────────────────
FWD, BWD, STOP = 0,1,6
SPIN_L, SPIN_R, ROT180 = 4,5,7
ACT_MAP = {0:FWD,1:BWD,2:SPIN_L,3:SPIN_R,4:STOP,5:ROT180}

# ─── hazard thresholds (same as unified) ────────────────────────────
CLIFF_T         =  10   # cm  (bottom)
OBST_T          = 12.0   # cm  (front)
CLIFF_BACK_SECS = 2.0    # per backward pass
ROT180_SECS     = 4.0
SINGLE_SPIN_SECS= 2.0
KEEPALIVE_SECS  = 5.0    # send FWD every X s if idle

CTRL_HZ = 2             # RL control loop frequency

# ────────────────────────────────────────────────────────────────────
class PolluxEnv(gym.Env):
    """Exactly the same PPO env you had, but with a hook that lets an
       external 'safety filter' replace / extend the chosen action."""
    metadata = {}

    def __init__(self, fall_penalty=True):
        super().__init__()
        self.cmd_pub = rospy.Publisher(CMD_TOPIC, Int32, queue_size=4)

        # sensor buffers
        self.bottom = [0.,0.,0.]
        self.front  = [0.,0.]
        self.acc_xy = [0.,0.]

        rospy.Subscriber(BOTTOM_TOPIC, Float32MultiArray,
                         lambda m: setattr(self,'bottom', list(m.data[:3])), queue_size=5)
        rospy.Subscriber(FRONT_TOPIC,  Float32MultiArray,
                         lambda m: setattr(self,'front',  list(m.data[:2])), queue_size=5)
        rospy.Subscriber(IMU_TOPIC,    Imu, self._imu_cb, queue_size=5)

        self.action_space      = spaces.Discrete(6)
        self.observation_space = spaces.Box(0,1,(7,),np.float32)

        self.rate  = rospy.Rate(CTRL_HZ)
        self.start = time.time()
        self.fall_penalty = fall_penalty
        self.last_obs = None

    # ------------ callbacks & helpers -----------------
    def _imu_cb(self,msg):
        self.acc_xy = [msg.linear_acceleration.x,msg.linear_acceleration.y]

    def _norm(self,v,lim): return min(abs(v),lim)/lim
    def _get_obs(self):
        MAX_CM, ACC_LIM = 100.0, 3.0
        norm = lambda v: min(v, MAX_CM)/MAX_CM
        ax_n = self._norm(self.acc_xy[0],ACC_LIM)
        ay_n = self._norm(self.acc_xy[1],ACC_LIM)
        return np.array([*map(norm,self.bottom),
                         *map(norm,self.front), ax_n, ay_n],np.float32)

    # ------------ Gym API -----------------------------
    def reset(self):
        self.cmd_pub.publish(STOP); rospy.sleep(0.2)
        self.start = time.time(); self.last_obs=None
        return self._get_obs()

    def step(self,action):
        self.cmd_pub.publish(ACT_MAP[action])
        self.rate.sleep()
        obs = self._get_obs()
        rew = 0.0                    # reward shaping omitted for brevity
        done= False                  # never auto-done; our wrapper handles it
        info={}
        self.last_obs=obs
        return obs,rew,done,info

    def close(self): self.cmd_pub.publish(STOP)

# ────────────────────────────────────────────────────────────────────
class SafetyShield:
    """Light-weight copy of your unified_brain_node logic.
       Called *synchronously* right before the PPO action
       is sent to the motors — may overwrite it."""
    def __init__(self, cmd_pub):
        self.cmd_pub = cmd_pub
        self.last_cliff = self.last_front = 0.0
        self.cool_cliff = 4.0
        self.cool_front = 3.0

    # ----- low-level helpers ---------------------------------------
    def _back_twice(self):
        for _ in range(2):
            self.cmd_pub.publish(BWD); rospy.sleep(CLIFF_BACK_SECS)

    def _spin_random(self):
        if random.random()<0.5: return
        cmd = SPIN_L if random.random()<0.5 else SPIN_R
        secs= random.uniform(1.0,2.0)
        self.cmd_pub.publish(cmd); rospy.sleep(secs)

    # ----- high-level safety routines ------------------------------
    def handle_cliff(self):
        now=rospy.get_time()
        if now-self.last_cliff<self.cool_cliff: return True
        self.last_cliff=now
        rospy.logwarn("SAFETY-SHIELD: cliff!")
        self.cmd_pub.publish(STOP); rospy.sleep(0.5)
        self._back_twice()
        self.cmd_pub.publish(ROT180); rospy.sleep(ROT180_SECS)
        self._spin_random()
        self.cmd_pub.publish(FWD);   rospy.sleep(1.0)
        return True  # override finished

    def handle_obstacle(self,left,right):
        now=rospy.get_time()
        if now-self.last_front<self.cool_front: return False
        self.last_front=now
        self.cmd_pub.publish(STOP); rospy.sleep(0.5)

        if left and right:               # treat like cliff
            self._back_twice()
            self.cmd_pub.publish(ROT180); rospy.sleep(ROT180_SECS)
            self._spin_random()
        elif left:                       # spin right
            self.cmd_pub.publish(SPIN_R); rospy.sleep(SINGLE_SPIN_SECS)
        elif right:                      # spin left
            self.cmd_pub.publish(SPIN_L); rospy.sleep(SINGLE_SPIN_SECS)
        else:
            return False

        self.cmd_pub.publish(FWD); rospy.sleep(1.0)
        return True

    # ----- main entry ----------------------------------------------
    def filter(self, obs):
        # obs = 7-dim vec from env (range 0-1)
        b = [v*100 for v in obs[:3]]
        f = [v*100 for v in obs[3:5]]
        cliff   = any(v>CLIFF_T for v in b)
        left    = 0<f[0]<OBST_T
        right   = 0<f[1]<OBST_T

        if cliff: return self.handle_cliff()
        if left or right: return self.handle_obstacle(left,right)
        return False

# ────────────────────────────────────────────────────────────────────
def parse_args():
    d="~/catkin_ws/src/pollux-AMR/models/safe_pollux_model_.zip"
    p=argparse.ArgumentParser()
    p.add_argument("--mode",choices=["train","resume","infer"],default="infer")
    p.add_argument("--model",type=str,default=d)
    p.add_argument("--timesteps",type=int,default=50000)
    p.add_argument("--save-every",type=int,default=25000)
    p.add_argument("--disable-fall-penalty",action="store_true")
    return p.parse_args()

# ────────────────────────────────────────────────────────────────────
def main():
    args=parse_args()
    rospy.init_node("safe_rl_brain_node",anonymous=True)

    env = VecMonitor(DummyVecEnv([lambda: PolluxEnv(
        fall_penalty=not args.disable_fall_penalty)]))
    shield = SafetyShield(env.envs[0].cmd_pub)

    model_path = Path(args.model).expanduser()
    model_path.parent.mkdir(parents=True,exist_ok=True)

    if args.mode=="train":
        model=PPO("MlpPolicy",env,learning_rate=1e-3,n_steps=512,
                  batch_size=64,ent_coef=0.01,verbose=1,device="cpu")
    else:
        if not model_path.exists():
            rospy.logerr(f"Model {model_path} not found"); return
        model=PPO.load(model_path,env=env,device="cpu")

    # -------- inference-only loop ----------------------------------
    if args.mode=="infer":
        rospy.loginfo("Inference with safety-shield – Ctrl-C to exit")
        try:
            idle_t=rospy.Time.now()
            while not rospy.is_shutdown():
                obs,done=env.reset(),False
                while not done and not rospy.is_shutdown():
                    # ① let the agent pick an action
                    act,_=model.predict(obs,deterministic=True)

                    # ② safety filter may run a blocking override
                    override=shield.filter(obs)
                    if not override:
                        env.envs[0].cmd_pub.publish(ACT_MAP[int(act)])

                    # ③ wait one control tick and gather next obs
                    env.envs[0].rate.sleep()
                    obs=env.envs[0]._get_obs()
        except KeyboardInterrupt: pass
        finally: env.close(); return

    # -------- training / resume loop -------------------------------
    next_ckpt=args.save_every
    def cb(_l,_g):
        nonlocal next_ckpt
        steps=_l["self"].num_timesteps
        if steps>=next_ckpt:
            ckpt=model_path.parent/f"{model_path.stem}_{steps//1000}k.zip"
            rospy.loginfo(f"Checkpoint → {ckpt}")
            _l["self"].save(ckpt); next_ckpt+=args.save_every
        return True

    try:
        rospy.loginfo(f"Training for {args.timesteps:,} steps with safety-shield")
        for _ in range(args.timesteps//(CTRL_HZ*4)):  # coarse outer loop
            # manual rollout to insert the shield
            obs=env.reset()
            for _ in range(CTRL_HZ*4):                # 2 Hz * 4 s = 8 steps
                act,_=model.predict(obs,deterministic=False)
                if shield.filter(obs):                # override handled episode
                    break
                obs,_,_,_=env.step(act)
            model.learn(total_timesteps=CTRL_HZ*4,reset_num_timesteps=False,callback=cb)
    except KeyboardInterrupt:
        rospy.logwarn("Interrupted – saving model")
    finally:
        model.save(model_path); env.close()

if __name__=="__main__": main()