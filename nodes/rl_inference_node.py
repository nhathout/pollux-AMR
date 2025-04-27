#!/usr/bin/env python3
"""
rl_inference_node.py

Runs a previously trained PPO model (e.g., pollux_rl_model.zip) in inference mode,
acting as a 'brain node' for the robot. It reads ultrasonic data, builds an observation,
calls model.predict(), and publishes motor commands continuously.

Usage:
  1. Ensure 'pollux_rl_model.zip' (or your chosen model file) is in an accessible location.
  2. rosrun pollux_amr rl_inference_node.py
     (assuming you made this file executable and placed it in pollux_amr/scripts/)

Dependencies:
  - stable-baselines3
  - gym
  - ROS Noetic
  - The standard hardware publisher nodes:
       hw_publisher.py  (for bottom ultrasonic + IMU, if needed)
       hw_publisher_2.py (for front ultrasonic)
  - The motor command node (motor_cmd_node.py)
"""

import rospy
import time
import numpy as np

from stable_baselines3 import PPO
from std_msgs.msg import Float32MultiArray, Int32

# --- Motor command codes (must match motor_cmd_node.py) ---
FORWARD_CMD  = 0
BACKWARD_CMD = 1
STOP_CMD     = 6
SPIN_LEFT    = 4  # or 8/2 if you prefer different spin codes
SPIN_RIGHT   = 5  # or 9/3 if you prefer different spin codes

# If your model was trained with a discrete action space, ensure this map
# aligns with the action indices used during training.
ACTION_MAP = {
    0: FORWARD_CMD,
    1: BACKWARD_CMD,
    2: SPIN_LEFT,
    3: SPIN_RIGHT,
    4: STOP_CMD
}

class RLInferenceNode:
    def __init__(self):
        rospy.init_node("rl_inference_node", anonymous=True)
        rospy.loginfo("RL Inference Node: Starting up...")

        # === Load PPO Model ===
        # Update the path if your model file is stored elsewhere
        model_path = "pollux_rl_model.zip"
        try:
            self.model = PPO.load(model_path)
            rospy.loginfo(f"Successfully loaded model from {model_path}")
        except Exception as e:
            rospy.logerr(f"Failed to load model from {model_path}: {e}")
            rospy.signal_shutdown("Model load error")
            return

        # === Subscribe to Sensor Topics ===
        self.bottom_data = [0.0, 0.0, 0.0]  # bottom ultrasonic: [left, mid, right]
        self.front_data  = [0.0, 0.0]      # front ultrasonic: [left, right]

        rospy.Subscriber("/pollux/ultrasonic_hw", Float32MultiArray, self.bottom_cb)
        rospy.Subscriber("/pollux/ultrasonic_2", Float32MultiArray, self.front_cb)
        # If you want IMU data, you can subscribe to /pollux/imu similarly.

        # === Motor Command Publisher ===
        self.cmd_pub = rospy.Publisher("/pollux/motor_cmd", Int32, queue_size=10)

        # === Parameters / Thresholds ===
        self.cliff_threshold = 15.0     # cm => reading above => "cliff"
        self.obstacle_threshold = 12.0  # cm => reading below => obstacle

        # Set up a periodic timer to run inference (e.g., 2 Hz)
        self.rate_hz = 2.0
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.inference_loop)

        rospy.loginfo("RL Inference Node ready. Listening to sensors and publishing actions.")

    def bottom_cb(self, msg):
        data = [max(d, 0.0) for d in msg.data]  # clamp negative to 0
        if len(data) == 3:
            self.bottom_data = data

    def front_cb(self, msg):
        data = [max(d, 0.0) for d in msg.data]
        if len(data) == 2:
            self.front_data = data

    def inference_loop(self, event):
        """
        Timer callback that:
          1) Builds an observation from sensor data
          2) Calls the PPO model's predict()
          3) Publishes the corresponding motor command
          4) Applies simple safety checks (optional)
        """
        # 1) Construct observation array
        obs = self.build_observation()

        # 2) Model predict
        # If your model expects a different shape, e.g. (1, obs_dim), you might do np.array([obs])
        action_idx, _ = self.model.predict(obs, deterministic=True)

        # 3) Convert to motor command
        if action_idx not in ACTION_MAP:
            rospy.logwarn(f"Inference: action_idx {action_idx} not in ACTION_MAP, defaulting to STOP.")
            action_idx = 4  # STOP

        motor_cmd_val = ACTION_MAP[action_idx]

        # === Optional: safety override ===
        # If we see a "cliff" reading above threshold, forcibly STOP
        if self.is_cliff_detected():
            rospy.logwarn("Cliff detected! Overriding action => STOP.")
            motor_cmd_val = STOP_CMD

        # 4) Publish motor command
        self.cmd_pub.publish(Int32(motor_cmd_val))

    def build_observation(self):
        """
        Build a 1D array of floats that matches what the PPO model was trained on.
        If you used 5-sensor observations, keep that shape consistent.
        Example: [bottom_left, bottom_mid, bottom_right, front_left, front_right]
        """
        bleft, bmid, bright = self.bottom_data
        fleft, fright = self.front_data

        # Possibly clamp or normalize them if your training environment did so
        obs = np.array([
            min(bleft, 100.0),
            min(bmid,  100.0),
            min(bright,100.0),
            min(fleft, 100.0),
            min(fright,100.0)
        ], dtype=np.float32)

        # If your model was trained with shape (1,5), you may need:
        # obs = obs.reshape((1, -1))

        return obs

    def is_cliff_detected(self):
        """
        Returns True if any bottom sensor reading is above the cliff threshold.
        """
        for val in self.bottom_data:
            if val > self.cliff_threshold:
                return True
        return False


def main():
    try:
        node = RLInferenceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("RL Inference Node shutting down.")


if __name__ == "__main__":
    main()