import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class PolluxEnv(gym.Env):
    def __init__(self):
        super(PolluxEnv, self).__init__()
        # Initialize ROS node
        rospy.init_node('pollux_env', anonymous=True)

        # Define action and observation spaces
        self.action_space = spaces.Box(low=np.array([-0.5, -1.0]), high=np.array([0.5, 1.0]), dtype=np.float32)
        self.observation_space = spaces.Box(low=0.0, high=2.0, shape=(4,), dtype=np.float32)

        # Publishers and Subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.range_subs = []
        self.state = np.zeros(4)

        sensor_topics = ['/hc_sr04_front_left', '/hc_sr04_front_right', '/hc_sr04_rear_left', '/hc_sr04_rear_right']
        for i, topic in enumerate(sensor_topics):
            rospy.Subscriber(topic, Range, self.range_callback, callback_args=i)

    def range_callback(self, data, idx):
        self.state[idx] = data.range

    def step(self, action):
        # Publish action to cmd_vel
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        self.cmd_vel_pub.publish(vel_cmd)

        # Wait for sensor updates
        rospy.sleep(0.1)

        # Calculate reward
        reward = self.calculate_reward()

        # Determine if episode is done
        done = self.is_done()

        # Return observation, reward, done, info
        return self.state.copy(), reward, done, {}

    def reset(self):
        # Reset simulation environment
        # Optionally, use a ROS service to reset the simulation
        self.state = np.zeros(4)
        return self.state.copy()

    def calculate_reward(self):
        # Implement your reward function
        reward = 0
        # Negative reward if any sensor detects a cliff
        if np.any(self.state < 0.1):
            reward -= 10
        else:
            reward += 1  # Reward for moving safely
        return reward

    def is_done(self):
        # Termination conditions
        done = False
        if np.any(self.state < 0.05):
            done = True
        return done

    def render(self, mode='human'):
        pass

    def close(self):
        pass
