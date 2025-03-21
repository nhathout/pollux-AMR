#!/usr/bin/env python3

import os
import sys
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32

# If needed, dynamically add pollux-AMR/hardware to PYTHONPATH
SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR/hardware'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

SANITIZE_OFF        = 0
SANITIZE_ON         = 1
ROBOT_ON_DIM        = 2
ROBOT_ON_BRIGHT     = 3
INDICATOR_ON        = 4
INDICATOR_OFF       = 5

ANGULAR_VELOCITY_THRESHOLD = 50.0  # deg/sec
DEBOUNCE_DURATION = 3.0  # seconds

class LedGyroNode:
    def __init__(self):
        rospy.init_node('led_gyro_node', anonymous=True)
        rospy.loginfo("led_gyro_node started. Subscribing to /pollux/imu.")

        # Subscribe to IMU
        self.imu_sub = rospy.Subscriber('/pollux/imu', Imu, self.imu_callback)

        # Publisher for LED commands
        self.led_pub = rospy.Publisher('/pollux/led_cmd', Int32, queue_size=10)

        # State variables
        self.last_event_time = rospy.get_time()
        self.in_action = False

    def imu_callback(self, msg):
        """
        Monitor the angular velocity to see if we exceed some threshold,
        which might mean the robot is flipped, or being lifted abruptly, etc.
        Then we turn off the sanitization LED for safety.
        """
        # If we are currently 'in action', skip
        if self.in_action:
            return

        current_time = rospy.get_time()
        if (current_time - self.last_event_time) < DEBOUNCE_DURATION:
            return

        # Angular velocity from the IMU message (rad/s in standard ROS units)
        # If your IMU is in deg/s, adjust accordingly.
        # In standard ROS Imu: angular_velocity.x/y/z are in rad/s
        # Convert rad/s to deg/s:
        ax = msg.angular_velocity.x * (180.0 / 3.14159)
        ay = msg.angular_velocity.y * (180.0 / 3.14159)
        az = msg.angular_velocity.z * (180.0 / 3.14159)

        # Simple check: if any axis exceeds threshold => “flipped” or “big movement”
        if abs(ax) > ANGULAR_VELOCITY_THRESHOLD or \
           abs(ay) > ANGULAR_VELOCITY_THRESHOLD or \
           abs(az) > ANGULAR_VELOCITY_THRESHOLD:
            rospy.logwarn("Significant flip/motion detected! Shutting off sanitization LED.")
            self.in_action = True
            self.last_event_time = current_time

            # Turn OFF sanitization LED
            self.led_pub.publish(Int32(data=SANITIZE_OFF))

            # Wait a bit before re-allowing new events
            rospy.sleep(1.0)
            self.in_action = False

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = LedGyroNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
