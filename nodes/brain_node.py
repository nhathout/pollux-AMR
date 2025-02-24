#!/usr/bin/env python3

import os
import sys
import random
import rospy
from std_msgs.msg import Float32MultiArray, Int32

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

CLIFF_THRESHOLD = 7.0  # any reading above ~7 cm = potential cliff for your sensor
DISCONNECTED_VALUE = -1.0

class BrainNode:
    def __init__(self):
        rospy.init_node('brain_node', anonymous=True)
        rospy.loginfo("brain_node started. Subscribing to /pollux/ultrasonic")

        # Subscribe to ultrasonic Float32MultiArray
        self.sub = rospy.Subscriber('/pollux/ultrasonic', Float32MultiArray, self.ultrasonic_callback)

        # Publisher for motor commands
        self.cmd_pub = rospy.Publisher('/pollux/motor_cmd', Int32, queue_size=10)

        # If you need IMU data, similarly subscribe to /pollux/imu

    def ultrasonic_callback(self, msg):
        """
        msg.data is e.g. [dist_front_left, dist_front_right, dist_rear_left, dist_rear_right]
        or whichever order you're using. If one sensor is disconnected, it's -1.0.
        We'll just look for any sensor > CLIFF_THRESHOLD for a "cliff" scenario.
        """
        distances = msg.data
        # Check if any sensor is disconnected:
        disconnected_sensors = [i for i, d in enumerate(distances) if d == DISCONNECTED_VALUE]
        if disconnected_sensors:
            rospy.logwarn("Some sensor(s) might be disconnected: indices %s", disconnected_sensors)

        # We'll look for any reading above threshold. e.g. distances[1]
        # If you're only using the second sensor, index=1. Let's just check them all:
        cliff_detected = False
        for i, dist in enumerate(distances):
            # Ignore disconnected = -1.0
            if dist > CLIFF_THRESHOLD:
                rospy.loginfo("Cliff detected by sensor index %d with distance=%.2f!", i, dist)
                cliff_detected = True
                break

        if cliff_detected:
            # 1) Stop motors
            rospy.loginfo("Stopping motors first.")
            self.cmd_pub.publish(Int32(data=6))  # 6 => stop (we'll define new motor codes below)

            # 2) Turn randomly left or right
            direction = random.choice(['spin_left', 'spin_right'])
            if direction == 'spin_left':
                rospy.loginfo("Spinning left to avoid cliff.")
                self.cmd_pub.publish(Int32(data=4))  # 4 => spin left
            else:
                rospy.loginfo("Spinning right to avoid cliff.")
                self.cmd_pub.publish(Int32(data=5))  # 5 => spin right

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = BrainNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
