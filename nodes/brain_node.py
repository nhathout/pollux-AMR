#!/usr/bin/env python3
import os
import sys
import time
import random
import rospy
from std_msgs.msg import Float32MultiArray, Int32

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

CLIFF_THRESHOLD = 10.0  # Increase to 10cm or a height you prefer
DEBOUNCE_DURATION = 3.0  # 3 seconds

class BrainNode:
    def __init__(self):
        rospy.init_node('brain_node', anonymous=True)
        rospy.loginfo("brain_node started. Subscribing to /pollux/ultrasonic")

        self.sub = rospy.Subscriber('/pollux/ultrasonic', Float32MultiArray, self.ultrasonic_callback)
        self.cmd_pub = rospy.Publisher('/pollux/motor_cmd', Int32, queue_size=10)

        # Store time of last cliff event
        self.last_cliff_time = 0.0

    def ultrasonic_callback(self, msg):
        distances = msg.data
        # Check disconnected:
        disconnected_sensors = [i for i, d in enumerate(distances) if d < 0]
        if disconnected_sensors:
            rospy.logwarn("Some sensor(s) might be disconnected: indices %s", disconnected_sensors)

        # If we triggered a cliff event recently, skip
        if (rospy.get_time() - self.last_cliff_time) < DEBOUNCE_DURATION:
            return

        # Check if any sensor is above threshold
        cliff_detected = False
        for i, dist in enumerate(distances):
            if dist > CLIFF_THRESHOLD:  # e.g. 10cm
                rospy.loginfo("Cliff detected by sensor index %d with distance=%.2f!", i, dist)
                cliff_detected = True
                break

        if cliff_detected:
            self.last_cliff_time = rospy.get_time()  # Record event time
            rospy.loginfo("Stopping motors first.")
            self.cmd_pub.publish(Int32(data=6))  # 6 => stop
            # Random spin
            direction = random.choice([4, 5])  # 4 => spin_left, 5 => spin_right
            rospy.loginfo("Spinning %s to avoid cliff.", "left" if direction == 4 else "right")
            self.cmd_pub.publish(Int32(data=direction))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = BrainNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
