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

# Motor command codes (must match motor_cmd_node.py):
FORWARD_CMD           = 0  # continuously move forward
BACKWARD_CMD          = 1  # move backward
STOP_CMD              = 6  # stop motors
ROTATE_180_CMD        = 7  # rotate 180° (full turn)
SPIN_ADJUST_LEFT_CMD  = 8  # slight spin left (~20°)
SPIN_ADJUST_RIGHT_CMD = 9  # slight spin right (~20°)

# Parameters
OBSTACLE_THRESHOLD = 18.0  # cm; if a sensor is under this distance => obstacle
DEBOUNCE_DURATION  = 3.0   # ignore subsequent triggers for 3 s after one is handled
ACTION_DURATION    = 2.0   # how long to back up or spin away

class UltrasonicBrainNode:
    def __init__(self):
        rospy.init_node('ultrasonic_brain_node', anonymous=True)
        rospy.loginfo("ultrasonic_brain_node started. Subscribing to /pollux/ultrasonic_2")

        # Subscribe to the new 2-sensor ultrasonic topic
        self.ultra_sub_2 = rospy.Subscriber('/pollux/ultrasonic_2', Float32MultiArray,
                                            self.ultrasonic_2_callback)

        # Publish motor commands
        self.cmd_pub = rospy.Publisher('/pollux/motor_cmd', Int32, queue_size=10)

        # State
        self.last_event_time = 0.0
        self.in_action = False

    def ultrasonic_2_callback(self, msg):
        # If we're currently in an avoidance maneuver, ignore new triggers
        if self.in_action:
            return

        current_time = rospy.get_time()
        if (current_time - self.last_event_time) < DEBOUNCE_DURATION:
            # We haven't waited long enough since the last obstacle
            return

        distances_2 = msg.data  # [dist_left, dist_right]
        if len(distances_2) < 2:
            rospy.logwarn("ultrasonic_brain_node: unexpected data size (need 2).")
            return

        dist_left, dist_right = distances_2
        # If -1.0 => invalid sensor reading. You can handle this as you see fit:
        if dist_left < 0 or dist_right < 0:
            rospy.logwarn("One of the front ultrasonic sensors is invalid. (left=%.2f, right=%.2f)",
                          dist_left, dist_right)

        # Check if either sensor is below the OBSTACLE_THRESHOLD
        obstacle_detected = False
        if (0 < dist_left  < OBSTACLE_THRESHOLD) or \
           (0 < dist_right < OBSTACLE_THRESHOLD):
            obstacle_detected = True

        if obstacle_detected:
            rospy.loginfo("Front obstacle detected. L=%.2f cm, R=%.2f cm",
                          dist_left, dist_right)
            self.last_event_time = current_time
            self.in_action = True

            # 1) Stop
            rospy.loginfo("Brain => STOP motors.")
            self.cmd_pub.publish(Int32(data=STOP_CMD))
            rospy.sleep(0.5)

            # 2) Move backward for ACTION_DURATION seconds
            rospy.loginfo("Brain => Move backward for ~%.1f s", ACTION_DURATION)
            self.cmd_pub.publish(Int32(data=BACKWARD_CMD))
            rospy.sleep(ACTION_DURATION)

            # 3) Turn away from the closer side
            if dist_left < dist_right:
                rospy.loginfo("Brain => Turn RIGHT for ~%.1f s", ACTION_DURATION)
                self.cmd_pub.publish(Int32(data=SPIN_ADJUST_RIGHT_CMD))
            else:
                rospy.loginfo("Brain => Turn LEFT for ~%.1f s", ACTION_DURATION)
                self.cmd_pub.publish(Int32(data=SPIN_ADJUST_LEFT_CMD))

            rospy.sleep(ACTION_DURATION)

            # 4) Resume forward
            rospy.loginfo("Brain => Forward.")
            self.cmd_pub.publish(Int32(data=FORWARD_CMD))

            # Done
            self.in_action = False

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = UltrasonicBrainNode()
        node.run()
    except rospy.ROSInterruptException:
        pass