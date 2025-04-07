#!/usr/bin/env python3

import os
import sys
import time
import random
import rospy
from std_msgs.msg import Float32MultiArray, Int32

# If needed, adjust PYTHONPATH to find your hardware modules
SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

# Motor command codes (match motor_cmd_node.py):
FORWARD_CMD = 0               # continuously move forward
BACKWARD_CMD = 1              # move backward
STOP_CMD = 6                  # stop motors
ROTATE_180_CMD = 7            # rotate 180° (full turn)
SPIN_ADJUST_LEFT_CMD = 8      # slight spin left (e.g. 20°)
SPIN_ADJUST_RIGHT_CMD = 9     # slight spin right (e.g. 20°)

# Parameters
OBSTACLE_THRESHOLD = 15.0  # cm; anything closer means “obstacle” detected
DEBOUNCE_DURATION  = 3.0   # ignore new triggers for a while after one is handled
ACTION_DURATION    = 2.0   # how long to back away or turn

class UltrasonicBrainNode:
    def __init__(self):
        rospy.init_node('ultrasonic_brain_node', anonymous=True)
        rospy.loginfo("ultrasonic_brain_node started. Subscribing to /pollux/ultrasonic_2")

        # Subscribe to the new 2-sensor ultrasonic topic
        self.ultra_sub_2 = rospy.Subscriber('/pollux/ultrasonic_2', Float32MultiArray, self.ultrasonic_2_callback)

        # Publish motor commands
        self.cmd_pub = rospy.Publisher('/pollux/motor_cmd', Int32, queue_size=10)

        # State
        self.last_event_time = 0.0
        self.in_action = False

    def ultrasonic_2_callback(self, msg):
        # If already handling an obstacle, ignore
        if self.in_action:
            return

        current_time = rospy.get_time()
        if (current_time - self.last_event_time) < DEBOUNCE_DURATION:
            return

        # We expect msg.data to have length = 2 ([left, right])
        # In your code, verify length to avoid index errors
        distances_2 = msg.data  # e.g. [dist_left, dist_right]
        if len(distances_2) < 2:
            rospy.logwarn("Unexpected ultrasonic_2 data size.")
            return

        dist_left, dist_right = distances_2
        # If -1.0 means sensor disconnected, handle that as well
        if dist_left < 0 or dist_right < 0:
            rospy.logwarn("One of the new ultrasonic sensors is disconnected or returning invalid data.")

        # Check for obstacle if distance < threshold
        obstacle_detected = False
        if 0 < dist_left < OBSTACLE_THRESHOLD or 0 < dist_right < OBSTACLE_THRESHOLD:
            obstacle_detected = True

        if obstacle_detected:
            rospy.loginfo("Obstacle detected by new sensors. Distances: L=%.2f cm, R=%.2f cm", dist_left, dist_right)
            self.last_event_time = current_time
            self.in_action = True

            # Example: stop, then back up, then randomly choose left or right turn
            rospy.loginfo("Brain => STOP motors.")
            self.cmd_pub.publish(Int32(data=STOP_CMD))
            rospy.sleep(0.5)

            rospy.loginfo("Brain => Move backward ~%.1f s", ACTION_DURATION)
            self.cmd_pub.publish(Int32(data=BACKWARD_CMD))
            rospy.sleep(ACTION_DURATION)

            # Then turn a bit away from the obstacle
            if dist_left < dist_right:
                rospy.loginfo("Brain => Turn RIGHT ~%.1f s", ACTION_DURATION)
                self.cmd_pub.publish(Int32(data=TURN_RIGHT_CMD))
            else:
                rospy.loginfo("Brain => Turn LEFT ~%.1f s", ACTION_DURATION)
                self.cmd_pub.publish(Int32(data=TURN_LEFT_CMD))

            rospy.sleep(ACTION_DURATION)

            # Resume forward
            rospy.loginfo("Brain => Forward.")
            self.cmd_pub.publish(Int32(data=FORWARD_CMD))

            # Done handling obstacle
            self.in_action = False

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = UltrasonicBrainNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
