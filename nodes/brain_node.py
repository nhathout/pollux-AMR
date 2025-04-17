#!/usr/bin/env python3

"""
brain_node.py monitors ultrasonic cliff sensors and issues motor commands to avoid falls.
It commands forward motion during idle states and triggers a backward->rotate->redirect sequence
when a potential cliff is detected. Actions depend on /pollux/motor_cmd and /pollux/ultrasonic_hw topics
"""

import os
import sys
import time
import random
import rospy
from std_msgs.msg import Float32MultiArray, Int32

# Dynamically add pollux-AMR folder to PYTHONPATH so we can import shared hardware modules.
SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

# Parameters and command codes (adjust as needed)
CLIFF_THRESHOLD = 15.0        # cm; readings above this indicate a potential cliff
DEBOUNCE_DURATION = 10.0      # seconds to ignore new cliff triggers after one is handled

# Motor command codes (must match what motor_cmd_node.py expects):
FORWARD_CMD            = 0  # continuously move forward
BACKWARD_CMD           = 1  # move backward
STOP_CMD               = 6  # stop motors
ROTATE_180_CMD         = 7  # rotate 180° (full turn)
SPIN_ADJUST_LEFT_CMD   = 8  # slight spin left (~20°)
SPIN_ADJUST_RIGHT_CMD  = 9  # slight spin right (~20°)

# Some baseline durations (in seconds) — we will randomize these below
BACKWARD_DURATION = 3.0      # was fixed; now we’ll randomize around this
ROTATE_DURATION   = 3.0
ADJUST_DURATION   = 1.5      # was fixed; we’ll randomize it for variety

class BrainNode:
    def __init__(self):
        rospy.init_node('brain_node', anonymous=True)
        rospy.loginfo("brain_node started. Subscribing to /pollux/ultrasonic_hw")
        
        # Subscribe to ultrasonic sensor data
        self.ultra_sub = rospy.Subscriber('/pollux/ultrasonic_hw', Float32MultiArray, self.ultrasonic_callback)
        # Publisher for motor commands
        self.cmd_pub = rospy.Publisher('/pollux/motor_cmd', Int32, queue_size=10)
        
        # State variables
        self.last_event_time = 0.0
        self.in_action = False
        
        # Timer to command forward motion periodically when idle
        self.forward_timer = rospy.Timer(rospy.Duration(2.0), self.forward_timer_cb)
    
    def forward_timer_cb(self, event):
        # If not currently in an avoidance action, command forward motion.
        if not self.in_action:
            rospy.loginfo("Brain => Commanding forward motion.")
            self.cmd_pub.publish(Int32(data=FORWARD_CMD))
    
    def ultrasonic_callback(self, msg):
        if self.in_action:
            return

        current_time = rospy.get_time()
        if current_time - self.last_event_time < DEBOUNCE_DURATION:
            return

        distances = list(msg.data)
        if len(distances) != 3:
            rospy.logwarn("brain_node: expected 3 bottom sensors, got %d", len(distances))
            return

        # 0=left, 1=mid, 2=right
        cliff_detected = False
        for idx, d in enumerate(distances):
            if d < 0:        # sensor error
                continue
            if d > CLIFF_THRESHOLD:
                rospy.loginfo("Cliff on bottom sensor %d (%.1f cm)", idx, d)
                cliff_detected = True
                break

        if not cliff_detected:
            return

        # ---- avoidance sequence (unchanged logic) ----
        self.last_event_time = current_time
        self.in_action       = True

        rospy.loginfo("Brain => STOP")
        self.cmd_pub.publish(Int32(data=STOP_CMD))
        rospy.sleep(0.5)

        back_time = random.uniform(1.0, 2.0)
        rospy.loginfo("Brain => BACKWARD %.1fs", back_time)
        self.cmd_pub.publish(Int32(data=BACKWARD_CMD))
        rospy.sleep(back_time)

        rospy.loginfo("Brain => ROTATE 180° %.1fs", ROTATE_DURATION)
        self.cmd_pub.publish(Int32(data=ROTATE_180_CMD))
        rospy.sleep(ROTATE_DURATION)

        for spin_idx in range(random.randint(1, 2)):
            cmd  = random.choice([SPIN_ADJUST_LEFT_CMD, SPIN_ADJUST_RIGHT_CMD])
            secs = random.uniform(2.0, 4.0)
            rospy.loginfo("Brain => spin %s %.1fs", "L" if cmd==SPIN_ADJUST_LEFT_CMD else "R", secs)
            self.cmd_pub.publish(Int32(data=cmd))
            rospy.sleep(secs)

        rospy.loginfo("Brain => done, resume forward")
        self.in_action = False

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = BrainNode()
        node.run()
    except rospy.ROSInterruptException:
        pass