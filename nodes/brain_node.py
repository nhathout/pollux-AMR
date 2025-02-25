#!/usr/bin/env python3

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
DEBOUNCE_DURATION = 5.0         # seconds to ignore new cliff triggers after one is handled

# Motor command codes (must match what motor_cmd_node.py expects):
FORWARD_CMD = 0               # continuously move forward
BACKWARD_CMD = 1              # move backward
STOP_CMD = 6                  # stop motors
ROTATE_180_CMD = 7            # rotate 180° (full turn)
SPIN_ADJUST_LEFT_CMD = 8      # slight spin left (e.g. 20°)
SPIN_ADJUST_RIGHT_CMD = 9     # slight spin right (e.g. 20°)

# Durations for each action (in seconds)
BACKWARD_DURATION = 3.0       # duration to move backward
ROTATE_DURATION = 3.0         # duration for 180° rotation
ADJUST_DURATION = 1.5         # duration for slight random adjustment

class BrainNode:
    def __init__(self):
        rospy.init_node('brain_node', anonymous=True)
        rospy.loginfo("brain_node started. Subscribing to /pollux/ultrasonic")
        
        # Subscribe to ultrasonic sensor data
        self.ultra_sub = rospy.Subscriber('/pollux/ultrasonic', Float32MultiArray, self.ultrasonic_callback)
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
        # Do nothing if already handling an avoidance sequence.
        if self.in_action:
            return
        
        current_time = rospy.get_time()
        # Debounce: if a cliff was handled recently, ignore new triggers.
        if current_time - self.last_event_time < DEBOUNCE_DURATION:
            return
        
        distances = msg.data
        cliff_detected = False
        for i, d in enumerate(distances):
            if d < 0:
                continue  # sensor disconnected
            if d > CLIFF_THRESHOLD:
                rospy.loginfo("Cliff detected on sensor %d: %.2f cm", i, d)
                cliff_detected = True
                break
        
        if cliff_detected:
            self.last_event_time = current_time
            self.in_action = True
            
            rospy.loginfo("Brain => STOP motors.")
            self.cmd_pub.publish(Int32(data=STOP_CMD))
            rospy.sleep(0.5)
            
            rospy.loginfo("Brain => Moving backward for %.1f seconds.", BACKWARD_DURATION)
            self.cmd_pub.publish(Int32(data=BACKWARD_CMD))
            rospy.sleep(BACKWARD_DURATION)
            
            rospy.loginfo("Brain => Rotating 180° for %.1f seconds.", ROTATE_DURATION)
            self.cmd_pub.publish(Int32(data=ROTATE_180_CMD))
            rospy.sleep(ROTATE_DURATION)
            
            # Perform a slight random adjustment: spin left or right by a small angle.
            adjust_cmd = random.choice([SPIN_ADJUST_LEFT_CMD, SPIN_ADJUST_RIGHT_CMD])
            direction_str = "left" if adjust_cmd == SPIN_ADJUST_LEFT_CMD else "right"
            rospy.loginfo("Brain => Adjusting spin %s for %.1f seconds.", direction_str, ADJUST_DURATION)
            self.cmd_pub.publish(Int32(data=adjust_cmd))
            rospy.sleep(ADJUST_DURATION)
            
            rospy.loginfo("Brain => Resuming forward motion.")
            self.in_action = False

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = BrainNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
