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

CLIFF_THRESHOLD = 10.0       # cm - anything above => treat as cliff
CLIFF_DEBOUNCE = 3.0         # seconds - ignore new cliff triggers for a bit
SPIN_DURATION = 3.0          # spin for 3s
FORWARD_CMD = 0              # from motor_cmd_node.py (0 => forward)
STOP_CMD = 6                 # from motor_cmd_node.py (6 => stop)
SPIN_LEFT_CMD = 4            # spin left
SPIN_RIGHT_CMD = 5           # spin right

class BrainNode:
    """
    State Machine:
      - FORWARD: continuously command forward unless we detect a cliff
      - SPINNING: we do a spin for SPIN_DURATION, ignore new cliffs
    """
    def __init__(self):
        rospy.init_node('brain_node', anonymous=True)
        rospy.loginfo("brain_node started. Subscribing to /pollux/ultrasonic")

        # Subscribe to ultrasonic Float32MultiArray
        self.sub = rospy.Subscriber('/pollux/ultrasonic', Float32MultiArray, self.ultrasonic_callback)
        # Publisher for motor commands
        self.cmd_pub = rospy.Publisher('/pollux/motor_cmd', Int32, queue_size=10)

        # State variables
        self.state = "FORWARD"
        self.last_cliff_time = 0.0
        self.spin_start_time = 0.0

        # Create a timer to periodically publish forward if in FORWARD state
        # e.g., every 2 seconds
        self.forward_timer = rospy.Timer(rospy.Duration(2.0), self.forward_timer_cb)

    def forward_timer_cb(self, event):
        """Called every 2s. If we're in FORWARD state, publish 'move forward'."""
        if self.state == "FORWARD":
            rospy.loginfo("Brain => Move forward!")
            self.cmd_pub.publish(Int32(data=FORWARD_CMD))

    def ultrasonic_callback(self, msg):
        distances = msg.data

        # Check disconnected:
        disconnected_sensors = [i for i, d in enumerate(distances) if d < 0]
        if disconnected_sensors:
            rospy.logwarn("Some sensor(s) might be disconnected: indices %s", disconnected_sensors)

        # If currently SPINNING, see if spin_time is done
        if self.state == "SPINNING":
            elapsed = rospy.get_time() - self.spin_start_time
            if elapsed >= SPIN_DURATION:
                rospy.loginfo("Spin complete. Switching to FORWARD state.")
                self.state = "FORWARD"
            return  # ignore new cliff triggers while spinning

        # If in FORWARD state, check for cliff
        # Also check "debounce" -> if less than CLIFF_DEBOUNCE since last cliff
        if self.state == "FORWARD":
            if (rospy.get_time() - self.last_cliff_time) < CLIFF_DEBOUNCE:
                # still in a cooldown; ignore
                return

            # Detect cliff
            cliff_detected = False
            for i, dist in enumerate(distances):
                # ignore disconnected sensors
                if dist < 0:
                    continue
                if dist > CLIFF_THRESHOLD:
                    rospy.loginfo("Cliff detected by sensor %d with dist=%.2f", i, dist)
                    cliff_detected = True
                    break

            if cliff_detected:
                self.last_cliff_time = rospy.get_time()
                # Stop
                rospy.loginfo("Brain => STOP motors!")
                self.cmd_pub.publish(Int32(data=STOP_CMD))
                # Random spin
                direction = random.choice([SPIN_LEFT_CMD, SPIN_RIGHT_CMD])
                if direction == SPIN_LEFT_CMD:
                    rospy.loginfo("Brain => Spin LEFT for %.1fs", SPIN_DURATION)
                else:
                    rospy.loginfo("Brain => Spin RIGHT for %.1fs", SPIN_DURATION)

                self.cmd_pub.publish(Int32(data=direction))
                # Switch to SPINNING
                self.state = "SPINNING"
                self.spin_start_time = rospy.get_time()

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = BrainNode()
        node.run()
    except rospy.ROSInterruptException:
        pass