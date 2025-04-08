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

# === Motor command codes ===
FORWARD_CMD            = 0
BACKWARD_CMD           = 1
STOP_CMD               = 6
ROTATE_180_CMD         = 7
SPIN_ADJUST_LEFT_CMD   = 8
SPIN_ADJUST_RIGHT_CMD  = 9

# === Cliff detection constants ===
CLIFF_THRESHOLD        = 15.0
CLIFF_DEBOUNCE_TIME    = 10.0
BACKWARD_DURATION_MIN  = 3.0
BACKWARD_DURATION_MAX  = 4.0
ROTATE_180_DURATION    = 4.0

# === Front obstacle constants ===
OBSTACLE_THRESHOLD     = 18.0
OBSTACLE_DEBOUNCE_TIME = 3.0
SINGLE_SPIN_DURATION   = 2.0
BACKWARD_DURATION_OBST = 0.5

class UnifiedBrainNode:
    def __init__(self):
        rospy.init_node('unified_brain_node', anonymous=True)
        rospy.loginfo("Unified Brain Node started.")

        # Subscribers
        self.bottom_sub = rospy.Subscriber(
            '/pollux/ultrasonic_hw', 
            Float32MultiArray,
            self.bottom_sensors_callback
        )
        self.front_sub = rospy.Subscriber(
            '/pollux/ultrasonic_2',
            Float32MultiArray,
            self.front_sensors_callback
        )

        # Publisher for motor commands
        self.cmd_pub = rospy.Publisher('/pollux/motor_cmd', Int32, queue_size=10)

        # State
        self.last_cliff_time = 0.0
        self.last_front_time = 0.0
        self.in_action       = False
        self.last_motor_cmd  = None  # track last published motor cmd

        # Timer for “keep-alive” forward
        self.forward_timer = rospy.Timer(rospy.Duration(3.0), self.forward_timer_cb)

    # --------------------------------------------------------------------------
    # Helper: publish a motor command ONLY if different from last
    # --------------------------------------------------------------------------
    def send_cmd(self, cmd):
        if cmd != self.last_motor_cmd:
            self.cmd_pub.publish(Int32(data=cmd))
            self.last_motor_cmd = cmd

    # --------------------------------------------------------------------------
    def forward_timer_cb(self, event):
        """
        Periodic keep-alive timer. If not in an avoidance and we aren't
        already in forward, then we re-issue a forward command.
        """
        if not self.in_action and self.last_motor_cmd != FORWARD_CMD:
            rospy.loginfo("UnifiedBrain => Commanding forward motion (timer).")
            self.send_cmd(FORWARD_CMD)

    # --------------------------------------------------------------------------
    # BOTTOM SENSORS: detect “cliffs”
    # --------------------------------------------------------------------------
    def bottom_sensors_callback(self, msg):
        if self.in_action:
            return

        current_time = rospy.get_time()
        if (current_time - self.last_cliff_time) < CLIFF_DEBOUNCE_TIME:
            return

        # Convert to a list of floats in case there's any weird type
        distances = [float(x) for x in msg.data]  
        if len(distances) != 3:
            rospy.logwarn("UnifiedBrain => bottom_sensors: expected 3, got %d", len(distances))
            return

        # Check if any sensor sees a “cliff” distance > CLIFF_THRESHOLD
        cliff_detected = False
        for idx, dist_val in enumerate(distances):
            if dist_val < 0:
                continue  # invalid reading
            if dist_val > CLIFF_THRESHOLD:
                rospy.loginfo("Cliff detected on bottom sensor %d (%.1f cm)", idx, dist_val)
                cliff_detected = True
                break

        if not cliff_detected:
            return

        # --- Cliff avoidance routine ---
        self.in_action       = True
        self.last_cliff_time = current_time

        rospy.loginfo("UnifiedBrain => STOP (cliff detected)")
        self.send_cmd(STOP_CMD)  # only once
        rospy.sleep(0.5)

        back_time = random.uniform(BACKWARD_DURATION_OBST, 2.0)
        rospy.loginfo("UnifiedBrain => BACKWARD for %.1f s", back_time)
        self.send_cmd(BACKWARD_CMD)
        rospy.sleep(back_time)

        rospy.loginfo("UnifiedBrain => ROTATE 180° (%.1f s)", ROTATE_180_DURATION)
        self.send_cmd(ROTATE_180_CMD)
        rospy.sleep(ROTATE_180_DURATION)

        spin_count = random.randint(1, 2)
        for i in range(spin_count):
            cmd = random.choice([SPIN_ADJUST_LEFT_CMD, SPIN_ADJUST_RIGHT_CMD])
            secs = random.uniform(1.0, 3.0)
            rospy.loginfo("UnifiedBrain => spin %s for %.1f s",
                          "LEFT" if cmd == SPIN_ADJUST_LEFT_CMD else "RIGHT", secs)
            self.send_cmd(cmd)
            rospy.sleep(secs)

        rospy.loginfo("UnifiedBrain => done w/ cliff avoidance, resume forward")
        self.in_action = False
        self.send_cmd(FORWARD_CMD)  # once at the end

    # --------------------------------------------------------------------------
    # FRONT OBSTACLES
    # --------------------------------------------------------------------------
    def front_sensors_callback(self, msg):
        if self.in_action:
            return

        current_time = rospy.get_time()
        if (current_time - self.last_front_time) < OBSTACLE_DEBOUNCE_TIME:
            return

        # Force float type
        distances_2 = [float(x) for x in msg.data]
        if len(distances_2) < 2:
            rospy.logwarn("UnifiedBrain => front_sensors: expected 2, got %d", len(distances_2))
            return

        dist_left, dist_right = distances_2

        if dist_left < 0 or dist_right < 0:
            rospy.logwarn("UnifiedBrain => front sensor invalid. (L=%.2f, R=%.2f)", dist_left, dist_right)

        left_trigger  = (0 < dist_left  < OBSTACLE_THRESHOLD)
        right_trigger = (0 < dist_right < OBSTACLE_THRESHOLD)

        if not (left_trigger or right_trigger):
            return

        rospy.loginfo("Front obstacle. L=%.2f, R=%.2f", dist_left, dist_right)
        self.in_action       = True
        self.last_front_time = current_time

        rospy.loginfo("UnifiedBrain => STOP (front obstacle)")
        self.send_cmd(STOP_CMD)
        rospy.sleep(0.5)

        # BOTH front sensors => do 180 + random spin
        if left_trigger and right_trigger:
            back_time = random.uniform(BACKWARD_DURATION_MIN, BACKWARD_DURATION_MAX)
            rospy.loginfo("UnifiedBrain => BACKWARD for %.1f s", back_time)
            self.send_cmd(BACKWARD_CMD)
            rospy.sleep(back_time)

            rospy.loginfo("UnifiedBrain => ROTATE 180° for %.1f s", ROTATE_180_DURATION)
            self.send_cmd(ROTATE_180_CMD)
            rospy.sleep(ROTATE_180_DURATION)

            spin_count = random.randint(1, 2)
            for i in range(spin_count):
                cmd  = random.choice([SPIN_ADJUST_LEFT_CMD, SPIN_ADJUST_RIGHT_CMD])
                secs = random.uniform(2.0, 4.0)
                rospy.loginfo("UnifiedBrain => spin %s for %.1f s",
                              "LEFT" if cmd == SPIN_ADJUST_LEFT_CMD else "RIGHT", secs)
                self.send_cmd(cmd)
                rospy.sleep(secs)

        else:
            # single side => short spin away
            if left_trigger:
                rospy.loginfo("UnifiedBrain => SPIN RIGHT for %.1f s", SINGLE_SPIN_DURATION)
                self.send_cmd(SPIN_ADJUST_RIGHT_CMD)
                rospy.sleep(SINGLE_SPIN_DURATION)
            else:
                rospy.loginfo("UnifiedBrain => SPIN LEFT for %.1f s", SINGLE_SPIN_DURATION)
                self.send_cmd(SPIN_ADJUST_LEFT_CMD)
                rospy.sleep(SINGLE_SPIN_DURATION)

        rospy.loginfo("UnifiedBrain => Forward.")
        self.in_action = False
        self.send_cmd(FORWARD_CMD)

    # --------------------------------------------------------------------------
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = UnifiedBrainNode()
        node.run()
    except rospy.ROSInterruptException:
        pass