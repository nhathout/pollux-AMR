#!/usr/bin/env python3

import os
import sys
import time
import random
import rospy
from std_msgs.msg import Float32MultiArray, Int32

# If needed, adjust PYTHONPATH
SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

# Motor command codes
FORWARD_CMD            = 0
BACKWARD_CMD           = 1
STOP_CMD               = 6
ROTATE_180_CMD         = 7
SPIN_ADJUST_LEFT_CMD   = 8
SPIN_ADJUST_RIGHT_CMD  = 9

# Cliff Detection
CLIFF_THRESHOLD        = 15.0
CLIFF_DEBOUNCE_TIME    = 10.0
CLIFF_BACK_SECS_EACH   = 2.0  # 2 x 2 = 4s total backward
ROTATE_180_DURATION    = 4.0

# Front Obstacle
OBSTACLE_THRESHOLD     = 20.0  # <== Bumped up to 25 so you see triggers earlier
OBSTACLE_DEBOUNCE_TIME = 3.0
SINGLE_SPIN_DURATION   = 2.0
BACKWARD_DURATION_OBST = 1.0

class UnifiedBrainNode:
    def __init__(self):
        rospy.init_node('unified_brain_node', anonymous=True)
        rospy.loginfo("Unified Brain Node started.")

        # Subscribers
        self.bottom_sub = rospy.Subscriber('/pollux/ultrasonic_hw',
                                           Float32MultiArray,
                                           self.bottom_sensors_callback)
        self.front_sub = rospy.Subscriber('/pollux/ultrasonic_2',
                                          Float32MultiArray,
                                          self.front_sensors_callback)

        # Motor Publisher
        self.cmd_pub = rospy.Publisher('/pollux/motor_cmd', Int32, queue_size=10)

        # Internal State
        self.last_cliff_time = 0.0
        self.last_front_time = 0.0
        self.in_action       = False   # "busy" avoiding something
        self.last_cmd        = None    # track last motor cmd to avoid re-issues

        # Timer to occasionally re-send FORWARD (only if idle)
        self.forward_timer = rospy.Timer(rospy.Duration(5.0), self.forward_timer_cb)

    # --------------------------------------------------------------------------
    # Helper: Publish a motor command, but only if different from last
    # --------------------------------------------------------------------------
    def send_cmd(self, cmd):
        if cmd != self.last_cmd:
            self.cmd_pub.publish(Int32(data=cmd))
            self.last_cmd = cmd

    # --------------------------------------------------------------------------
    # Called every 5s to keep the robot going forward if it's not in avoidance
    # --------------------------------------------------------------------------
    def forward_timer_cb(self, event):
        if not self.in_action and self.last_cmd != FORWARD_CMD:
            rospy.loginfo("UnifiedBrain => Keep-alive: forward.")
            self.send_cmd(FORWARD_CMD)

    # --------------------------------------------------------------------------
    # CLIFF DETECTION
    # --------------------------------------------------------------------------
    def bottom_sensors_callback(self, msg):
        if self.in_action:
            return

        current_time = rospy.get_time()
        if (current_time - self.last_cliff_time) < CLIFF_DEBOUNCE_TIME:
            return

        distances = list(msg.data)
        if len(distances) != 3:
            rospy.logwarn("bottom_sensors: expected 3, got %d", len(distances))
            return

        # Check if any sensor sees a "cliff" (> CLIFF_THRESHOLD)
        cliff_found = False
        for idx, dist_val in enumerate(distances):
            if dist_val < 0:
                continue
            if dist_val > CLIFF_THRESHOLD:
                rospy.loginfo("Cliff on bottom sensor %d (%.1f cm)", idx, dist_val)
                cliff_found = True
                break

        if not cliff_found:
            return

        # Start Cliff Avoidance
        self.in_action        = True
        self.last_cliff_time  = current_time

        rospy.loginfo("Cliff Avoid => STOP")
        self.send_cmd(STOP_CMD)
        rospy.sleep(0.5)

        # Double backward => ~4s
        for i in range(2):
            rospy.loginfo(f"Cliff Avoid => BACKWARD pass {i+1} for {CLIFF_BACK_SECS_EACH} s")
            self.send_cmd(BACKWARD_CMD)
            rospy.sleep(CLIFF_BACK_SECS_EACH)

        # 180° rotate
        rospy.loginfo(f"Cliff Avoid => ROTATE 180° for {ROTATE_180_DURATION} s")
        self.send_cmd(ROTATE_180_CMD)
        rospy.sleep(ROTATE_180_DURATION)

        # Single optional spin
        spin_count = random.randint(0, 1)
        for i in range(spin_count):
            cmd  = random.choice([SPIN_ADJUST_LEFT_CMD, SPIN_ADJUST_RIGHT_CMD])
            secs = random.uniform(1.0, 2.0)
            rospy.loginfo(f"Cliff Avoid => spin {cmd} for {secs:.1f} s")
            self.send_cmd(cmd)
            rospy.sleep(secs)

        rospy.loginfo("Cliff Avoid => done, forward + 1s cooldown.")
        self.send_cmd(FORWARD_CMD)
        rospy.sleep(1.0)

        self.in_action = False

    # --------------------------------------------------------------------------
    # FRONT OBSTACLES
    # --------------------------------------------------------------------------
    def front_sensors_callback(self, msg):
        if self.in_action:
            return

        current_time = rospy.get_time()
        if (current_time - self.last_front_time) < OBSTACLE_DEBOUNCE_TIME:
            return

        distances_2 = list(msg.data)
        if len(distances_2) < 2:
            rospy.logwarn("front_sensors: expected 2, got %d", len(distances_2))
            return

        dist_left, dist_right = distances_2

        # e.g., [12.0, 30.5]
        left_trigger  = (0 < dist_left  < OBSTACLE_THRESHOLD)
        right_trigger = (0 < dist_right < OBSTACLE_THRESHOLD)
        if not (left_trigger or right_trigger):
            return  # no obstacle

        # Mark in action
        self.in_action       = True
        self.last_front_time = current_time

        rospy.loginfo("Front Obstacle => STOP")
        self.send_cmd(STOP_CMD)
        rospy.sleep(0.5)

        # If BOTH triggered => do 180
        if left_trigger and right_trigger:
            rospy.loginfo(f"Both front sensors => backward {BACKWARD_DURATION_OBST}s then 180°")
            self.send_cmd(BACKWARD_CMD)
            rospy.sleep(BACKWARD_DURATION_OBST)

            rospy.loginfo("Obstacle Avoid => rotate 180°")
            self.send_cmd(ROTATE_180_CMD)
            rospy.sleep(ROTATE_180_DURATION)

            # If you want no extra spins here, skip them:
            # spin_count = random.randint(0, 1)
            # for i in range(spin_count):
            #    ...
        else:
            # Single side => spin away
            if left_trigger:
                # obstacle on left => spin right
                rospy.loginfo(f"Obstacle left => spin RIGHT for {SINGLE_SPIN_DURATION}s")
                self.send_cmd(SPIN_ADJUST_RIGHT_CMD)
                rospy.sleep(SINGLE_SPIN_DURATION)
            else:
                # obstacle on right => spin left
                rospy.loginfo(f"Obstacle right => spin LEFT for {SINGLE_SPIN_DURATION}s")
                self.send_cmd(SPIN_ADJUST_LEFT_CMD)
                rospy.sleep(SINGLE_SPIN_DURATION)

        rospy.loginfo("Obstacle Avoid => forward + 1s cooldown.")
        self.send_cmd(FORWARD_CMD)

        rospy.sleep(1.0)
        self.in_action = False

    # --------------------------------------------------------------------------
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = UnifiedBrainNode()
        node.run()
    except rospy.ROSInterruptException:
        pass