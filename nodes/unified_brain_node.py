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

# Motor command codes (must match motor_cmd_node.py)
FORWARD_CMD            = 0
BACKWARD_CMD           = 1
STOP_CMD               = 6
ROTATE_180_CMD         = 7
SPIN_ADJUST_LEFT_CMD   = 8
SPIN_ADJUST_RIGHT_CMD  = 9

# Cliff (bottom) Detection Constants
CLIFF_THRESHOLD        = 15.0  # cm => reading above => "cliff"
CLIFF_DEBOUNCE_TIME    = 10.0  # ignore new cliff triggers for 10 s
ROTATE_180_DURATION    = 4.0

# Instead of random back time for a single backward, we’ll do 2 repeated BACKWARD commands
CLIFF_BACK_SECS_EACH   = 2.0  # each backward command runs for 2 seconds, done twice => ~4s total

# Front Obstacle Constants
OBSTACLE_THRESHOLD     = 18.0
OBSTACLE_DEBOUNCE_TIME = 3.0
SINGLE_SPIN_DURATION   = 2.0
BACKWARD_DURATION_OBST = 1.0  # do multiple commands or a bigger backward if you want

class UnifiedBrainNode:
    """
    Merged node:
      • Bottom ultrasonic (cliffs) => highest priority
      • Front ultrasonic (obstacles) => second priority
      • Publishes motor commands to /pollux/motor_cmd
    """
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

        # Motor publisher
        self.cmd_pub = rospy.Publisher('/pollux/motor_cmd', Int32, queue_size=10)

        # State
        self.last_cliff_time = 0.0
        self.last_front_time = 0.0
        self.in_action       = False

        # Timer for forward keep-alive (only if not in avoidance)
        self.forward_timer = rospy.Timer(rospy.Duration(5.0), self.forward_timer_cb)

    # --------------------------------------------------------------------------
    def forward_timer_cb(self, event):
        if not self.in_action:
            rospy.loginfo("UnifiedBrain => Commanding forward motion.")
            self.cmd_pub.publish(FORWARD_CMD)

    # --------------------------------------------------------------------------
    # CLIFF DETECTION (highest priority)
    # --------------------------------------------------------------------------
    def bottom_sensors_callback(self, msg):
        if self.in_action:
            return

        current_time = rospy.get_time()
        # Debounce
        if (current_time - self.last_cliff_time) < CLIFF_DEBOUNCE_TIME:
            return

        distances = list(msg.data)
        if len(distances) != 3:
            rospy.logwarn("UnifiedBrain => bottom_sensors: expected 3, got %d", len(distances))
            return

        # If any sensor sees distance > CLIFF_THRESHOLD => cliff
        cliff_detected = False
        for idx, dist_val in enumerate(distances):
            if dist_val < 0:
                continue
            if dist_val > CLIFF_THRESHOLD:
                rospy.loginfo("Cliff detected on bottom sensor %d (%.1f cm)", idx, dist_val)
                cliff_detected = True
                break

        if not cliff_detected:
            return

        # Start cliff avoidance
        self.in_action = True
        self.last_cliff_time = current_time

        rospy.loginfo("UnifiedBrain => STOP (cliff detected)")
        self.cmd_pub.publish(STOP_CMD)
        rospy.sleep(0.5)

        # Instead of random back time, do multiple BACKWARD commands => ~4s total
        # This effectively "doubles" the backward distance.
        for i in range(2):
            rospy.loginfo(f"UnifiedBrain => BACKWARD pass {i+1} for {CLIFF_BACK_SECS_EACH:.1f} s")
            self.cmd_pub.publish(BACKWARD_CMD)
            rospy.sleep(CLIFF_BACK_SECS_EACH)

        # 180° rotate
        rospy.loginfo("UnifiedBrain => ROTATE 180° for %.1f s", ROTATE_180_DURATION)
        self.cmd_pub.publish(ROTATE_180_CMD)
        rospy.sleep(ROTATE_180_DURATION)

        # (Optional) random spin once at most
        # If you find repeated spins cause you to get "stuck," limit to spin_count=0 or 1
        spin_count = random.randint(0, 1)  # 0 or 1
        for i in range(spin_count):
            cmd  = random.choice([SPIN_ADJUST_LEFT_CMD, SPIN_ADJUST_RIGHT_CMD])
            secs = random.uniform(1.0, 2.0)
            rospy.loginfo("UnifiedBrain => spin %s for %.1f s",
                          "LEFT" if cmd == SPIN_ADJUST_LEFT_CMD else "RIGHT", secs)
            self.cmd_pub.publish(cmd)
            rospy.sleep(secs)

        rospy.loginfo("UnifiedBrain => done w/ cliff avoidance, now resume forward + short cooldown")
        self.cmd_pub.publish(FORWARD_CMD)

        # Short cooldown so we don't re-trigger the same cliff reading immediately
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
        self.cmd_pub.publish(STOP_CMD)
        rospy.sleep(0.5)

        if left_trigger and right_trigger:
            # BOTH => full 180 + spin
            rospy.loginfo("UnifiedBrain => BACKWARD for %.1f s", BACKWARD_DURATION_OBST)
            self.cmd_pub.publish(BACKWARD_CMD)
            rospy.sleep(BACKWARD_DURATION_OBST)

            rospy.loginfo("UnifiedBrain => ROTATE 180° for %.1f s", ROTATE_180_DURATION)
            self.cmd_pub.publish(ROTATE_180_CMD)
            rospy.sleep(ROTATE_180_DURATION)

            # Optional small random spin or none
            spin_count = random.randint(0, 1)
            for i in range(spin_count):
                cmd  = random.choice([SPIN_ADJUST_LEFT_CMD, SPIN_ADJUST_RIGHT_CMD])
                secs = random.uniform(2.0, 3.0)
                rospy.loginfo("UnifiedBrain => spin %s for %.1f s",
                              "LEFT" if cmd == SPIN_ADJUST_LEFT_CMD else "RIGHT", secs)
                self.cmd_pub.publish(cmd)
                rospy.sleep(secs)
        else:
            # Single side => spin away
            if left_trigger:
                rospy.loginfo("UnifiedBrain => SPIN RIGHT for %.1f s", SINGLE_SPIN_DURATION)
                self.cmd_pub.publish(SPIN_ADJUST_RIGHT_CMD)
                rospy.sleep(SINGLE_SPIN_DURATION)
            else:
                rospy.loginfo("UnifiedBrain => SPIN LEFT for %.1f s", SINGLE_SPIN_DURATION)
                self.cmd_pub.publish(SPIN_ADJUST_LEFT_CMD)
                rospy.sleep(SINGLE_SPIN_DURATION)

        rospy.loginfo("UnifiedBrain => Forward, then short cooldown so we don't re-trigger instantly.")
        self.cmd_pub.publish(FORWARD_CMD)

        rospy.sleep(1.0)  # let the robot actually move forward a bit
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