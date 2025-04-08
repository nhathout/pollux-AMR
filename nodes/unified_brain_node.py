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

# === Motor command codes (must match motor_cmd_node.py) ===
FORWARD_CMD            = 0
BACKWARD_CMD           = 1
STOP_CMD               = 6
ROTATE_180_CMD         = 7
SPIN_ADJUST_LEFT_CMD   = 8
SPIN_ADJUST_RIGHT_CMD  = 9

# === Cliff (bottom) Detection Constants ===
CLIFF_THRESHOLD        = 15.0   # cm => reading above => "cliff"
CLIFF_DEBOUNCE_TIME    = 10.0   # ignore new cliff triggers for 10 s
ROTATE_180_DURATION    = 4.0
CLIFF_BACK_SECS_EACH   = 2.0    # each backward pass is 2 s, do 2 passes => ~4 s total

# === Front Obstacle Constants ===
OBSTACLE_THRESHOLD     = 18.0
OBSTACLE_DEBOUNCE_TIME = 3.0
SINGLE_SPIN_DURATION   = 2.0    # spin away from obstacle side if only one sensor triggers

class UnifiedBrainNode:
    """
    A single node that:
      - Subscribes to bottom ultrasonic sensors (/pollux/ultrasonic_hw) -> detect cliffs
      - Subscribes to front ultrasonic sensors (/pollux/ultrasonic_2) -> detect obstacles
      - Publishes motor commands on /pollux/motor_cmd

    The keep-alive timer commands forward every 5 s if not in the middle of an action.
    """

    def __init__(self):
        rospy.init_node('unified_brain_node', anonymous=True)
        rospy.loginfo("Unified Brain Node started.")

        # Subscribers
        self.bottom_sub = rospy.Subscriber('/pollux/ultrasonic_hw',
                                           Float32MultiArray,
                                           self.bottom_sensors_callback)
        self.front_sub  = rospy.Subscriber('/pollux/ultrasonic_2',
                                           Float32MultiArray,
                                           self.front_sensors_callback)

        # Publisher
        self.cmd_pub = rospy.Publisher('/pollux/motor_cmd', Int32, queue_size=10)

        # State
        self.last_cliff_time  = 0.0
        self.last_front_time  = 0.0
        self.in_action        = False  # busy flag

        # Timer: re-issue forward if idle
        self.forward_timer = rospy.Timer(rospy.Duration(4.0), self.forward_timer_cb)

    # --------------------------------------------------------------------------
    # Keep-alive forward motion if not in avoidance
    # --------------------------------------------------------------------------
    def forward_timer_cb(self, event):
        if not self.in_action:
            rospy.loginfo("UnifiedBrain => Commanding forward motion.")
            self.cmd_pub.publish(FORWARD_CMD)

    # --------------------------------------------------------------------------
    # CLIFF DETECTION
    # --------------------------------------------------------------------------
    def bottom_sensors_callback(self, msg):
        """
        Called for bottom ultrasonic readings: [left, mid, right].
        If distance > CLIFF_THRESHOLD => "cliff" => old avoidance routine.
        """
        if self.in_action:
            return

        current_time = rospy.get_time()
        if (current_time - self.last_cliff_time) < CLIFF_DEBOUNCE_TIME:
            return

        distances = list(msg.data)
        if len(distances) != 3:
            rospy.logwarn("UnifiedBrain => bottom_sensors: expected 3, got %d", len(distances))
            return

        # Check any sensor for "cliff"
        cliff_found = False
        for idx, dist_val in enumerate(distances):
            if dist_val < 0:
                continue
            if dist_val > CLIFF_THRESHOLD:
                rospy.loginfo("Cliff detected on bottom sensor %d (%.1f cm)", idx, dist_val)
                cliff_found = True
                break

        if not cliff_found:
            return

        # Start cliff avoidance
        self.in_action        = True
        self.last_cliff_time  = current_time

        rospy.loginfo("UnifiedBrain => STOP (cliff detected)")
        self.cmd_pub.publish(STOP_CMD)
        rospy.sleep(0.5)

        # Double backward => ~4 s total
        for i in range(2):
            rospy.loginfo(f"Cliff => BACKWARD pass {i+1} for {CLIFF_BACK_SECS_EACH}s")
            self.cmd_pub.publish(BACKWARD_CMD)
            rospy.sleep(CLIFF_BACK_SECS_EACH)

        rospy.loginfo(f"Cliff => ROTATE 180° ({ROTATE_180_DURATION}s)")
        self.cmd_pub.publish(ROTATE_180_CMD)
        rospy.sleep(ROTATE_180_DURATION)

        # Optional single random spin
        spin_count = random.randint(0, 1)
        for _ in range(spin_count):
            cmd  = random.choice([SPIN_ADJUST_LEFT_CMD, SPIN_ADJUST_RIGHT_CMD])
            secs = random.uniform(1.0, 2.0)
            rospy.loginfo(f"Cliff => spin {cmd} for {secs:.1f}s")
            self.cmd_pub.publish(cmd)
            rospy.sleep(secs)

        rospy.loginfo("Cliff => done, forward + cooldown")
        self.cmd_pub.publish(FORWARD_CMD)
        rospy.sleep(1.0)  # short cooldown
        self.in_action = False

    # --------------------------------------------------------------------------
    # FRONT OBSTACLES
    # --------------------------------------------------------------------------
    def front_sensors_callback(self, msg):
        """
        Called for front ultrasonic readings: [left, right].
        If both triggered => same "cliff" routine (stop, double-back, rotate, optional spin).
        If only one => short stop, quick spin away, forward again.
        """
        if self.in_action:
            return

        current_time = rospy.get_time()
        if (current_time - self.last_front_time) < 6.0:  # OBSTACLE_DEBOUNCE_TIME
            return

        distances_2 = list(msg.data)
        if len(distances_2) < 2:
            rospy.logwarn("UnifiedBrain => front_sensors: expected 2, got %d", len(distances_2))
            return

        dist_left, dist_right = distances_2
        if dist_left < 0 or dist_right < 0:
            rospy.logwarn("UnifiedBrain => front sensor invalid. (L=%.1f, R=%.1f)", dist_left, dist_right)

        left_trigger  = (0 < dist_left  < OBSTACLE_THRESHOLD)
        right_trigger = (0 < dist_right < OBSTACLE_THRESHOLD)

        if not (left_trigger or right_trigger):
            return  # no obstacle

        # Start obstacle avoidance
        self.in_action       = True
        self.last_front_time = current_time

        rospy.loginfo(f"Front obstacle => STOP. L={dist_left:.1f}, R={dist_right:.1f}")
        self.cmd_pub.publish(STOP_CMD)
        rospy.sleep(1.0)  # .5–1s stop

        if left_trigger and right_trigger:
            # Both sensors => do the same logic as cliff:
            rospy.loginfo("Front => BOTH triggered => double BACKWARD + rotate 180 + optional spin")

            # Double backward => ~4 s
            for i in range(1):
                rospy.loginfo(f"Front => BACKWARD pass {i+1} for {CLIFF_BACK_SECS_EACH}s")
                self.cmd_pub.publish(BACKWARD_CMD)
                rospy.sleep(CLIFF_BACK_SECS_EACH)

            rospy.loginfo(f"Front => ROTATE 180° ({ROTATE_180_DURATION}s)")
            self.cmd_pub.publish(ROTATE_180_CMD)
            #rospy.sleep(ROTATE_180_DURATION)

            # optional single random spin
            spin_count = random.randint(0, 1)
            for _ in range(spin_count):
                cmd  = random.choice([SPIN_ADJUST_LEFT_CMD, SPIN_ADJUST_RIGHT_CMD])
                secs = random.uniform(1.0, 2.0)
                rospy.loginfo(f"Front => spin {cmd} for {secs:.1f}s")
                self.cmd_pub.publish(cmd)
                rospy.sleep(secs)

        else:
            # Only one side => slight spin away
            if left_trigger:
                rospy.loginfo(f"Front => single obstacle LEFT => spin RIGHT {SINGLE_SPIN_DURATION}s")
                self.cmd_pub.publish(SPIN_ADJUST_RIGHT_CMD)
                rospy.sleep(SINGLE_SPIN_DURATION)
            else:
                rospy.loginfo(f"Front => single obstacle RIGHT => spin LEFT {SINGLE_SPIN_DURATION}s")
                self.cmd_pub.publish(SPIN_ADJUST_LEFT_CMD)
                rospy.sleep(SINGLE_SPIN_DURATION)

        rospy.loginfo("Front => done, forward + short cooldown")
        self.cmd_pub.publish(FORWARD_CMD)
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