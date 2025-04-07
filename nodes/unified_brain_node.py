#!/usr/bin/env python3

import os
import sys
import time
import random
import rospy
from std_msgs.msg import Float32MultiArray, Int32

# === Adjust Python path if needed for your hardware modules ===
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
CLIFF_THRESHOLD        = 15.0   # cm => reading above means "cliff"
CLIFF_DEBOUNCE_TIME    = 10.0   # ignore new cliff triggers for 10 s
BACKWARD_DURATION_MIN  = 1.0    # random back time range
BACKWARD_DURATION_MAX  = 2.0
ROTATE_180_DURATION    = 4.0

# === Front Obstacle Constants ===
OBSTACLE_THRESHOLD     = 18.0   # cm => reading below means "obstacle"
OBSTACLE_DEBOUNCE_TIME = 3.0    # ignore new triggers for 3 s
# If BOTH sensors see an obstacle, we do the same "full 180 + random spin" as cliffs
# If only one sensor sees an obstacle, we do a short spin away, then forward
SINGLE_SPIN_DURATION   = 2.0    # how long we spin if only one sensor is triggered
BACKWARD_DURATION_OBST = 0.5    # how long we back up if both sensors triggered

class UnifiedBrainNode:
    """
    A single node that:
      • Subscribes to bottom ultrasonic sensors (/pollux/ultrasonic_hw) -> detect cliffs
      • Subscribes to front ultrasonic sensors (/pollux/ultrasonic_2) -> detect front obstacles
      • Publishes motor commands on /pollux/motor_cmd
    """
    def __init__(self):
        rospy.init_node('unified_brain_node', anonymous=True)
        rospy.loginfo("Unified Brain Node started.")

        # === Subscribe to bottom (cliff) sensors ===
        self.bottom_sub = rospy.Subscriber(
            '/pollux/ultrasonic_hw',
            Float32MultiArray,
            self.bottom_sensors_callback
        )

        # === Subscribe to front (obstacle) sensors ===
        self.front_sub = rospy.Subscriber(
            '/pollux/ultrasonic_2',
            Float32MultiArray,
            self.front_sensors_callback
        )

        # === Publisher for motor commands ===
        self.cmd_pub = rospy.Publisher('/pollux/motor_cmd', Int32, queue_size=10)

        # === State variables ===
        self.last_cliff_time  = 0.0
        self.last_front_time  = 0.0
        self.in_action        = False  # "busy" flag to prevent overlaps

        # Timer to send FORWARD_CMD periodically when idle
        self.forward_timer = rospy.Timer(rospy.Duration(4.0), self.forward_timer_cb)

    # --------------------------------------------------------------------------
    # Periodic forward "keep alive" (only if not in avoidance):
    # --------------------------------------------------------------------------
    def forward_timer_cb(self, event):
        if not self.in_action:
            rospy.loginfo("UnifiedBrain => Commanding forward motion.")
            self.cmd_pub.publish(FORWARD_CMD)

    # --------------------------------------------------------------------------
    # CLIFF DETECTION (Priority #1)
    # --------------------------------------------------------------------------
    def bottom_sensors_callback(self, msg):
        """
        Called for bottom ultrasonic readings: [dist_left, dist_mid, dist_right].
        If we detect a "cliff" (distance > CLIFF_THRESHOLD), we do the "old" cliff
        avoidance routine (stop -> random backward -> rotate 180 -> random spin).
        """
        if self.in_action:
            return  # Already handling something

        current_time = rospy.get_time()
        if (current_time - self.last_cliff_time) < CLIFF_DEBOUNCE_TIME:
            return  # Not enough time since last cliff event

        distances = list(msg.data)
        if len(distances) != 3:
            rospy.logwarn("UnifiedBrain => bottom_sensors: expected 3, got %d", len(distances))
            return

        # Check for cliff
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

        # --- Perform the "old" cliff-avoidance sequence ---
        self.in_action       = True
        self.last_cliff_time = current_time

        rospy.loginfo("UnifiedBrain => STOP (cliff detected)")
        self.cmd_pub.publish(STOP_CMD)
        rospy.sleep(0.5)

        # 1) Random backward (1 to 2 seconds)
        back_time = random.uniform(BACKWARD_DURATION_MIN, BACKWARD_DURATION_MAX)
        rospy.loginfo("UnifiedBrain => BACKWARD for %.1f s", back_time)
        self.cmd_pub.publish(BACKWARD_CMD)
        rospy.sleep(back_time)

        # 2) Full 180° rotation
        rospy.loginfo("UnifiedBrain => ROTATE 180° (%.1f s)", ROTATE_180_DURATION)
        self.cmd_pub.publish(ROTATE_180_CMD)
        rospy.sleep(ROTATE_180_DURATION)

        # 3) Random spin adjustments (1 or 2 spins, each ~2-4s)
        spin_count = random.randint(1, 2)
        for i in range(spin_count):
            cmd  = random.choice([SPIN_ADJUST_LEFT_CMD, SPIN_ADJUST_RIGHT_CMD])
            secs = random.uniform(2.0, 4.0)
            rospy.loginfo("UnifiedBrain => spin %s for %.1f s",
                          "LEFT" if cmd == SPIN_ADJUST_LEFT_CMD else "RIGHT", secs)
            self.cmd_pub.publish(cmd)
            rospy.sleep(secs)

        rospy.loginfo("UnifiedBrain => done w/ cliff avoidance, resume forward")
        self.in_action = False

    # --------------------------------------------------------------------------
    # FRONT OBSTACLES
    # --------------------------------------------------------------------------
    def front_sensors_callback(self, msg):
        """
        Called for front ultrasonic readings: [dist_left, dist_right].
        If *both* are under threshold => do a full "180° + random spin" sequence.
        If only one side => do a short spin away, then forward, so we can iterate
        if the obstacle is still there.
        """
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

        # Check triggers
        left_trigger  = (0 < dist_left  < OBSTACLE_THRESHOLD)
        right_trigger = (0 < dist_right < OBSTACLE_THRESHOLD)

        if not (left_trigger or right_trigger):
            return  # no obstacle

        rospy.loginfo("Front obstacle. L=%.2f, R=%.2f", dist_left, dist_right)
        self.in_action       = True
        self.last_front_time = current_time

        rospy.loginfo("UnifiedBrain => STOP (front obstacle)")
        self.cmd_pub.publish(STOP_CMD)
        rospy.sleep(0.5)

        # If BOTH sensors triggered => do the "full 180° + random spin" routine
        if left_trigger and right_trigger:
            # 1) Random short back
            back_time = random.uniform(BACKWARD_DURATION_MIN, BACKWARD_DURATION_MAX)
            rospy.loginfo("UnifiedBrain => BACKWARD for %.1f s", back_time)
            self.cmd_pub.publish(BACKWARD_CMD)
            rospy.sleep(back_time)

            # 2) 180° rotate
            rospy.loginfo("UnifiedBrain => ROTATE 180° for %.1f s", ROTATE_180_DURATION)
            self.cmd_pub.publish(ROTATE_180_CMD)
            rospy.sleep(ROTATE_180_DURATION)

            # 3) Random spin adjustments
            spin_count = random.randint(1, 2)
            for i in range(spin_count):
                cmd  = random.choice([SPIN_ADJUST_LEFT_CMD, SPIN_ADJUST_RIGHT_CMD])
                secs = random.uniform(2.0, 4.0)
                rospy.loginfo("UnifiedBrain => spin %s for %.1f s",
                              "LEFT" if cmd == SPIN_ADJUST_LEFT_CMD else "RIGHT", secs)
                self.cmd_pub.publish(cmd)
                rospy.sleep(secs)

        else:
            # Only one sensor triggered => small spin away from that side
            # (No large backward; just move away.)
            if left_trigger:
                # Obstacle on left => spin right
                rospy.loginfo("UnifiedBrain => SPIN RIGHT for %.1f s", SINGLE_SPIN_DURATION)
                self.cmd_pub.publish(SPIN_ADJUST_RIGHT_CMD)
                rospy.sleep(SINGLE_SPIN_DURATION)
            else:
                # Obstacle on right => spin left
                rospy.loginfo("UnifiedBrain => SPIN LEFT for %.1f s", SINGLE_SPIN_DURATION)
                self.cmd_pub.publish(SPIN_ADJUST_LEFT_CMD)
                rospy.sleep(SINGLE_SPIN_DURATION)

        # Finally, resume forward
        rospy.loginfo("UnifiedBrain => Forward.")
        self.cmd_pub.publish(FORWARD_CMD)

        self.in_action = False

    # --------------------------------------------------------------------------
    def run(self):
        rospy.spin()

# ------------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        node = UnifiedBrainNode()
        node.run()
    except rospy.ROSInterruptException:
        pass