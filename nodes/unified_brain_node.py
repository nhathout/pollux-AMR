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

# === Motor command codes ===
FORWARD_CMD            = 0
BACKWARD_CMD           = 1
STOP_CMD               = 6
ROTATE_180_CMD         = 7
SPIN_ADJUST_LEFT_CMD   = 8
SPIN_ADJUST_RIGHT_CMD  = 9

# === Cliff Detection Constants (bottom sensors) ===
CLIFF_THRESHOLD        = 15.0   # cm; reading > 15 => potential cliff
CLIFF_DEBOUNCE_TIME    = 10.0   # ignore new cliff triggers for 10 s
BACKWARD_DURATION_MIN  = 1.0
BACKWARD_DURATION_MAX  = 2.0
ROTATE_180_DURATION    = 3.0

# === Front Obstacle Constants (front sensors) ===
OBSTACLE_THRESHOLD     = 18.0   # cm; reading < 18 => obstacle
OBSTACLE_DEBOUNCE_TIME = 3.0    # ignore new obstacle triggers for 3 s
OBSTACLE_BACKWARD_SEC  = 2.0    # how long to back up
OBSTACLE_TURN_SEC      = 2.0    # how long to turn away

class UnifiedBrainNode:
    """
    A single node that:
      - Subscribes to bottom ultrasonic sensors (/pollux/ultrasonic_hw)
        to detect cliffs and avoid them
      - Subscribes to front ultrasonic sensors (/pollux/ultrasonic_2)
        to detect obstacles in front
      - Publishes motor commands on /pollux/motor_cmd

    This avoids conflicting commands from two separate "brain" nodes.
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
        self.in_action        = False   # "busy" flag to prevent overlap

        # === Timer to periodically command forward motion if idle ===
        self.forward_timer = rospy.Timer(rospy.Duration(2.0), self.forward_timer_cb)

    def forward_timer_cb(self, event):
        """
        Called every 2 seconds to ensure the robot is moving forward
        whenever we are not in the middle of an avoidance action.
        """
        if not self.in_action:
            rospy.loginfo("UnifiedBrain => Commanding forward motion.")
            self.cmd_pub.publish(FORWARD_CMD)

    # --------------------------------------------------------------------------
    # 1) BOTTOM SENSORS (detect “cliffs”)
    # --------------------------------------------------------------------------
    def bottom_sensors_callback(self, msg):
        """
        Called whenever the bottom ultrasonic sensors publish data
        (e.g., [dist_left, dist_mid, dist_right]).
        If we detect a “cliff” reading (distance > CLIFF_THRESHOLD),
        perform the cliff-avoidance sequence.
        """
        if self.in_action:
            return  # Already handling an event

        current_time = rospy.get_time()
        if current_time - self.last_cliff_time < CLIFF_DEBOUNCE_TIME:
            # Not enough time has passed since last cliff event
            return

        distances = list(msg.data)
        if len(distances) != 3:
            rospy.logwarn("Expected 3 bottom sensors, got %d", len(distances))
            return

        # Check if any bottom sensor indicates a cliff
        cliff_detected = False
        for idx, dist_val in enumerate(distances):
            if dist_val < 0:
                continue  # invalid sensor reading
            if dist_val > CLIFF_THRESHOLD:
                rospy.loginfo("Cliff detected on bottom sensor %d (%.1f cm)", idx, dist_val)
                cliff_detected = True
                break

        if not cliff_detected:
            return

        # -------- Cliff Avoidance Sequence --------
        self.in_action       = True
        self.last_cliff_time = current_time

        rospy.loginfo("UnifiedBrain => STOP (cliff detected)")
        self.cmd_pub.publish(STOP_CMD)
        rospy.sleep(0.5)

        # Random back time to give variety
        back_time = random.uniform(BACKWARD_DURATION_MIN, BACKWARD_DURATION_MAX)
        rospy.loginfo("UnifiedBrain => BACKWARD for %.1f s", back_time)
        self.cmd_pub.publish(BACKWARD_CMD)
        rospy.sleep(back_time)

        rospy.loginfo("UnifiedBrain => ROTATE 180° for %.1f s", ROTATE_180_DURATION)
        self.cmd_pub.publish(ROTATE_180_CMD)
        rospy.sleep(ROTATE_180_DURATION)

        # Optional random spin adjustments
        spin_count = random.randint(1, 2)
        for i in range(spin_count):
            cmd  = random.choice([SPIN_ADJUST_LEFT_CMD, SPIN_ADJUST_RIGHT_CMD])
            secs = random.uniform(2.0, 4.0)
            side = "L" if cmd == SPIN_ADJUST_LEFT_CMD else "R"
            rospy.loginfo(f"UnifiedBrain => spin {side} for {secs:.1f}s")
            self.cmd_pub.publish(cmd)
            rospy.sleep(secs)

        rospy.loginfo("UnifiedBrain => done with cliff avoidance, resume forward")
        self.in_action = False

    # --------------------------------------------------------------------------
    # 2) FRONT SENSORS (detect obstacles)
    # --------------------------------------------------------------------------
    def front_sensors_callback(self, msg):
        """
        Called whenever the front ultrasonic sensors publish data
        (e.g., [dist_left, dist_right]).
        If either sensor is < OBSTACLE_THRESHOLD => front obstacle detected.
        """
        if self.in_action:
            return  # Already busy with an avoidance action

        current_time = rospy.get_time()
        if current_time - self.last_front_time < OBSTACLE_DEBOUNCE_TIME:
            # Not enough time has passed since last front event
            return

        distances_2 = list(msg.data)
        if len(distances_2) < 2:
            rospy.logwarn("Expected 2 front sensors, got %d", len(distances_2))
            return

        dist_left, dist_right = distances_2
        # If -1 => invalid reading
        if dist_left < 0 or dist_right < 0:
            rospy.logwarn("Front sensor invalid. (L=%.2f, R=%.2f)", dist_left, dist_right)

        # Check if either is under obstacle threshold
        obstacle_detected = False
        if (0 < dist_left  < OBSTACLE_THRESHOLD) or (0 < dist_right < OBSTACLE_THRESHOLD):
            obstacle_detected = True

        if not obstacle_detected:
            return

        # -------- Front Obstacle Avoidance Sequence --------
        rospy.loginfo("Front obstacle detected. L=%.2f cm, R=%.2f cm",
                      dist_left, dist_right)
        self.in_action       = True
        self.last_front_time = current_time

        # 1) STOP
        rospy.loginfo("UnifiedBrain => STOP for front obstacle")
        self.cmd_pub.publish(STOP_CMD)
        rospy.sleep(0.5)

        # 2) Move backward
        rospy.loginfo("UnifiedBrain => BACKWARD for %.1f s", OBSTACLE_BACKWARD_SEC)
        self.cmd_pub.publish(BACKWARD_CMD)
        rospy.sleep(OBSTACLE_BACKWARD_SEC)

        # 3) Turn away from the obstacle
        if dist_left < dist_right:
            rospy.loginfo("UnifiedBrain => Turn RIGHT for %.1f s", OBSTACLE_TURN_SEC)
            self.cmd_pub.publish(SPIN_ADJUST_RIGHT_CMD)
        else:
            rospy.loginfo("UnifiedBrain => Turn LEFT for %.1f s", OBSTACLE_TURN_SEC)
            self.cmd_pub.publish(SPIN_ADJUST_LEFT_CMD)

        rospy.sleep(OBSTACLE_TURN_SEC)

        # 4) Resume forward
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