#!/usr/bin/env python3
import os, sys, math, rospy
from sensor_msgs.msg import Imu
from std_msgs.msg  import Int32

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

# LED command codes (should match led_control_node)
SANITIZE_OFF    = 0
SANITIZE_ON     = 1
ROBOT_ON_BRIGHT = 3

# Thresholds and timing
TILT_LIMIT_DEG  = 60.0       # Consider robot level if tilt is below this value
SAFE_DELAY_SECS = 0.5        # Duration required level before re-enabling UV/indicator
CHECK_PERIOD    = 0.5        # Timer callback period

class LedGyroNode:
    def __init__(self):
        rospy.init_node('led_gyro_node')

        # Publisher for LED commands
        self.led_pub = rospy.Publisher('/pollux/led_cmd', Int32, queue_size=3)

        # Track current state for UV/indicator LEDs (True means ON)
        self.uv_on = False

        # For determining safe re-enabling time
        self.current_tilt_deg = None
        self.tilt_safe_since = None

        # Subscribe to IMU for tilt measurement
        rospy.Subscriber('/pollux/imu', Imu, self.imu_cb)

        # At startup, turn on the robot on LED once and enable UV/indicator LEDs if safe
        rospy.sleep(0.3)
        self.led_pub.publish(Int32(data=ROBOT_ON_BRIGHT))
        self.update_uv_state(True)

        # Timer to periodically check if conditions are met to re-enable UV/indicator LEDs
        rospy.Timer(rospy.Duration(CHECK_PERIOD), self.timer_cb)
        rospy.loginfo("led_gyro_node running: LEDs initialized.")
        rospy.spin()

    def update_uv_state(self, desired_state):
        """
        Publish a new command for the UV/indicator LEDs only if the state is changing.
        desired_state: True to turn ON, False to turn OFF.
        """
        if desired_state and not self.uv_on:
            self.led_pub.publish(Int32(data=SANITIZE_ON))
            self.uv_on = True
            rospy.loginfo("UV/Indicator LEDs turned ON")
        elif not desired_state and self.uv_on:
            self.led_pub.publish(Int32(data=SANITIZE_OFF))
            self.uv_on = False
            rospy.loginfo("UV/Indicator LEDs turned OFF")

    def imu_cb(self, imu):
        # Compute the magnitude of the acceleration vector
        la = imu.linear_acceleration
        gx, gy, gz = la.x, la.y, la.z
        g_mag = math.sqrt(gx*gx + gy*gy + gz*gz)
        if g_mag < 1e-3:
            return  # Avoid division by zero

        # Calculate tilt angle (angle between gravity and vertical)
        tilt_deg = math.degrees(math.acos(max(min(gz / g_mag, 1.0), -1.0)))
        self.current_tilt_deg = tilt_deg

        # If tilt exceeds the safe limit, ensure UV/indicator are off
        if tilt_deg > TILT_LIMIT_DEG:
            self.tilt_safe_since = None
            self.update_uv_state(False)
            rospy.logwarn_throttle(2.0, f"Tilt {tilt_deg:.0f}° > {TILT_LIMIT_DEG}° – turning off UV/Indicator")
        else:
            # Record the time the robot became level (if not already recorded)
            if self.tilt_safe_since is None:
                self.tilt_safe_since = rospy.get_time()

    def timer_cb(self, _):
        # If the robot has been level for at least SAFE_DELAY_SECS, re-enable UV/indicator LEDs if they're off.
        if self.tilt_safe_since is not None:
            elapsed = rospy.get_time() - self.tilt_safe_since
            if elapsed >= SAFE_DELAY_SECS:
                self.update_uv_state(True)

if __name__ == '__main__':
    try:
        LedGyroNode()
    except rospy.ROSInterruptException:
        pass