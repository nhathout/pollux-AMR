#!/usr/bin/env python3
import os, sys, math, rospy
from sensor_msgs.msg import Imu
from std_msgs.msg  import Int32

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

# LED command codes
SANITIZE_OFF    = 0
SANITIZE_ON     = 1
ROBOT_ON_BRIGHT = 3

# Motor command for STOP
STOP_CMD        = 6

# New thresholds
TILT_LIMIT_DEG  = 60.0       # If computed tilt is less than 60°, we consider the robot level
SAFE_DELAY_SECS = 0.5        # Need level for at least 0.5 s before turning UV back on
CHECK_PERIOD    = 0.5        # Periodic timer check

class LedGyroNode:
    def __init__(self):
        rospy.init_node('led_gyro_node')

        self.led_pub        = rospy.Publisher('/pollux/led_cmd', Int32, queue_size=3)
        self.last_motion    = STOP_CMD
        self.uv_on          = False

        # We'll keep track of the most recent tilt reading
        self.current_tilt_deg = None
        self.tilt_safe_since  = None

        rospy.Subscriber('/pollux/motor_cmd', Int32, self.motor_cb)
        rospy.Subscriber('/pollux/imu',        Imu,  self.imu_cb)

        # Ensure the ON‑LED is set to bright
        rospy.sleep(0.3)
        self.led_pub.publish(ROBOT_ON_BRIGHT)

        rospy.Timer(rospy.Duration(CHECK_PERIOD), self.timer_cb)
        rospy.loginfo("led_gyro_node running.")
        rospy.spin()

    def motor_cb(self, msg):
        self.last_motion = msg.data
        # If stopped, ensure UV is off.
        if msg.data == STOP_CMD:
            if self.uv_on:
                self.led_pub.publish(SANITIZE_OFF)
                self.uv_on = False
        else:
            # If moving and currently level, try to enable UV
            if self._is_level() and not self.uv_on:
                self.led_pub.publish(SANITIZE_ON)
                self.uv_on = True

    def imu_cb(self, imu):
        la = imu.linear_acceleration
        gx, gy, gz = la.x, la.y, la.z
        g_mag = math.sqrt(gx*gx + gy*gy + gz*gz)
        if g_mag < 1e-3:
            return
        # Compute tilt as the angle between gravity vector and vertical.
        tilt_deg = math.degrees(math.acos(max(min(gz / g_mag, 1.0), -1.0)))
        self.current_tilt_deg = tilt_deg

        if tilt_deg > TILT_LIMIT_DEG:
            # Mark the time as unsafe immediately.
            self.tilt_safe_since = None
            if self.uv_on:
                rospy.logwarn_throttle(2.0, f"Tilt {tilt_deg:.0f}° > {TILT_LIMIT_DEG}° – UV OFF")
                self.led_pub.publish(SANITIZE_OFF)
                self.uv_on = False
        else:
            # If level, record the time if not already done.
            if self.tilt_safe_since is None:
                self.tilt_safe_since = rospy.get_time()

    def timer_cb(self, _):
        # Only consider re‑enabling UV if the robot is commanded to move.
        if self.last_motion == STOP_CMD:
            return
        # If we already have UV on, nothing to do.
        if self.uv_on:
            return
        # Only re‑enable if we’ve been level for at least SAFE_DELAY_SECS.
        if self.tilt_safe_since is not None and (rospy.get_time() - self.tilt_safe_since) >= SAFE_DELAY_SECS:
            rospy.loginfo("Robot level for %.1fs – UV back ON", SAFE_DELAY_SECS)
            self.led_pub.publish(SANITIZE_ON)
            self.uv_on = True

    def _is_level(self):
        # We consider the robot level if the current tilt reading is below TILT_LIMIT_DEG.
        return self.current_tilt_deg is not None and self.current_tilt_deg < TILT_LIMIT_DEG

if __name__ == '__main__':
    try:
        LedGyroNode()
    except rospy.ROSInterruptException:
        pass