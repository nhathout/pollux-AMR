#!/usr/bin/env python3
import os, sys, math, rospy
from sensor_msgs.msg import Imu
from std_msgs.msg  import Int32

# package path helper ---------------------------------------------------------
SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

# LED commands ---------------------------------------------------------------
SANITIZE_OFF    = 0
SANITIZE_ON     = 1
ROBOT_ON_BRIGHT = 3

STOP_CMD        = 6                     # motor stop

TILT_LIMIT_DEG      = 25.0              # >25° ⇒ unsafe
SAFE_DELAY_SECS     = 2.0               # need this long of level before UV on
CHECK_PERIOD        = 0.5               # timer period

class LedGyroNode:
    def __init__(self):
        rospy.init_node('led_gyro_node')

        self.led_pub   = rospy.Publisher('/pollux/led_cmd', Int32, queue_size=3)
        self.last_motion_cmd = STOP_CMD
        self.sanitize_on     = False
        self.last_tilt_time  = rospy.get_time()

        rospy.Subscriber('/pollux/motor_cmd', Int32, self.motor_cb)
        rospy.Subscriber('/pollux/imu',        Imu,  self.imu_cb)

        # make sure ON‑LED is bright
        rospy.sleep(0.3)
        self.led_pub.publish(ROBOT_ON_BRIGHT)

        # periodic check to re‑enable sanitize
        rospy.Timer(rospy.Duration(CHECK_PERIOD), self.timer_cb)

        rospy.loginfo("led_gyro_node running.")
        rospy.spin()

    # ---------------------------- motor commands ----------------------------
    def motor_cb(self, msg):
        self.last_motion_cmd = msg.data
        if msg.data == STOP_CMD:
            if self.sanitize_on:
                self.led_pub.publish(SANITIZE_OFF)
                self.sanitize_on = False
        else:
            # robot commanded to move → desire UV on when safe
            if self._is_level() and not self.sanitize_on:
                self.led_pub.publish(SANITIZE_ON)
                self.sanitize_on = True

    # ----------------------------- IMU callback ----------------------------
    def imu_cb(self, imu):
        la = imu.linear_acceleration
        gx, gy, gz = la.x, la.y, la.z
        g_mag = math.sqrt(gx*gx + gy*gy + gz*gz)
        if g_mag < 1e-3:        # invalid reading
            return

        tilt_deg = math.degrees(math.acos(max(min(gz / g_mag, 1.0), -1.0)))

        if tilt_deg > TILT_LIMIT_DEG:
            self.last_tilt_time = rospy.get_time()
            if self.sanitize_on:
                rospy.logwarn_throttle(2.0, f"Tilt {tilt_deg:.0f}° – UV OFF")
                self.led_pub.publish(SANITIZE_OFF)
                self.sanitize_on = False

    # ---------------------------- periodic check ---------------------------
    def timer_cb(self, _):
        if self.sanitize_on:
            return                             # already on
        if self.last_motion_cmd == STOP_CMD:
            return                             # robot idle
        if self._is_level() and self._level_for(SAFE_DELAY_SECS):
            rospy.loginfo("Level for %.1fs – UV back ON", SAFE_DELAY_SECS)
            self.led_pub.publish(SANITIZE_ON)
            self.sanitize_on = True

    # ----------------------------- helpers ---------------------------------
    def _is_level(self):
        """Return True if the robot is currently level (< tilt limit)."""
        # This flag is updated in imu_cb: if we've been tilted very recently,
        # we assume we're still not level.  That's sufficient for our logic.
        return (rospy.get_time() - self.last_tilt_time) > 0.1

    def _level_for(self, secs):
        """True if we've been level for at least `secs` seconds."""
        return (rospy.get_time() - self.last_tilt_time) >= secs

if __name__ == '__main__':
    try:
        LedGyroNode()
    except rospy.ROSInterruptException:
        pass