#!/usr/bin/env python3
import os, sys, math, rospy
from sensor_msgs.msg import Imu
from std_msgs.msg  import Int32

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

SANITIZE_OFF    = 0
SANITIZE_ON     = 1
ROBOT_ON_BRIGHT = 3
STOP_CMD        = 6

TILT_RATIO_LIMIT = 0.6          # |gz| / |g| below this ⇒ unsafe (~>53° tilt)
SAFE_DELAY       = 1.5          # s of level before re‑enabling UV
CHECK_PERIOD     = 0.5

class LedGyroNode:
    def __init__(self):
        rospy.init_node('led_gyro_node')

        self.led_pub   = rospy.Publisher('/pollux/led_cmd', Int32, queue_size=3)
        self.last_motion_cmd = STOP_CMD
        self.uv_on      = False
        self.last_unsafe = rospy.get_time()

        rospy.Subscriber('/pollux/motor_cmd', Int32, self.motor_cb)
        rospy.Subscriber('/pollux/imu',        Imu,  self.imu_cb)

        rospy.sleep(0.3)
        self.led_pub.publish(ROBOT_ON_BRIGHT)

        rospy.Timer(rospy.Duration(CHECK_PERIOD), self.timer_cb)
        rospy.loginfo("led_gyro_node ready.")
        rospy.spin()

    # ---------------- IMU ----------------
    def imu_cb(self, imu):
        la = imu.linear_acceleration
        gx, gy, gz = la.x, la.y, la.z
        g = math.sqrt(gx*gx + gy*gy + gz*gz)
        if g < 1e-3:
            return                                # bad packet
        if abs(gz) / g < TILT_RATIO_LIMIT:        # too much tilt
            self.last_unsafe = rospy.get_time()
            if self.uv_on:
                rospy.logwarn_throttle(2.0, "Tilt detected – UV OFF")
                self.led_pub.publish(SANITIZE_OFF)
                self.uv_on = False

    # -------------- motor cmd ------------
    def motor_cb(self, msg):
        self.last_motion_cmd = msg.data
        if msg.data == STOP_CMD and self.uv_on:
            self.led_pub.publish(SANITIZE_OFF)
            self.uv_on = False
        elif msg.data != STOP_CMD:
            # robot commanded to move → want UV on when safe
            self.try_enable_uv()

    # ------------- periodic --------------
    def timer_cb(self, _):
        self.try_enable_uv()

    # -------- helper to enable UV --------
    def try_enable_uv(self):
        if self.uv_on:
            return
        if self.last_motion_cmd == STOP_CMD:
            return
        if rospy.get_time() - self.last_unsafe < SAFE_DELAY:
            return
        self.led_pub.publish(SANITIZE_ON)
        self.uv_on = True

if __name__ == '__main__':
    try:
        LedGyroNode()
    except rospy.ROSInterruptException:
        pass