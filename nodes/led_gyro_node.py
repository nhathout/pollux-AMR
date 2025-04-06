#!/usr/bin/env python3
import os, sys, math, rospy
from sensor_msgs.msg import Imu
from std_msgs.msg  import Int32

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

# LED commands
SANITIZE_OFF = 0
SANITIZE_ON  = 1

STOP_CMD     = 6   # from motor_cmd_node

# If |gz| / |g| < 0.5  → tilt > ~60°
TILT_RATIO_LIMIT = 0.50
TILT_HOLD_SECS   = 0.3   # must stay tilted this long
SAFE_DELAY       = 1.0   # need this long of level before UV can re‑enable

class LedGyroNode:
    def __init__(self):
        rospy.init_node('led_gyro_node')

        self.led_pub      = rospy.Publisher('/pollux/led_cmd', Int32, queue_size=2)
        self.last_motion  = STOP_CMD
        self.uv_on        = False
        self.tilt_start   = None        # None while level
        self.last_unsafe  = rospy.get_time()

        rospy.Subscriber('/pollux/motor_cmd', Int32, self.motor_cb)
        rospy.Subscriber('/pollux/imu',        Imu,  self.imu_cb)

        rospy.Timer(rospy.Duration(0.5), self.timer_cb)
        rospy.loginfo("led_gyro_node ready.")
        rospy.spin()

    # ---------------- IMU ----------------
    def imu_cb(self, imu):
        la = imu.linear_acceleration
        g  = math.sqrt(la.x*la.x + la.y*la.y + la.z*la.z)
        if g < 1e-3:
            return                      # bad packet
        ratio = abs(la.z) / g           # 1 when upright, 0 when flat

        now = rospy.get_time()
        if ratio < TILT_RATIO_LIMIT:    # tilted
            if self.tilt_start is None:
                self.tilt_start = now
            elif (now - self.tilt_start) >= TILT_HOLD_SECS:
                self.last_unsafe = now
                if self.uv_on:
                    self.led_pub.publish(SANITIZE_OFF)
                    self.uv_on = False
        else:
            self.tilt_start = None      # reset

    # ------------- motor cmd -------------
    def motor_cb(self, msg):
        self.last_motion = msg.data
        if msg.data == STOP_CMD and self.uv_on:
            self.led_pub.publish(SANITIZE_OFF)
            self.uv_on = False
        elif msg.data != STOP_CMD:
            self.try_enable_uv()

    # ---------- periodic re‑enable -------
    def timer_cb(self, _):
        self.try_enable_uv()

    def try_enable_uv(self):
        if self.uv_on:                       return
        if self.last_motion == STOP_CMD:     return
        if rospy.get_time() - self.last_unsafe < SAFE_DELAY:
            return
        self.led_pub.publish(SANITIZE_ON)
        self.uv_on = True

if __name__ == '__main__':
    try:
        LedGyroNode()
    except rospy.ROSInterruptException:
        pass