#!/usr/bin/env python3
import os, sys, math, rospy
from sensor_msgs.msg import Imu
from std_msgs.msg  import Int32

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR/hardware'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

# LED commands
SANITIZE_OFF    = 0
SANITIZE_ON     = 1
ROBOT_ON_BRIGHT = 3

# Motor commands
STOP_CMD        = 6

# Tilt / jerk detection
ANG_VEL_LIMIT_DEG_S = 120.0      # less sensitive
MOTION_MIN_TIME     = 0.25       # must exceed limit for ≥ this long
QUIET_TIME_REENABLE = 3.0        # seconds of calm before auto‑re‑enable
DEBOUNCE_SECS       = 1.0

class LedGyroNode:
    def __init__(self):
        rospy.init_node('led_gyro_node', anonymous=True)

        self.led_pub    = rospy.Publisher('/pollux/led_cmd', Int32, queue_size=5)
        self.sanitize   = False
        self.last_tilt  = 0.0
        self.tilt_start = None
        self.last_motion_cmd = STOP_CMD

        rospy.Subscriber('/pollux/imu',       Imu,  self.imu_cb)
        rospy.Subscriber('/pollux/motor_cmd', Int32, self.motor_cb)

        # Bright ON‑LED on boot
        rospy.sleep(0.3)
        self.led_pub.publish(ROBOT_ON_BRIGHT)

        # Timer to auto‑re‑enable sanitize after calm
        rospy.Timer(rospy.Duration(0.5), self.reenable_timer)

        rospy.loginfo("led_gyro_node ready.")
        rospy.spin()

    # ---------- motor cmd ----------
    def motor_cb(self, msg):
        self.last_motion_cmd = msg.data
        if msg.data == STOP_CMD:
            if self.sanitize:
                self.led_pub.publish(SANITIZE_OFF)
                self.sanitize = False
        else:
            if not self.sanitize:
                self.led_pub.publish(SANITIZE_ON)
                self.sanitize = True

    # ---------- IMU ----------
    def imu_cb(self, imu):
        av = imu.angular_velocity
        deg_s = max(abs(av.x), abs(av.y), abs(av.z)) * 180.0 / math.pi

        now = rospy.get_time()
        if deg_s > ANG_VEL_LIMIT_DEG_S:
            # start or continue a tilt interval
            if self.tilt_start is None:
                self.tilt_start = now
            elif (now - self.tilt_start) >= MOTION_MIN_TIME:
                if now - self.last_tilt > DEBOUNCE_SECS:
                    rospy.logwarn("Tilt/jerk detected (%.0f deg/s) – sanitize OFF", deg_s)
                    self.led_pub.publish(SANITIZE_OFF)
                    self.sanitize  = False
                    self.last_tilt = now
        else:
            self.tilt_start = None   # reset counter

    # ---------- timer to re‑enable ----------
    def reenable_timer(self, _):
        if self.sanitize:
            return
        # no sanitize right now
        if (rospy.get_time() - self.last_tilt) < QUIET_TIME_REENABLE:
            return
        if self.last_motion_cmd != STOP_CMD:
            rospy.loginfo("IMU calm – sanitize back ON")
            self.led_pub.publish(SANITIZE_ON)
            self.sanitize = True

if __name__ == '__main__':
    try:
        LedGyroNode()
    except rospy.ROSInterruptException:
        pass