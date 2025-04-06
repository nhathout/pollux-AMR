#!/usr/bin/env python3
import os, sys, rospy, math
from sensor_msgs.msg import Imu
from std_msgs.msg  import Int32

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR/hardware'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

# --- LED commands ---
SANITIZE_OFF    = 0
SANITIZE_ON     = 1
ROBOT_ON_BRIGHT = 3

# --- motor command we care about ---
STOP_CMD = 6

# --- thresholds ---
ANG_VEL_DEG_S  = 50.0     # tilt / jerk threshold
DEBOUNCE_SECS  = 2.0

class LedGyroNode:
    def __init__(self):
        rospy.init_node('led_gyro_node', anonymous=True)

        self.led_pub   = rospy.Publisher('/pollux/led_cmd', Int32, queue_size=10)
        self.last_tilt = 0.0
        self.sanitize  = False

        # Subscribe to IMU and motor commands
        rospy.Subscriber('/pollux/imu',        Imu,  self.imu_cb)
        rospy.Subscriber('/pollux/motor_cmd',  Int32, self.motor_cb)

        # Make sure ON‑LED is bright even if control node started first
        rospy.sleep(0.3)                       # tiny delay so publisher is ready
        self.led_pub.publish(ROBOT_ON_BRIGHT)

        rospy.loginfo("led_gyro_node running.")
        rospy.spin()

    # ---------- motor command callback ----------
    def motor_cb(self, msg):
        if msg.data == STOP_CMD:
            if self.sanitize:
                self.led_pub.publish(SANITIZE_OFF)
                self.sanitize = False
        else:
            if not self.sanitize:
                self.led_pub.publish(SANITIZE_ON)
                self.sanitize = True

    # ---------- IMU callback ----------
    def imu_cb(self, imu):
        now = rospy.get_time()
        if now - self.last_tilt < DEBOUNCE_SECS:
            return

        # rad/s → deg/s
        av = imu.angular_velocity
        ax = abs(av.x) * 180.0 / math.pi
        ay = abs(av.y) * 180.0 / math.pi
        az = abs(av.z) * 180.0 / math.pi

        if max(ax, ay, az) > ANG_VEL_DEG_S:
            rospy.logwarn("Tilt/jerk detected (%.0f/%.0f/%.0f deg/s) → Sanitize OFF", ax, ay, az)
            self.led_pub.publish(SANITIZE_OFF)
            self.sanitize  = False
            self.last_tilt = now

if __name__ == '__main__':
    try:
        LedGyroNode()
    except rospy.ROSInterruptException:
        pass