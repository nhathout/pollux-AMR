#!/usr/bin/env python3
import os, sys, math, rospy
from sensor_msgs.msg import Imu
from std_msgs.msg  import Int32

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

# ----- LED commands ----------------------------------------------------------
SANITIZE_OFF    = 0
SANITIZE_ON     = 1
ROBOT_ON_BRIGHT = 3

# ----- Motor command we treat as "not moving / cleaning paused" --------------
STOP_CMD        = 6

# ----- Safety thresholds -----------------------------------------------------
TILT_LIMIT_DEG        = 25.0      # > 25 deg from vertical  ⇒ unsafe
ANG_VEL_LIMIT_DEG_S   = 200.0     # > 200 deg/s            ⇒ unsafe
UNSAFE_CLEAR_SECS     = 2.0       # need this long of calm to re‑enable
CHECK_PERIOD          = 0.5       # timer period

class LedGyroNode:
    def __init__(self):
        rospy.init_node('led_gyro_node', anonymous=True)

        self.led_pub   = rospy.Publisher('/pollux/led_cmd', Int32, queue_size=5)
        self.last_motion_cmd = STOP_CMD
        self.sanitize_active = False     # what LEDs are actually doing
        self.desired_clean   = False     # robot is supposed to be cleaning
        self.last_unsafe_t   = 0.0

        rospy.Subscriber('/pollux/motor_cmd', Int32, self.motor_cb)
        rospy.Subscriber('/pollux/imu',        Imu,  self.imu_cb)

        # Bright ON‑LED once everything is ready
        rospy.sleep(0.3)
        self.led_pub.publish(ROBOT_ON_BRIGHT)

        # periodic check to (re)enable sanitize when safe
        rospy.Timer(rospy.Duration(CHECK_PERIOD), self.timer_cb)

        rospy.loginfo("led_gyro_node started.")
        rospy.spin()

    # --------------------------------------------------------------------- IMU
    def imu_cb(self, imu: Imu):
        """Set last_unsafe_t whenever tilt or jerk exceeds limits."""
        now = rospy.get_time()

        # -------- angular‑velocity jerk check --------
        av = imu.angular_velocity
        max_deg_s = max(abs(av.x), abs(av.y), abs(av.z)) * 180.0 / math.pi
        if max_deg_s > ANG_VEL_LIMIT_DEG_S:
            self.last_unsafe_t = now
            rospy.logwarn_throttle(2.0,
                f"Unsafe jerk ({max_deg_s:.0f} deg/s) – sanitize OFF")

        # -------- tilt check using gravity vector --------
        la = imu.linear_acceleration
        gx, gy, gz = la.x, la.y, la.z
        g_mag = math.sqrt(gx*gx + gy*gy + gz*gz)
        if g_mag > 0.05:                         # ignore near‑zero vector
            tilt_deg = math.degrees(math.acos(min(max(gz / g_mag, -1.0), 1.0)))
            if tilt_deg > TILT_LIMIT_DEG:
                self.last_unsafe_t = now
                rospy.logwarn_throttle(2.0,
                    f"Tilt {tilt_deg:.0f}° – sanitize OFF")

        # -------- enforce LED state immediately if unsafe --------
        if now - self.last_unsafe_t < 0.1 and self.sanitize_active:
            self.led_pub.publish(SANITIZE_OFF)
            self.sanitize_active = False

    # ---------------------------------------------------------- motor command
    def motor_cb(self, msg: Int32):
        self.last_motion_cmd = msg.data
        self.desired_clean   = (msg.data != STOP_CMD)

        # if robot resumes motion and we are already safe, turn LEDs on
        if self.desired_clean and not self.sanitize_active \
           and self._safe_for(0.0):
            self.led_pub.publish(SANITIZE_ON)
            self.sanitize_active = True

        # if STOP command and LEDs are on, turn them off
        if msg.data == STOP_CMD and self.sanitize_active:
            self.led_pub.publish(SANITIZE_OFF)
            self.sanitize_active = False

    # --------------------------------------------------------------- timer
    def timer_cb(self, _):
        """Every 0.5 s: if robot is safe & should be cleaning, ensure LEDs ON."""
        if self.desired_clean and not self.sanitize_active \
           and self._safe_for(UNSAFE_CLEAR_SECS):
            rospy.loginfo("Robot level & calm – sanitize back ON")
            self.led_pub.publish(SANITIZE_ON)
            self.sanitize_active = True

    # --------------------------------------------------------- helper
    def _safe_for(self, secs: float) -> bool:
        """True if IMU has been calm for >= secs seconds."""
        return (rospy.get_time() - self.last_unsafe_t) >= secs

if __name__ == '__main__':
    try:
        LedGyroNode()
    except rospy.ROSInterruptException:
        pass