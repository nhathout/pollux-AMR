#!/usr/bin/env python3
import os, sys, math, rospy
from sensor_msgs.msg import Imu
from std_msgs.msg  import Int32

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

# LED command codes (match those in led_control_node)
SANITIZE_OFF    = 0
SANITIZE_ON     = 1
ROBOT_ON_BRIGHT = 3

# New thresholds
TILT_LIMIT_DEG  = 60.0       # Consider robot level if tilt is below this value
SAFE_DELAY_SECS = 0.5        # Time in seconds that the robot must remain level before re-enabling UV/Indicator
CHECK_PERIOD    = 0.5        # Timer check period

class LedGyroNode:
    def __init__(self):
        rospy.init_node('led_gyro_node')

        self.led_pub = rospy.Publisher('/pollux/led_cmd', Int32, queue_size=3)
        self.uv_on = False  # Tracks whether the UV/Indicator LEDs are on

        # Track the most recent tilt reading and when the robot was last level
        self.current_tilt_deg = None
        self.tilt_safe_since = None

        # Subscribe to the IMU topic to monitor tilt
        rospy.Subscriber('/pollux/imu', Imu, self.imu_cb)

        # At startup, ensure all LEDs are ON:
        # - Turn the "robot on" LED bright
        # - Turn on the UV (sanitize) and indicator LEDs (handled together in the control node)
        rospy.sleep(0.3)
        self.led_pub.publish(Int32(data=ROBOT_ON_BRIGHT))
        self.led_pub.publish(Int32(data=SANITIZE_ON))
        self.uv_on = True

        # Start periodic checking for tilt conditions
        rospy.Timer(rospy.Duration(CHECK_PERIOD), self.timer_cb)
        rospy.loginfo("led_gyro_node running: all LEDs are ON by default.")
        rospy.spin()

    def imu_cb(self, imu):
        # Calculate the magnitude of the acceleration vector
        la = imu.linear_acceleration
        gx, gy, gz = la.x, la.y, la.z
        g_mag = math.sqrt(gx*gx + gy*gy + gz*gz)
        if g_mag < 1e-3:
            return  # Avoid division by near-zero

        # Compute the tilt (angle between the gravity vector and the vertical)
        tilt_deg = math.degrees(math.acos(max(min(gz / g_mag, 1.0), -1.0)))
        self.current_tilt_deg = tilt_deg

        # If the robot is flipped (tilt greater than the limit), immediately mark as unsafe and turn off UV/Indicator
        if tilt_deg > TILT_LIMIT_DEG:
            self.tilt_safe_since = None
            if self.uv_on:
                rospy.logwarn_throttle(2.0, f"Tilt {tilt_deg:.0f}° > {TILT_LIMIT_DEG}° – turning off UV/Indicator")
                self.led_pub.publish(Int32(data=SANITIZE_OFF))
                self.uv_on = False
        else:
            # When level, record the time (if not already recorded)
            if self.tilt_safe_since is None:
                self.tilt_safe_since = rospy.get_time()

    def timer_cb(self, _):
        # If the UV/Indicator LEDs are off and the robot has been level for at least SAFE_DELAY_SECS,
        # then turn them back on.
        if not self.uv_on:
            if self.tilt_safe_since is not None and (rospy.get_time() - self.tilt_safe_since) >= SAFE_DELAY_SECS:
                rospy.loginfo("Robot level for %.1fs – turning UV/Indicator back ON", SAFE_DELAY_SECS)
                self.led_pub.publish(Int32(data=SANITIZE_ON))
                self.uv_on = True

if __name__ == '__main__':
    try:
        LedGyroNode()
    except rospy.ROSInterruptException:
        pass