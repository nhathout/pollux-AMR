#!/usr/bin/env python3
import os, sys, math, rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

# LED command codes (should match the led_control_node)
SANITIZE_OFF    = 0
SANITIZE_ON     = 1
ROBOT_ON_BRIGHT = 3

# Threshold constants
TILT_LIMIT_DEG  = 60.0      # Safe if tilt is below 60°
ANG_VEL_LIMIT   = 5.0       # Safe if angular velocity magnitude is below 5.0 (units typically rad/s)
SAFE_DELAY_SECS = 0.5       # Must be safe for 0.5 s before enabling the LED
CHECK_PERIOD    = 0.5       # Timer callback period

class LedGyroNode:
    def __init__(self):
        rospy.init_node('led_gyro_node')
        self.led_pub = rospy.Publisher('/pollux/led_cmd', Int32, queue_size=3)

        # Tracks the current state of the UV/indicator LED (True = ON, False = OFF)
        self.uv_on = False

        # Variables for safe condition checking
        self.current_tilt_deg = None
        self.tilt_safe_since = None

        # Subscribe to IMU topic to get linear acceleration and angular velocity data
        rospy.Subscriber('/pollux/imu', Imu, self.imu_cb)

        # At startup, publish the "robot on" command once and try to enable UV/indicator LEDs.
        rospy.sleep(0.3)
        self.led_pub.publish(Int32(data=ROBOT_ON_BRIGHT))
        # Attempt to turn on UV/indicator; this will be managed by our safe checks.
        self.update_uv_state(True)

        rospy.Timer(rospy.Duration(CHECK_PERIOD), self.timer_cb)
        rospy.loginfo("led_gyro_node running: using tilt and angular velocity for LED safety.")
        rospy.spin()

    def update_uv_state(self, desired_state):
        """
        Publish a new command for the UV/indicator LEDs only if the state is changing.
        desired_state: True to turn them ON, False to turn them OFF.
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
        # Compute tilt angle using the linear acceleration vector.
        la = imu.linear_acceleration
        gx, gy, gz = la.x, la.y, la.z
        g_mag = math.sqrt(gx*gx + gy*gy + gz*gz)
        if g_mag < 1e-3:
            return  # Avoid division by a near-zero value

        tilt_deg = math.degrees(math.acos(max(min(gz / g_mag, 1.0), -1.0)))
        self.current_tilt_deg = tilt_deg

        # Compute the magnitude of the angular velocity vector.
        av = imu.angular_velocity
        av_mag = math.sqrt(av.x**2 + av.y**2 + av.z**2)

        # Check safe condition: both tilt and angular velocity must be below thresholds.
        if tilt_deg < TILT_LIMIT_DEG and av_mag < ANG_VEL_LIMIT:
            if self.tilt_safe_since is None:
                self.tilt_safe_since = rospy.get_time()
        else:
            # Unsafe condition: reset safe timer and turn off UV/indicator LED if needed.
            self.tilt_safe_since = None
            self.update_uv_state(False)
            rospy.logwarn_throttle(2.0, f"Unsafe condition: tilt {tilt_deg:.1f}° (limit {TILT_LIMIT_DEG}°), " +
                                               f"angular velocity {av_mag:.1f} (limit {ANG_VEL_LIMIT})")

    def timer_cb(self, _):
        # If we've been in a safe state for at least SAFE_DELAY_SECS, enable the UV/indicator LED.
        if self.tilt_safe_since is not None and (rospy.get_time() - self.tilt_safe_since) >= SAFE_DELAY_SECS:
            self.update_uv_state(True)

if __name__ == '__main__':
    try:
        LedGyroNode()
    except rospy.ROSInterruptException:
        pass