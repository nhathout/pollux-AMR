#!/usr/bin/env python3
import os, sys, math, rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

# LED command codes (should match those used by led_control_node)
SANITIZE_OFF    = 0
SANITIZE_ON     = 1
ROBOT_ON_BRIGHT = 3

# Thresholds and timing constants
TILT_LIMIT_DEG      = 165.0      # Robot is considered safe if tilt is below 60°
ANG_VEL_LIMIT       = 10.0      # Robot is considered safe if angular velocity magnitude is below 10.0 (e.g., rad/s)
SAFE_DELAY_SECS     = 0.5      # Must be safe for 0.5 s before enabling the UV/indicator LED
UNSAFE_DELAY_SECS   = 0.3       # Must be unsafe for 0.3 s before turning off the UV/indicator LED
CHECK_PERIOD        = 1      # Timer callback period (more frequent for hysteresis)

class LedGyroNode:
    def __init__(self):
        rospy.init_node('led_gyro_node')
        self.led_pub = rospy.Publisher('/pollux/led_cmd', Int32, queue_size=3)

        # UV/indicator LED state: True means ON, False means OFF.
        # We assume the UV LED should be on at startup.
        self.uv_on = None

        # Hysteresis timers: record when safe or unsafe condition was first observed.
        self.safe_since = None
        self.unsafe_since = None

        # For debugging: track the most recent tilt value.
        self.current_tilt_deg = None

        # Subscribe to the IMU data.
        rospy.Subscriber('/pollux/imu', Imu, self.imu_cb)

        # At startup, send the robot_on_bright command once.
        #rospy.sleep(0.5)
        #self.led_pub.publish(Int32(data=ROBOT_ON_BRIGHT))
        #rospy.loginfo("Sent ROBOT_ON_BRIGHT command.")

        # Set the initial UV/indicator LED state to ON.
        self.uv_on = True
        self.led_pub.publish(Int32(data=SANITIZE_ON))
        rospy.loginfo("Initial UV/Indicator LED turned ON.")

        # Start a frequent timer to check safe/unsafe conditions.
        rospy.Timer(rospy.Duration(CHECK_PERIOD), self.timer_cb)
        rospy.loginfo("led_gyro_node running: using tilt and angular velocity with hysteresis.")
        rospy.spin()

    def update_uv_state(self, desired_state):
        """Publish a new LED command only if the state needs to change."""
        if desired_state and not self.uv_on:
            self.led_pub.publish(Int32(data=SANITIZE_ON))
            self.uv_on = True
            rospy.loginfo("UV/Indicator LEDs turned ON.")
        elif not desired_state and self.uv_on:
            self.led_pub.publish(Int32(data=SANITIZE_OFF))
            self.uv_on = False
            rospy.loginfo("UV/Indicator LEDs turned OFF.")

    def imu_cb(self, imu):
        # Compute tilt from the linear acceleration vector.
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

        # Determine safe condition: both tilt and angular velocity must be below their limits.
        safe = (tilt_deg < TILT_LIMIT_DEG) and (av_mag < ANG_VEL_LIMIT)

        if safe:
            # Clear any unsafe timer and start (or continue) the safe timer.
            self.unsafe_since = None
            if self.safe_since is None:
                self.safe_since = rospy.get_time()
        else:
            # Clear safe timer and start (or continue) the unsafe timer.
            self.safe_since = None
            if self.unsafe_since is None:
                self.unsafe_since = rospy.get_time()
            rospy.logwarn_throttle(2.0, f"Unsafe condition: tilt {tilt_deg:.1f}° (limit {TILT_LIMIT_DEG}°), " +
                                               f"angular velocity {av_mag:.1f} (limit {ANG_VEL_LIMIT})")

    def timer_cb(self, event):
        now = rospy.get_time()
        # If safe condition has been maintained long enough, ensure UV LED is ON.
        if self.safe_since is not None and (now - self.safe_since >= SAFE_DELAY_SECS):
            self.update_uv_state(True)
        # If unsafe condition has been maintained long enough, ensure UV LED is OFF.
        if self.unsafe_since is not None and (now - self.unsafe_since >= UNSAFE_DELAY_SECS):
            self.update_uv_state(False)

if __name__ == '__main__':
    try:
        LedGyroNode()
    except rospy.ROSInterruptException:
        pass