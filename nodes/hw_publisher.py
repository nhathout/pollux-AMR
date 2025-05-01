#!/usr/bin/env python3
"""
hw_publisher.py
Publishes:
  • /pollux/imu              (sensor_msgs/Imu)
  • /pollux/ultrasonic_hw    (std_msgs/Float32MultiArray)

Now bullet-proof against occasional bad/garbled distance readings.
"""

import os, sys, time, math, rospy, RPi.GPIO as GPIO
import numpy as np
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

SCRIPT_DIR     = os.path.dirname(__file__)
HW_DIR         = os.path.abspath(os.path.join(SCRIPT_DIR,
                                              '../../pollux-AMR/hardware'))
if HW_DIR not in sys.path:
    sys.path.insert(0, HW_DIR)

import imu, ultrasonic   # ← your custom HW drivers

# ───────────────────────────────────────────────────────────────────
def safe_float(x, default=-1.0):
    """Convert x→float32.  If it fails or is not finite, return default."""
    try:
        f = float(x)
        if not math.isfinite(f):
            raise ValueError
        return np.float32(f)
    except Exception:
        return np.float32(default)

# ───────────────────────────────────────────────────────────────────
def main():
    rospy.init_node('hw_publisher', anonymous=True)

    GPIO.setmode(GPIO.BCM); GPIO.setwarnings(False)

    imu_pub = rospy.Publisher('/pollux/imu', Imu, queue_size=10)
    us_pub  = rospy.Publisher('/pollux/ultrasonic_hw',
                              Float32MultiArray, queue_size=10)

    my_imu  = imu.IMUController()

    sensor_cfg = {
        'bottom_left':  {'trig': 23, 'echo': 18},
        'bottom_mid':   {'trig': 25, 'echo': 24},
        'bottom_right': {'trig': 7,  'echo': 8},
    }
    my_us = ultrasonic.UltrasonicArray(sensor_cfg)

    rate, tick = rospy.Rate(5), 0     # 5 Hz

    while not rospy.is_shutdown():
        # ---------- IMU ----------
        data = my_imu.get_filtered_data()
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = data['accelerometer']['x']
        imu_msg.linear_acceleration.y = data['accelerometer']['y']
        imu_msg.linear_acceleration.z = data['accelerometer']['z']
        imu_msg.angular_velocity.x    = data['gyroscope']['x']
        imu_msg.angular_velocity.y    = data['gyroscope']['y']
        imu_msg.angular_velocity.z    = data['gyroscope']['z']
        imu_pub.publish(imu_msg)

        # ---------- Ultrasonic ----------
        d = my_us.get_distances()           # dict
        dist_msg       = Float32MultiArray()
        dist_msg.data  = [
            safe_float(d.get('bottom_left')),
            safe_float(d.get('bottom_mid')),
            safe_float(d.get('bottom_right')),
        ]
        us_pub.publish(dist_msg)

        # periodic console print
        tick += 1
        if tick % 10 == 0:
            rospy.loginfo(f"[hw_publisher] bottom US: {list(dist_msg.data)}")

        rate.sleep()

# ───────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()