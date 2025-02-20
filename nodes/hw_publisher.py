#!/usr/bin/env python3

import os
import sys
import time
import rospy
import RPi.GPIO as GPIO  # <-- CRITICAL: Import RPi.GPIO

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

# Dynamically add pollux-AMR/hardware to PYTHONPATH
SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR/hardware'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

import imu
import ultrasonic

def main():
    rospy.init_node('hw_publisher')

    # --- IMPORTANT: set BCM mode BEFORE creating any UltrasonicSensor objects ---
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)  # optional

    imu_pub = rospy.Publisher('/pollux/imu', Imu, queue_size=10)
    ultrasonic_pub = rospy.Publisher('/pollux/ultrasonic', Float32MultiArray, queue_size=10)

    # Initialize your IMU
    my_imu = imu.IMUController()

    # Inlined sensor config; if you want to load pins.yaml, do so here
    sensor_config = {
        'front_left':  {'trig': 14, 'echo': 15},
        'front_right': {'trig': 27, 'echo': 17},
        'rear_left':   {'trig': 9,  'echo': 10},
        'rear_right':  {'trig': 24, 'echo': 23},
    }
    my_sensors = ultrasonic.UltrasonicArray(sensor_config)

    rate = rospy.Rate(5)  # 5 Hz

    while not rospy.is_shutdown():
        # --- IMU Reading ---
        data = my_imu.get_filtered_data()
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = data['accelerometer']['x']
        imu_msg.linear_acceleration.y = data['accelerometer']['y']
        imu_msg.linear_acceleration.z = data['accelerometer']['z']
        imu_msg.angular_velocity.x = data['gyroscope']['x']
        imu_msg.angular_velocity.y = data['gyroscope']['y']
        imu_msg.angular_velocity.z = data['gyroscope']['z']
        imu_pub.publish(imu_msg)

        # --- Ultrasonic Reading ---
        distances = my_sensors.get_distances()
        distance_msg = Float32MultiArray()
        distance_msg.data = [
            distances.get('front_left',  -1.0) or -1.0,
            distances.get('front_right', -1.0) or -1.0,
            distances.get('rear_left',   -1.0) or -1.0,
            distances.get('rear_right',  -1.0) or -1.0,
        ]
        ultrasonic_pub.publish(distance_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()

