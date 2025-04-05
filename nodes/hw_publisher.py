#!/usr/bin/env python3

import os
import sys
import time
import rospy
import RPi.GPIO as GPIO

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR/hardware'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

import imu
import ultrasonic

def main():
    rospy.init_node('hw_publisher')

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    imu_pub = rospy.Publisher('/pollux/imu', Imu, queue_size=10)
    ultrasonic_pub = rospy.Publisher('/pollux/ultrasonic', Float32MultiArray, queue_size=10)

    # Initialize IMU
    my_imu = imu.IMUController()

    # Updated sensor config for 5 sensors
    sensor_config = {
        'top_left':     {'trig': 17, 'echo': 4},
        'top_right':    {'trig': 22, 'echo': 27},
        'bottom_left':  {'trig': 23, 'echo': 18},
        'bottom_mid':   {'trig': 25, 'echo': 24},
        'bottom_right': {'trig': 7,  'echo': 8},
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
        # Store the 5 readings in a list, in a consistent order
        distance_msg.data = [
            distances.get('top_left',     -1.0),
            distances.get('top_right',    -1.0),
            distances.get('bottom_left',  -1.0),
            distances.get('bottom_mid',   -1.0),
            distances.get('bottom_right', -1.0),
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