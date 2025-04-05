#!/usr/bin/env python3

import os
import sys
import time
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Float32MultiArray

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR/hardware'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

import ultrasonic

def main():
    rospy.init_node('hw_publisher_2', anonymous=True)

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    # If you only want top_left & top_right for "obstacle" detection,
    # for example:
    sensor_config_2 = {
        'obstacle_left':  {'trig': 17, 'echo': 4},
        'obstacle_right': {'trig': 22, 'echo': 27},
    }

    my_extra_sensors = ultrasonic.UltrasonicArray(sensor_config_2)
    ultrasonic_2_pub = rospy.Publisher('/pollux/ultrasonic_2', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        distances_2 = my_extra_sensors.get_distances()
        distance_msg_2 = Float32MultiArray()
        distance_msg_2.data = [
            distances_2.get('obstacle_left',  -1.0),
            distances_2.get('obstacle_right', -1.0),
        ]
        ultrasonic_2_pub.publish(distance_msg_2)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
