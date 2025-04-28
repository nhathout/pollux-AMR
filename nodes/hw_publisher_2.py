#!/usr/bin/env python3

"""
This ROS node reads distance data from two forward-facing ultrasonic sensors
and publishes the measurements to the topic /pollux/ultrasonic_2 at 5Hz using
a Float32MultiArray message. The sensor pins are configured through a dictionary, and
readings are accessed via a reusable UltrasonicArray class from ultrasonic module.
"""

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

    sensor_config_2 = {
        'top_left':  {'trig': 17, 'echo': 4},
        'top_right': {'trig': 22, 'echo': 27},
    }

    my_extra_sensors = ultrasonic.UltrasonicArray(sensor_config_2)
    ultrasonic_2_pub = rospy.Publisher('/pollux/ultrasonic_2', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(5)  # 5 Hz
    tick = 0

    while not rospy.is_shutdown():
        distances_2 = my_extra_sensors.get_distances()  # returns dict keyed by 'top_left' and 'top_right'
        
        distance_msg_2 = Float32MultiArray()
        distance_msg_2.data = [
            distances_2.get('top_left',  -1.0),
            distances_2.get('top_right', -1.0),
        ]
        ultrasonic_2_pub.publish(distance_msg_2)

        tick += 1
        # Optional debug print every 2s (5 Hz => 10 cycles = 2s)
        if tick % 10 == 0:
            rospy.loginfo(f"[hw_publisher_2] front US: {distance_msg_2.data}")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()