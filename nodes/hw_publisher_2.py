#!/usr/bin/env python3

import os
import sys
import time
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Float32MultiArray

# --- FILL IN YOUR OWN PATHS OR REMOVE IF NOT NEEDED ---
SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR/hardware'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

# FILL IN THE BLANK: import your own ultrasonic helper if you have one
# e.g. from pollux_AMR.hardware import ultrasonic
import ultrasonic

def main():
    rospy.init_node('hw_publisher_2')  # A new node name for these additional sensors

    # --- IMPORTANT: set BCM mode BEFORE creating any UltrasonicSensor objects ---
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    sensor_config_2 = {
        'obstacle_left':  {'trig':  ###FILL_ME_IN###, 'echo':  ###FILL_ME_IN###},
        'obstacle_right': {'trig':  ###FILL_ME_IN###, 'echo':  ###FILL_ME_IN###},
    }

    # Create an instance of your ultrasonic array or sensor class
    my_extra_sensors = ultrasonic.UltrasonicArray(sensor_config_2)

    # Create a new publisher for the additional ultrasonic data
    ultrasonic_2_pub = rospy.Publisher('/pollux/ultrasonic_2', Float32MultiArray, queue_size=10)

    rate = rospy.Rate(5)  # e.g. 5 Hz

    while not rospy.is_shutdown():
        distances_2 = my_extra_sensors.get_distances()
        distance_msg_2 = Float32MultiArray()

        # distance_msg_2.data can contain two measurements, e.g. [left, right]
        # or you can keep them in a dictionary and then place them in a list:
        distance_msg_2.data = [
            distances_2.get('obstacle_left',  -1.0) or -1.0,
            distances_2.get('obstacle_right', -1.0) or -1.0,
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
