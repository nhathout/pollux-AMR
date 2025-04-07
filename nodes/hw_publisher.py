#!/usr/bin/env python3
import os, sys, time, rospy, RPi.GPIO as GPIO
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR/hardware'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

import imu, ultrasonic

def main():
    rospy.init_node('hw_publisher', anonymous=True)

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    imu_pub        = rospy.Publisher('/pollux/imu',        Imu,               queue_size=10)
    ultrasonic_pub = rospy.Publisher('/pollux/ultrasonic_hw', Float32MultiArray, queue_size=10)

    my_imu = imu.IMUController()

    # ⬇⬇  **ONLY the three bottom sensors**  ⬇⬇
    sensor_config = {
        'bottom_left':  {'trig': 23, 'echo': 18},
        'bottom_mid':   {'trig': 25, 'echo': 24},
        'bottom_right': {'trig': 7,  'echo': 8},
    }
    my_sensors = ultrasonic.UltrasonicArray(sensor_config)

    rate   = rospy.Rate(5)        # 5 Hz
    tick   = 0                    # for periodic debug printing

    while not rospy.is_shutdown():
        # ---- IMU ----
        data = my_imu.get_filtered_data()
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = data['accelerometer']['x']
        imu_msg.linear_acceleration.y = data['accelerometer']['y']
        imu_msg.linear_acceleration.z = data['accelerometer']['z']
        imu_msg.angular_velocity.x    = data['gyroscope']['x']
        imu_msg.angular_velocity.y    = data['gyroscope']['y']
        imu_msg.angular_velocity.z    = data['gyroscope']['z']
        imu_pub.publish(imu_msg)

        # ---- Ultrasonic ----
        distances = my_sensors.get_distances()   # dict keyed by bottom_* names
        distance_msg        = Float32MultiArray()
        distance_msg.data   = [
            distances.get('bottom_left',  -1.0),
            distances.get('bottom_mid',   -1.0),
            distances.get('bottom_right', -1.0),
        ]
        ultrasonic_pub.publish(distance_msg)

        # Optional: print every 2 s for quick debugging
        tick += 1
        if tick % 10 == 0:
            rospy.loginfo(f"[hw_publisher] bottom US: {distance_msg.data}")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()