import os
import sys
import time
import random
import rospy
from std_msgs.msg import Float32MultiArray, Int32

# Dynamically add pollux-AMR/hardware to PYTHONPATH
SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR/hardware'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)


# Parameters
ANGULAR_VELOCITY_THRESHOLD = 50         # degrees per second; if exceeded, assume the robot is being tilted upside down
DEBOUNCE_DURATION = 3.0     # seconds to ignore new events after one is handled

# Motor and LED command codes
STOP_CMD = 0                # Stop motors
LED_ALERT_CMD = 1           # Command to signal LED to turn on
LED_OFF_CMD = 2             # Command to turn LED off


class led_gyro_node:
    def __init__(self):
        rospy.init_node('led_gyro_node', anonymous=True)
        rospy.loginfo("led_gyro_node started. Subscribing to /pollux/imu")

        # subscribe to gyroscope sensor data
        self.imu_sub = rospy.Subscriber('/pollux/imu',Float32MultiArray, self.imu_callback)

        # Publishers for motor and LED commands
        self.cmd_pub = rospy.Publisher('/pollux/motor_cmd', Int32, queue_size=10)
        self.led_pub = rospy.Publisher('/pollux/led_cmd', Int32, queue_size=10)

        # State variables
        self.last_event_time = 0.0
        self.in_action = False

    def gyroscope_callback(self, msg):
        if self.in_action:
            return

        current_time = rospy.get_time()
        if current_time - self.last_event_time < DEBOUNCE_DURATION:
            return


        #check with noah if this is how to receive the velocities from the imu
        angular_velocities = msg.data  # Expecting [X, Y, Z] values
        if len(angular_velocities) != 3:
            rospy.logwarn("Unexpected gyroscope data format!")
            return