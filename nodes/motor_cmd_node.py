#!/usr/bin/env python3

import os
import sys
import rospy
from std_msgs.msg import Int32

# 1) Insert pollux-AMR/hardware into sys.path
SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR/hardware'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

# 2) Import the motors code
import motors

def command_callback(msg):
    """
    Example interpretation of motor commands:
    0 => Move forward
    1 => Turn left
    2 => Turn right
    3 => Move backward
    """
    if msg.data == 0:
        motor_ctrl.move_forward(50, 10)
    elif msg.data == 1:
        motor_ctrl.turn_left(90, 5)
    elif msg.data == 2:
        motor_ctrl.turn_right(90, 5)
    elif msg.data == 3:
        motor_ctrl.move_backward(50, 10)
    else:
        rospy.logwarn("Unknown motor command: %d", msg.data)

if __name__ == '__main__':
    rospy.init_node('motor_cmd_node')

    # Initialize motor pins
    motor_pins = {
        'left': [6, 13, 19, 26],
        'right': [12, 16, 20, 21]
    }
    motor_ctrl = motors.DualMotorController(motor_pins)

    # Subscribe for integer motor commands
    rospy.Subscriber('/pollux/motor_cmd', Int32, command_callback)

    rospy.loginfo("motor_cmd_node started; waiting for /pollux/motor_cmd messages.")
    rospy.spin()

    # Cleanup
    motor_ctrl.cleanup()

