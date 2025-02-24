#!/usr/bin/env python3

import os
import sys
import rospy
from std_msgs.msg import Int32

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR/hardware'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

import motors

def command_callback(msg):
    """
    Command mapping:
      0 => Move forward
      1 => Move backward
      2 => Turn left  (left motor OFF, right motor forward)
      3 => Turn right (right motor OFF, left motor forward)
      4 => Spin left  (left motor backward, right motor forward)
      5 => Spin right (left motor forward, right motor backward)
      6 => Stop
    """
    command = msg.data
    rospy.loginfo("motor_cmd_node received command: %d", command)

    if command == 0:
        # Move forward ~3 sec
        rospy.loginfo("Moving forward ~3s.")
        motor_ctrl.move_forward(1000, 10)  # steps=1000, rpm=10
    elif command == 1:
        # Move backward
        rospy.loginfo("Moving backward ~3s.")
        motor_ctrl.move_backward(1000, 10)
    elif command == 2:
        # Turn left
        rospy.loginfo("Turning left ~3s. (left off, right forward)")
        # left off => rotate(0), right => forward 1000 steps
        motor_ctrl.motor_left.rotate(0, 10)   # do nothing
        motor_ctrl.motor_right.rotate(1000, 10)
    elif command == 3:
        # Turn right
        rospy.loginfo("Turning right ~3s. (right off, left forward)")
        motor_ctrl.motor_left.rotate(1000, 10)
        motor_ctrl.motor_right.rotate(0, 10)
    elif command == 4:
        # Spin left
        rospy.loginfo("Spinning left ~3s. (left backward, right forward)")
        motor_ctrl.motor_left.rotate(-1000, 10)
        motor_ctrl.motor_right.rotate(1000, 10)
    elif command == 5:
        # Spin right
        rospy.loginfo("Spinning right ~3s. (left forward, right backward)")
        motor_ctrl.motor_left.rotate(1000, 10)
        motor_ctrl.motor_right.rotate(-1000, 10)
    elif command == 6:
        # Stop
        rospy.loginfo("STOP command received. Stopping motors.")
        pass
    else:
        rospy.logwarn("Unknown motor command: %d", command)

if __name__ == '__main__':
    rospy.init_node('motor_cmd_node')

    # Initialize motor pins
    motor_pins = {
        'left': [6, 13, 19, 26],
        'right': [12, 16, 20, 21]
    }
    motor_ctrl = motors.DualMotorController(motor_pins)

    rospy.Subscriber('/pollux/motor_cmd', Int32, command_callback)

    rospy.loginfo("motor_cmd_node started; waiting for /pollux/motor_cmd messages.")
    rospy.spin()

    # On shutdown
    rospy.loginfo("Cleaning up GPIO pins.")
    motor_ctrl.cleanup()
