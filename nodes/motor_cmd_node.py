#!/usr/bin/env python3

import os
import sys
import threading
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
      0 => Move forward (both motors forward)
      1 => Move backward (both motors backward)
      2 => Turn left  (left motor off, right motor forward)
      3 => Turn right (right motor off, left motor forward)
      4 => Spin left  (left motor backward, right motor forward)
      5 => Spin right (left motor forward, right motor backward)
      6 => Stop (no movement)
      7 => Rotate 180° (both motors spin in opposite directions concurrently)
      8 => Slight spin left (small left adjustment)
      9 => Slight spin right (small right adjustment)
    """
    command = msg.data
    rospy.loginfo("motor_cmd_node received command: %d", command)

    if command == 0:
        rospy.loginfo("Moving forward ~3s.")
        motor_ctrl.move_forward(900, 4)
    elif command == 1:
        rospy.loginfo("Moving backward ~3s.")
        motor_ctrl.move_backward(900, 4)
    elif command == 2:
        rospy.loginfo("Turning left ~3s. (left motor off, right motor forward)")
        motor_ctrl.motor_left.rotate(0, 4)
        motor_ctrl.motor_right.rotate(900, 4)
    elif command == 3:
        rospy.loginfo("Turning right ~3s. (right motor off, left motor forward)")
        motor_ctrl.motor_left.rotate(900, 4)
        motor_ctrl.motor_right.rotate(0, 4)
    elif command == 4:
        rospy.loginfo("Spinning left ~3s. (left motor backward, right motor forward)")
        motor_ctrl.motor_left.rotate(-900, 4)
        motor_ctrl.motor_right.rotate(900, 4)
    elif command == 5:
        rospy.loginfo("Spinning right ~3s. (left motor forward, right motor backward)")
        motor_ctrl.motor_left.rotate(900, 4)
        motor_ctrl.motor_right.rotate(-900, 4)
    elif command == 6:
        rospy.loginfo("STOP command received. Stopping motors.")
        # Implement stop logic if needed; here, we simply do nothing.
        pass
    elif command == 7:
        rospy.loginfo("Rotating 180° (~3s).")
        left_thread = threading.Thread(target=motor_ctrl.motor_left.rotate, args=(3000, 4))
        right_thread = threading.Thread(target=motor_ctrl.motor_right.rotate, args=(3000, 4))
        left_thread.start()
        right_thread.start()
        left_thread.join()
        right_thread.join()
    elif command == 8:
        rospy.loginfo("Slight spin left (~1.5s).")
        left_thread = threading.Thread(target=motor_ctrl.motor_left.rotate, args=(-200, 4))
        right_thread = threading.Thread(target=motor_ctrl.motor_right.rotate, args=(200, 4))
        left_thread.start()
        right_thread.start()
        left_thread.join()
        right_thread.join()
    elif command == 9:
        rospy.loginfo("Slight spin right (~1.5s).")
        left_thread = threading.Thread(target=motor_ctrl.motor_left.rotate, args=(200, 4))
        right_thread = threading.Thread(target=motor_ctrl.motor_right.rotate, args=(-200, 4))
        left_thread.start()
        right_thread.start()
        left_thread.join()
        right_thread.join()
    else:
        rospy.logwarn("Unknown motor command: %d", command)

if __name__ == '__main__':
    rospy.init_node('motor_cmd_node')

    # Updated pins for new prototype
    motor_pins = {
        'left':  [19, 26, 13, 6],   # IN1/IN2 swapped
        'right': [20, 21, 16, 12],  # IN1/IN2 swapped
    }   

    motor_ctrl = motors.DualMotorController(motor_pins)

    rospy.Subscriber('/pollux/motor_cmd', Int32, command_callback)

    rospy.loginfo("motor_cmd_node started; waiting for /pollux/motor_cmd messages.")
    rospy.spin()

    rospy.loginfo("Cleaning up GPIO pins.")
    motor_ctrl.cleanup()
