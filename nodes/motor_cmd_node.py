#!/usr/bin/env python3
import os, sys, threading, rospy
from std_msgs.msg import Int32

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR/hardware'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

import motors

def command_callback(msg):
    """
    Command mapping:
      0 → forward   1 → backward
      2 → turn left 3 → turn right
      4 → spin left 5 → spin right
      6 → stop
      7 → 180° rotate
      8 → slight spin L   9 → slight spin R
    """
    cmd = msg.data
    rospy.loginfo("motor_cmd_node got cmd %d", cmd)

    if   cmd == 0:  motor_ctrl.move_forward(900, 4)
    elif cmd == 1:  motor_ctrl.move_backward(900, 4)
    elif cmd == 2:  motor_ctrl.motor_left.rotate(0,   4); motor_ctrl.motor_right.rotate(900, 4)
    elif cmd == 3:  motor_ctrl.motor_left.rotate(900, 4); motor_ctrl.motor_right.rotate(0,   4)
    elif cmd == 4:  motor_ctrl.motor_left.rotate(-900,4); motor_ctrl.motor_right.rotate(900, 4)
    elif cmd == 5:  motor_ctrl.motor_left.rotate(900, 4); motor_ctrl.motor_right.rotate(-900,4)
    elif cmd == 6:  pass   # stop handled inside library
    elif cmd == 7:
        t1 = threading.Thread(target=motor_ctrl.motor_left.rotate,  args=(3000,4))
        t2 = threading.Thread(target=motor_ctrl.motor_right.rotate, args=(3000,4))
        t1.start(); t2.start(); t1.join(); t2.join()
    elif cmd == 8:
        t1 = threading.Thread(target=motor_ctrl.motor_left.rotate,  args=(-200,4))
        t2 = threading.Thread(target=motor_ctrl.motor_right.rotate, args=( 200,4))
        t1.start(); t2.start(); t1.join(); t2.join()
    elif cmd == 9:
        t1 = threading.Thread(target=motor_ctrl.motor_left.rotate,  args=( 200,4))
        t2 = threading.Thread(target=motor_ctrl.motor_right.rotate, args=(-200,4))
        t1.start(); t2.start(); t1.join(); t2.join()
    else:
        rospy.logwarn("Unknown motor command %d", cmd)

if __name__ == '__main__':
    rospy.init_node('motor_cmd_node')

    motor_pins = {                     
        'left':  [6, 13, 19, 26],
        'right': [12, 16, 20, 21],
    }
    motor_ctrl = motors.DualMotorController(motor_pins)

    rospy.Subscriber('/pollux/motor_cmd', Int32, command_callback)
    rospy.loginfo("motor_cmd_node ready.")
    rospy.spin()
    motor_ctrl.cleanup()
