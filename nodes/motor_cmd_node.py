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

    if   cmd == 0:  # Forward
        # Lower speed from 900 to, say, 500 for a slower forward
        motor_ctrl.move_forward(500, 3)

    elif cmd == 1:  # Backward
        # Also slower
        motor_ctrl.move_backward(500, 3)

    elif cmd == 2:  # turn left (not used in your code, presumably)
        motor_ctrl.motor_left.rotate(0,   3)
        motor_ctrl.motor_right.rotate(1000, 3)

    elif cmd == 3:  # turn right
        motor_ctrl.motor_left.rotate(1000, 3)
        motor_ctrl.motor_right.rotate(0,   3)

    elif cmd == 4:  # spin left
        motor_ctrl.motor_left.rotate(-1200,3)
        motor_ctrl.motor_right.rotate( 1200,3)

    elif cmd == 5:  # spin right
        motor_ctrl.motor_left.rotate( 1200,3)
        motor_ctrl.motor_right.rotate(-1200,3)

    elif cmd == 6:  # stop
        pass

    elif cmd == 7:  # 180° rotate
        # Increase the step count for a bigger turn if needed
        t1 = threading.Thread(target=motor_ctrl.motor_left.rotate,  args=(2500,3))
        t2 = threading.Thread(target=motor_ctrl.motor_right.rotate, args=(2500,3))
        t1.start(); t2.start(); t1.join(); t2.join()

    elif cmd == 8:  # slight spin left
        # Increase from 200 to e.g. 1200 steps if you want a 2-second spin
        # (depends on your motor, speed, gear ratio, etc.)
        steps = 1200
        t1 = threading.Thread(target=motor_ctrl.motor_left.rotate,  args=(-steps,3))
        t2 = threading.Thread(target=motor_ctrl.motor_right.rotate, args=( steps,3))
        t1.start(); t2.start(); t1.join(); t2.join()

    elif cmd == 9:  # slight spin right
        steps = 1200
        t1 = threading.Thread(target=motor_ctrl.motor_left.rotate,  args=( steps,3))
        t2 = threading.Thread(target=motor_ctrl.motor_right.rotate, args=(-steps,3))
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