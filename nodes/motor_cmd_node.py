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
      0 → forward        1 → backward
      2 → turn left      3 → turn right
      4 → spin left      5 → spin right
      6 → stop
      7 → 180° rotate
      8 → slight spin L  9 → slight spin R
    """
    cmd = msg.data
    rospy.loginfo("motor_cmd_node got cmd %d", cmd)

    if cmd == 0:  # Forward
        # Move forward at speed=3 with step=500 each call
        motor_ctrl.move_forward(500, 3)

    elif cmd == 1:  # Backward
        # Move backward at speed=3 with step=500 each call
        motor_ctrl.move_backward(500, 3)

    elif cmd == 2:  # turn left (threaded approach)
        # e.g. ~90° left turn using 1250 steps on each motor:
        # left motor negative, right motor positive => pivot
        steps = 1250
        t1 = threading.Thread(target=motor_ctrl.motor_left.rotate,  args=(-steps, 3))
        t2 = threading.Thread(target=motor_ctrl.motor_right.rotate, args=( steps, 3))
        t1.start(); t2.start()
        t1.join();  t2.join()

    elif cmd == 3:  # turn right (threaded approach)
        # Mirror of cmd=2
        steps = 1250
        t1 = threading.Thread(target=motor_ctrl.motor_left.rotate,  args=( steps, 3))
        t2 = threading.Thread(target=motor_ctrl.motor_right.rotate, args=(-steps, 3))
        t1.start(); t2.start()
        t1.join();  t2.join()

    elif cmd == 4:  # spin left
        # For a "spin," both motors rotate in opposite directions, possibly bigger steps
        steps = 1800  # pick a larger step for a bigger spin
        t1 = threading.Thread(target=motor_ctrl.motor_left.rotate,  args=(-steps, 3))
        t2 = threading.Thread(target=motor_ctrl.motor_right.rotate, args=( steps, 3))
        t1.start(); t2.start()
        t1.join();  t2.join()

    elif cmd == 5:  # spin right
        steps = 1800
        t1 = threading.Thread(target=motor_ctrl.motor_left.rotate,  args=( steps, 3))
        t2 = threading.Thread(target=motor_ctrl.motor_right.rotate, args=(-steps, 3))
        t1.start(); t2.start()
        t1.join();  t2.join()

    elif cmd == 6:  # stop
        # The motors library may have a .stop() method
        # that explicitly stops all motion. We'll call it if it exists:
        pass

    elif cmd == 7:  # 180° rotate
        # We'll keep your original "smooth" 180 approach
        t1 = threading.Thread(target=motor_ctrl.motor_left.rotate,  args=(2500, 3))
        t2 = threading.Thread(target=motor_ctrl.motor_right.rotate, args=(2500, 3))
        t1.start(); t2.start()
        t1.join();  t2.join()

    elif cmd == 8:  # slight spin left
        steps = 1200
        t1 = threading.Thread(target=motor_ctrl.motor_left.rotate,  args=(-steps, 3))
        t2 = threading.Thread(target=motor_ctrl.motor_right.rotate, args=( steps, 3))
        t1.start(); t2.start()
        t1.join();  t2.join()

    elif cmd == 9:  # slight spin right
        steps = 1200
        t1 = threading.Thread(target=motor_ctrl.motor_left.rotate,  args=( steps, 3))
        t2 = threading.Thread(target=motor_ctrl.motor_right.rotate, args=(-steps, 3))
        t1.start(); t2.start()
        t1.join();  t2.join()

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