#!/usr/bin/env python3
import os, sys, rospy, RPi.GPIO as GPIO
from std_msgs.msg import Int32

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR/hardware'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

# --- GPIO pins ---
SANITIZE_LED_PIN  = 10   # UV LED strip
ROBOT_ON_LED_PIN  = 9    # “robot on” indicator LED
INDICATOR_LED_PIN = 11   # should follow sanitize

# --- command codes ---
SANITIZE_OFF    = 0
SANITIZE_ON     = 1
ROBOT_ON_DIM    = 2   # not used anymore
ROBOT_ON_BRIGHT = 3

class LEDControllerNode:
    def __init__(self):
        rospy.init_node('led_control_node', anonymous=True)
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Set up all LED pins as outputs
        for pin in (SANITIZE_LED_PIN, ROBOT_ON_LED_PIN, INDICATOR_LED_PIN):
            GPIO.setup(pin, GPIO.OUT)

        rospy.Subscriber('/pollux/led_cmd', Int32, self.cb)
        rospy.loginfo("led_control_node ready (ON‑LED steady bright).")
        rospy.spin()

    def cb(self, msg):
        cmd = msg.data
        if   cmd == SANITIZE_ON:
            GPIO.output(SANITIZE_LED_PIN,  GPIO.HIGH)
            GPIO.output(INDICATOR_LED_PIN, GPIO.HIGH)
            rospy.loginfo("Sanitize+Indicator ON")
        elif cmd == SANITIZE_OFF:
            GPIO.output(SANITIZE_LED_PIN,  GPIO.LOW)
            GPIO.output(INDICATOR_LED_PIN, GPIO.LOW)
            rospy.loginfo("Sanitize+Indicator OFF")
        elif cmd == ROBOT_ON_DIM:
            # Not used—keep ON at full brightness
            rospy.loginfo("ROBOT_ON_DIM command ignored; using steady brightness")
        elif cmd == ROBOT_ON_BRIGHT: # should be automatically done above
            GPIO.output(ROBOT_ON_LED_PIN, GPIO.HIGH)
            rospy.loginfo("Robot On LED: BRIGHT")
        else:
            rospy.logwarn("Unknown LED cmd %d", cmd)

    def cleanup(self):
        GPIO.cleanup()

if __name__ == '__main__':
    try:
        node = LEDControllerNode()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.cleanup()