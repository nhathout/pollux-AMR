#!/usr/bin/env python3
import os, sys, rospy, RPi.GPIO as GPIO
from std_msgs.msg import Int32

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR/hardware'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

SANITIZE_LED_PIN  = 10   # UVC strip
ROBOT_ON_LED_PIN  = 9    # “power” LED
INDICATOR_LED_PIN = 11   # green indicator

SANITIZE_OFF    = 0
SANITIZE_ON     = 1
ROBOT_ON_DIM    = 2      # not used anymore, kept for compatibility
ROBOT_ON_BRIGHT = 3

class LEDController:
    def __init__(self):
        rospy.init_node('led_control_node')
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for p in (SANITIZE_LED_PIN, ROBOT_ON_LED_PIN, INDICATOR_LED_PIN):
            GPIO.setup(p, GPIO.OUT)

        # ON‑LED solid bright for whole session
        GPIO.output(ROBOT_ON_LED_PIN, GPIO.HIGH)

        rospy.Subscriber('/pollux/led_cmd', Int32, self.cb)
        rospy.loginfo("led_control_node ready (ON‑LED steady bright).")
        rospy.spin()

    def cb(self, msg):
        if msg.data == SANITIZE_ON:
            GPIO.output(SANITIZE_LED_PIN,  GPIO.HIGH)
            GPIO.output(INDICATOR_LED_PIN, GPIO.HIGH)
        elif msg.data == SANITIZE_OFF:
            GPIO.output(SANITIZE_LED_PIN,  GPIO.LOW)
            GPIO.output(INDICATOR_LED_PIN, GPIO.LOW)

if __name__ == '__main__':
    try:
        LEDController()
    finally:
        GPIO.cleanup()