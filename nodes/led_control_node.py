#!/usr/bin/env python3
import os, sys, rospy, RPi.GPIO as GPIO
from std_msgs.msg import Int32

SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR/hardware'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

# --- GPIO pins ---
SANITIZE_LED_PIN  = 10
ROBOT_ON_LED_PIN  = 9
INDICATOR_LED_PIN = 11

# --- command codes ---
SANITIZE_OFF    = 0
SANITIZE_ON     = 1
ROBOT_ON_DIM    = 2
ROBOT_ON_BRIGHT = 3
# 4 / 5 are no longer needed externally because indicator follows sanitize

class LEDControllerNode:
    def __init__(self):
        rospy.init_node('led_control_node', anonymous=True)
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for pin in (SANITIZE_LED_PIN, ROBOT_ON_LED_PIN, INDICATOR_LED_PIN):
            GPIO.setup(pin, GPIO.OUT)

        # --- PWM for ON‑LED ---
        self.on_pwm = GPIO.PWM(ROBOT_ON_LED_PIN, 500)   # 500 Hz
        self.on_pwm.start(100)                          # 100 % = BRIGHT

        rospy.Subscriber('/pollux/led_cmd', Int32, self.cb)
        rospy.loginfo("led_control_node ready (ON‑LED bright).")

    # ---------- callback ----------
    def cb(self, msg):
        cmd = msg.data
        if   cmd == SANITIZE_ON:
            GPIO.output(SANITIZE_LED_PIN, GPIO.HIGH)
            GPIO.output(INDICATOR_LED_PIN, GPIO.HIGH)
            rospy.loginfo("Sanitize+Indicator ON")
        elif cmd == SANITIZE_OFF:
            GPIO.output(SANITIZE_LED_PIN, GPIO.LOW)
            GPIO.output(INDICATOR_LED_PIN, GPIO.LOW)
            rospy.loginfo("Sanitize+Indicator OFF")
        elif cmd == ROBOT_ON_DIM:
            self.on_pwm.ChangeDutyCycle(30)             # 30 % = DIM
        elif cmd == ROBOT_ON_BRIGHT:
            self.on_pwm.ChangeDutyCycle(100) 
        else:
            rospy.logwarn("Unknown LED cmd %d", cmd)

    def run(self):
        rospy.spin()

    def __del__(self):
        self.on_pwm.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    LEDControllerNode().run()