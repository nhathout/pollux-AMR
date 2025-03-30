#!/usr/bin/env python3

import os
import sys
import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Int32

# If needed, dynamically add pollux-AMR/hardware to PYTHONPATH
SCRIPT_DIR = os.path.dirname(__file__)
POLLUX_AMR_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '../../pollux-AMR/hardware'))
if POLLUX_AMR_DIR not in sys.path:
    sys.path.insert(0, POLLUX_AMR_DIR)

# --------------------
SANITIZE_LED_PIN    = 5   
ROBOT_ON_LED_PIN    = 6  
INDICATOR_LED_PIN   = 13  

SANITIZE_OFF        = 0
SANITIZE_ON         = 1
ROBOT_ON_DIM        = 2
ROBOT_ON_BRIGHT     = 3
INDICATOR_ON        = 4
INDICATOR_OFF       = 5

class LEDControllerNode:
    def __init__(self):
        rospy.init_node('led_control_node', anonymous=True)
        rospy.loginfo("led_control_node started. Subscribing to /pollux/led_cmd.")

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Initialize LED pins as output
        GPIO.setup(SANITIZE_LED_PIN, GPIO.OUT)
        GPIO.setup(ROBOT_ON_LED_PIN, GPIO.OUT)
        GPIO.setup(INDICATOR_LED_PIN, GPIO.OUT)

        # Optionally use PWM for “dim” vs “bright” states on the Robot-On LED:
        self.robot_on_led_pwm = GPIO.PWM(ROBOT_ON_LED_PIN, 1000)  # 1kHz frequency
        self.robot_on_led_pwm.start(0)  # start with 0% duty cycle (off)

        # Subscribe to the LED command topic
        self.led_sub = rospy.Subscriber('/pollux/led_cmd', Int32, self.led_cmd_callback)

        rospy.loginfo("LED pins initialized. Ready to receive LED commands.")

    def led_cmd_callback(self, msg):
        """
        This callback receives a single Int32 command and
        toggles/adjusts the LEDs accordingly.
        """
        cmd = msg.data
        rospy.loginfo("led_control_node received cmd: %d", cmd)

        if cmd == SANITIZE_OFF:
            GPIO.output(SANITIZE_LED_PIN, GPIO.LOW)
            rospy.loginfo("Sanitization LED turned OFF.")

        elif cmd == SANITIZE_ON:
            GPIO.output(SANITIZE_LED_PIN, GPIO.HIGH)
            rospy.loginfo("Sanitization LED turned ON.")

        elif cmd == ROBOT_ON_DIM:
            # Example: 30% duty cycle for “dim”
            self.robot_on_led_pwm.ChangeDutyCycle(30.0)
            rospy.loginfo("Robot On LED: DIM.")

        elif cmd == ROBOT_ON_BRIGHT:
            # Example: 100% duty cycle for full brightness
            self.robot_on_led_pwm.ChangeDutyCycle(100.0)
            rospy.loginfo("Robot On LED: BRIGHT.")

        elif cmd == INDICATOR_ON:
            GPIO.output(INDICATOR_LED_PIN, GPIO.HIGH)
            rospy.loginfo("Indicator LED turned ON.")

        elif cmd == INDICATOR_OFF:
            GPIO.output(INDICATOR_LED_PIN, GPIO.LOW)
            rospy.loginfo("Indicator LED turned OFF.")

        else:
            rospy.logwarn("Unknown LED command received: %d", cmd)

    def run(self):
        rospy.spin()

    def cleanup(self):
        rospy.loginfo("Cleaning up LED GPIO pins.")
        self.robot_on_led_pwm.stop()
        GPIO.cleanup()


if __name__ == '__main__':
    try:
        node = LEDControllerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        node.cleanup()
