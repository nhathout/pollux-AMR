# test_ultrasonic.py
import RPi.GPIO as GPIO
import time
import ultrasonic

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
sensor = ultrasonic.UltrasonicSensor(trig_pin=23, echo_pin=18)  # Use one of your working configurations

try:
    while True:
        dist = sensor.get_distance()
        print("Distance:", dist)
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()