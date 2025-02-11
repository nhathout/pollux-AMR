import RPi.GPIO as GPIO
from time import sleep

class PolluxHardware:
    def __init__(self):
        # Motor setup
        GPIO.setmode(GPIO.BCM)
        self.motor_pins = [17, 18, 22, 23]
        for pin in self.motor_pins:
            GPIO.setup(pin, GPIO.OUT)
        
        # Sensor setup
        self.ultrasonic_trigger = 24
        self.ultrasonic_echo = 25
        GPIO.setup(self.ultrasonic_trigger, GPIO.OUT)
        GPIO.setup(self.ultrasonic_echo, GPIO.IN)

    def get_distance(self):
        # Ultrasonic measurement logic
        pass

    def move_forward(self, duration=0.5):
        # Motor control logic
        pass

    def cleanup(self):
        GPIO.cleanup()