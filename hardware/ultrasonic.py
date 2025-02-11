# combined sensor stuff.

import RPi.GPIO as GPIO
import time

class UltrasonicSensor:
    def __init__(self, trig_pin, echo_pin):
        self.trig = trig_pin
        self.echo = echo_pin
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)
        
    def get_distance(self):
        GPIO.output(self.trig, GPIO.LOW)
        time.sleep(0.0001)
        GPIO.output(self.trig, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trig, GPIO.LOW)
        
        pulse_start = time.time()
        while GPIO.input(self.echo) == GPIO.LOW:
            if time.time() - pulse_start > 0.1:
                return None  # Timeout
        pulse_start = time.time()
        
        pulse_end = time.time()
        while GPIO.input(self.echo) == GPIO.HIGH:
            if time.time() - pulse_end > 0.1:
                return None  # Timeout
        pulse_end = time.time()
        
        return round((pulse_end - pulse_start) * 17150, 2)

class UltrasonicArray:
    def __init__(self, sensor_config):
        self.sensors = {
            name: UltrasonicSensor(cfg['trig'], cfg['echo'])
            for name, cfg in sensor_config.items()
        }
        
    def get_distances(self):
        return {
            name: sensor.get_distance()
            for name, sensor in self.sensors.items()
        }