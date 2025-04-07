import RPi.GPIO as GPIO
import time

class UltrasonicSensor:
    def __init__(self, trig_pin, echo_pin):
        self.trig = trig_pin
        self.echo = echo_pin
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)
        
    def get_distance(self):
        # Ensure trigger is low initially
        GPIO.output(self.trig, GPIO.LOW)
        time.sleep(0.0001)
        # Send a 10µs pulse.
        GPIO.output(self.trig, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trig, GPIO.LOW)
        
        # Wait for echo to go HIGH and record the start time.
        start_time = time.time()
        while GPIO.input(self.echo) == GPIO.LOW:
            if time.time() - start_time > 0.1:
                return None  # Timeout waiting for echo
        pulse_start = time.time()
        
        # Wait for echo to go LOW and record the end time.
        while GPIO.input(self.echo) == GPIO.HIGH:
            if time.time() - pulse_start > 0.1:
                return None  # Timeout during echo
        pulse_end = time.time()
        
        # Calculate distance: (time difference * speed of sound in cm/s) / 2.
        distance = round((pulse_end - pulse_start) * 17150, 2)
        return distance

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