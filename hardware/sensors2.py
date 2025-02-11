import RPi.GPIO as GPIO
import time

# Define GPIO pins for HC-SR04 sensors
# Sensor 1
TRIG1 = 3
ECHO1 = 2
# Sensor 2
TRIG2 = 27
ECHO2 = 17
# Sensor 3
TRIG3 = 9
ECHO3 = 10
# Sensor 4
TRIG4 = 24
ECHO4 = 23

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup([TRIG1, TRIG2, TRIG3, TRIG4], GPIO.OUT)
GPIO.setup([ECHO1, ECHO2, ECHO3, ECHO4], GPIO.IN)

# Function to get the distance for each sensor
def get_distance(trig_pin, echo_pin):
    # Send a pulse to trigger the sensor
    GPIO.output(trig_pin, GPIO.LOW)  # Ensure TRIG is low initially
    time.sleep(0.1)
    GPIO.output(trig_pin, GPIO.HIGH)  # Send trigger pulse
    time.sleep(0.00001)  # 10 microseconds pulse width
    GPIO.output(trig_pin, GPIO.LOW)  # Stop sending pulse
    
    # Measure the duration of the echo pulse
    while GPIO.input(echo_pin) == GPIO.LOW:
        pulse_start = time.time()
    
    while GPIO.input(echo_pin) == GPIO.HIGH:
        pulse_end = time.time()

    # Calculate the distance
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound in cm/s
    distance = round(distance, 2)  # Round the distance to 2 decimal places
    return distance

try:
    while True:
        for i, (trig, echo) in enumerate([(TRIG1, ECHO1), (TRIG2, ECHO2), (TRIG3, ECHO3), (TRIG4, ECHO4)], 1):
            distance = get_distance(trig, echo)
            print(f"Sensor {i}: Distance = {distance} cm")
            time.sleep(1)  # Delay of 1 second before next measurement
        
except KeyboardInterrupt:
    print("Stopping and cleaning up GPIO.")
    GPIO.cleanup()
