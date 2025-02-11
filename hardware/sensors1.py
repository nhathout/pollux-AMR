import RPi.GPIO as GPIO
import time

# Define GPIO pins for HC-SR04
TRIG = 3
ECHO = 2

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Function to get the distance
def get_distance():
    # Send a pulse to trigger the sensor
    GPIO.output(TRIG, GPIO.LOW)  # Ensure TRIG is low initially
    time.sleep(0.1)
    GPIO.output(TRIG, GPIO.HIGH)  # Send trigger pulse
    time.sleep(0.00001)  # 10 microseconds pulse width
    GPIO.output(TRIG, GPIO.LOW)  # Stop sending pulse
    
    # Measure the duration of the echo pulse
    while GPIO.input(ECHO) == GPIO.LOW:
        pulse_start = time.time()
    
    while GPIO.input(ECHO) == GPIO.HIGH:
        pulse_end = time.time()

    # Calculate the distance
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound in cm/s
    distance = round(distance, 2)  # Round the distance to 2 decimal places
    return distance

try:
    while True:
        distance = get_distance()
        print(f"Distance: {distance} cm")
        time.sleep(1)  # Delay of 1 second before next measurement

except KeyboardInterrupt:
    print("Stopping and cleaning up GPIO.")
    GPIO.cleanup()