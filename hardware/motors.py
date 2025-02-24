# combined motor control stuff.

import RPi.GPIO as GPIO
import time
import threading

class StepperMotor:
    def __init__(self, pins, steps_per_rev=2038):
        self.pins = pins
        self.steps_per_rev = steps_per_rev
        self.step_sequence = [
            [1,0,0,0],
            [1,1,0,0],
            [0,1,0,0],
            [0,1,1,0],
            [0,0,1,0],
            [0,0,1,1],
            [0,0,0,1],
            [1,0,0,1]
        ]
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pins, GPIO.OUT)
        
    def rotate(self, steps, rpm):
        delay = 60.0 / (self.steps_per_rev * rpm) / 8
        direction = 1 if steps > 0 else -1
        steps = abs(steps)
        
        for _ in range(steps):
            step_index = _ % 8
            current_step = self.step_sequence[step_index if direction > 0 else 7 - step_index]
            for pin, state in zip(self.pins, current_step):
                GPIO.output(pin, state)
            time.sleep(delay)

class DualMotorController:
    def __init__(self, motor_pins):
        self.motor_left = StepperMotor(motor_pins['left'])
        self.motor_right = StepperMotor(motor_pins['right'])
        
    def move_forward(self, steps, rpm):
        # Run both motor rotates concurrently.
        left_thread = threading.Thread(target=self.motor_left.rotate, args=(steps, rpm))
        right_thread = threading.Thread(target=self.motor_right.rotate, args=(steps, rpm))
        left_thread.start()
        right_thread.start()
        left_thread.join()
        right_thread.join()
        
    def move_backward(self, steps, rpm):
        self.move_forward(-steps, rpm)
        
    def turn_left(self, degrees, rpm):
        steps = int(degrees * self.motor_left.steps_per_rev / 360)
        self.motor_right.rotate(steps, rpm)
        
    def turn_right(self, degrees, rpm):
        steps = int(degrees * self.motor_left.steps_per_rev / 360)
        self.motor_left.rotate(steps, rpm)
        
    def cleanup(self):
        GPIO.cleanup()