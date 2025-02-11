import RPi.GPIO as GPIO
import time

""" MOTOR 1 """

IN11 = 6
IN12 = 13 
IN13 = 19
IN14 = 26

IN21 = 12
IN22 = 16 
IN23 = 20
IN24 = 21

step_sequence = [
    [1,0,0,0],
    [1,1,0,0],
    [0,1,0,0],
    [0,1,1,0],
    [0,0,1,0],
    [0,0,1,1],
    [0,0,0,1],
    [1,0,0,1]
]

steps_per_revolution = 2038

GPIO.setmode(GPIO.BCM)
GPIO.setup([IN11, IN12, IN13, IN14],GPIO.OUT)
GPIO.setup([IN21, IN22, IN23, IN24],GPIO.OUT)

def set_step(step):
    GPIO.output(IN11, step[0])
    GPIO.output(IN12, step[1])
    GPIO.output(IN13, step[2])
    GPIO.output(IN14, step[3])
    
    GPIO.output(IN21, step[0])
    GPIO.output(IN22, step[1])
    GPIO.output(IN23, step[2])
    GPIO.output(IN24, step[3])


def rotate(steps, rpm):
    delay = 60.0 / (steps_per_revolution * rpm) / 8
    for _ in range(abs(steps)):
        step_index = _ % 8
        set_step(step_sequence[step_index if steps > 0 else -step_index])
        time.sleep(delay)

try:
    while True:
        rotate(steps_per_revolution, 5)
        time.sleep(1)
        
        rotate(-steps_per_revolution, 10)
        time.sleep(1)

except KeyboardInterrupt:
    print("Stoping and cleaning up GPIO.")
    GPIO.cleanup()
    

