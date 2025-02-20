# pollux-AMR/hardware/tests/test_motors.py
import time
from ..motors import DualMotorController  # relative import from parent

def test_motors():
    motor_config = {
        'left': [6, 13, 19, 26],
        'right': [12, 16, 20, 21]
    }
    motors = DualMotorController(motor_config)

    try:
        while True:
            motors.move_forward(100, 10)
            time.sleep(1)
            motors.turn_left(90, 5)
            time.sleep(1)
    except KeyboardInterrupt:
        motors.cleanup()

if __name__ == '__main__':
    test_motors()
