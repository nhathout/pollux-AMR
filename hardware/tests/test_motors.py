import os
import sys
import time

# Dynamically add the parent directory (../) to sys.path
# so Python can import hardware.*
SCRIPT_DIR = os.path.dirname(__file__)           # .../hardware/tests
PARENT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..'))  # .../hardware
if PARENT_DIR not in sys.path:
    sys.path.insert(0, PARENT_DIR)

from hardware.motors import DualMotorController

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
