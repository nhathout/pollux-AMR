from .imu import IMUController
from .motors import DualMotorController
from .ultrasonic import UltrasonicArray
import yaml

class PolluxHardware:
    def __init__(self, config_file='/pollux-AMR/config/pins.yaml'):
        with open(config_file) as f:
            config = yaml.safe_load(f)
            
        self.imu = IMUController()
        self.motors = DualMotorController(config['motor_pins'])
        self.ultrasonic = UltrasonicArray(config['ultrasonic_pins'])
        
    def get_sensor_data(self):
        return {
            'imu': self.imu.get_filtered_data(),
            'distances': self.ultrasonic.get_distances()
        }
    
    def execute_action(self, action):
        action_map = {
            0: lambda: self.motors.move_forward(50, 10),
            1: lambda: self.motors.turn_left(45, 5),
            2: lambda: self.motors.turn_right(45, 5),
            3: self.motors.cleanup
        }
        action_map[action]()
        
    def safe_shutdown(self):
        self.motors.cleanup()
        GPIO.cleanup()