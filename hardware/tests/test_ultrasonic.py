# pollux-AMR/hardware/tests/test_ultrasonic.py
import time
import yaml
from ..ultrasonic import UltrasonicArray
import os

def test_ultrasonic_sensors():
    print("=== Ultrasonic Sensor Test ===")
    
    # Build absolute path to pins.yaml
    current_dir = os.path.dirname(__file__)  # .../hardware/tests
    pins_path = os.path.abspath(os.path.join(current_dir, '../../config/pins.yaml'))
    with open(pins_path) as f:
        config = yaml.safe_load(f)
    
    array = UltrasonicArray(config['ultrasonic_pins'])
    
    try:
        while True:
            distances = array.get_distances()
            for name, dist in distances.items():
                print(f"{name.replace('_', ' ').title():<12}: {dist or 'N/A':>5} cm")

            valid_ranges = all(2 < d < 400 for d in distances.values() if d)
            assert valid_ranges, "Invalid distance readings!"

            time.sleep(0.5)
            print("\033[2J\033[H", end='')  # Clear terminal
    except KeyboardInterrupt:
        print("\nUltrasonic test completed.")

if __name__ == '__main__':
    test_ultrasonic_sensors()
