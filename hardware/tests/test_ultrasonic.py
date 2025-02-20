import os
import sys
import time
import yaml

# Dynamically add ../ to sys.path so Python can import hardware.*
SCRIPT_DIR = os.path.dirname(__file__)          # .../hardware/tests
PARENT_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..'))  # .../hardware
if PARENT_DIR not in sys.path:
    sys.path.insert(0, PARENT_DIR)

from hardware.ultrasonic import UltrasonicArray

def test_ultrasonic_sensors():
    print("=== Ultrasonic Sensor Test ===")
    
    # Build an absolute path to ../../config/pins.yaml, so we can open it
    CONFIG_PATH = os.path.abspath(
        os.path.join(SCRIPT_DIR, '../../config/pins.yaml')
    )
    with open(CONFIG_PATH) as f:
        config = yaml.safe_load(f)
    
    array = UltrasonicArray(config['ultrasonic_pins'])
    
    try:
        while True:
            distances = array.get_distances()
            for name, dist in distances.items():
                print(f"{name.replace('_', ' ').title():<12}: {dist or 'N/A':>5} cm")
            
            # Validate readings
            valid_ranges = all(2 < d < 400 for d in distances.values() if d)
            assert valid_ranges, "Invalid distance readings!"
            
            time.sleep(0.5)
            # Clear terminal for a neat output
            print("\033[2J\033[H", end='')
            
    except KeyboardInterrupt:
        print("\nUltrasonic test completed.")

if __name__ == '__main__':
    test_ultrasonic_sensors()