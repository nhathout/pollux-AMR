# tests ultrasonic.py

from hardware.ultrasonic import UltrasonicArray
import time
import yaml

def test_ultrasonic_sensors():
    print("=== Ultrasonic Sensor Test ===")
    
    # Load config
    with open('../config/pins.yaml') as f: # might have to fix config file or location
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
            
            print("\033[2J\033[H")  # Clear terminal
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nUltrasonic test completed.")

if __name__ == '__main__':
    test_ultrasonic_sensors()