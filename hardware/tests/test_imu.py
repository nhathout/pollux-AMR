# tests imu.py

from hardware.imu import IMUController
import time
import numpy as np

def test_imu_calibration():
    imu = IMUController()
    print("\n=== IMU Calibration Test ===")
    
    # Test calibration stability
    samples = []
    for _ in range(100):
        data = imu.get_filtered_data()
        samples.append([
            data['accelerometer']['x'],
            data['accelerometer']['y'],
            data['accelerometer']['z']
        ])
        time.sleep(0.01)
    
    means = np.mean(samples, axis=0)
    stds = np.std(samples, axis=0)
    
    print(f"Accel Means (X,Y,Z): {means}")
    print(f"Accel STD (X,Y,Z): {stds}")
    
    assert np.all(stds < 0.1), "High calibration variance!"
    print("Calibration test passed!\n")

def test_imu_stream():
    imu = IMUController()
    print("=== IMU Live Stream Test ===")
    print("Press Ctrl+C to stop...")
    
    try:
        while True:
            data = imu.get_filtered_data()
            print(f"Accel: X:{data['accelerometer']['x']:6.2f}  "
                  f"Y:{data['accelerometer']['y']:6.2f}  "
                  f"Z:{data['accelerometer']['z']:6.2f} m/s²")
            print(f"Gyro:  X:{data['gyroscope']['x']:6.2f}  "
                  f"Y:{data['gyroscope']['y']:6.2f}  "
                  f"Z:{data['gyroscope']['z']:6.2f} °/s")
            print("\033[2J\033[H")  # Clear terminal
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nIMU test completed.")

if __name__ == '__main__':
    test_imu_calibration()
    test_imu_stream()