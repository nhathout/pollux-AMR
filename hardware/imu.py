# gyro/accelerometer stuff

import mpu6050
import time

class IMUController:
    def __init__(self, address=0x68):
        self.sensor = mpu6050.mpu6050(0x68)
        self.calibration_samples = 100
        self._calibrate()
        
    def _calibrate(self):
        """Calculate average offsets from initial readings"""
        accel_sum = {'x': 0, 'y': 0, 'z': 0}
        gyro_sum = {'x': 0, 'y': 0, 'z': 0}
        
        for _ in range(self.calibration_samples):
            accel = self.sensor.get_accel_data()
            gyro = self.sensor.get_gyro_data()
            for axis in ['x', 'y', 'z']:
                accel_sum[axis] += accel[axis]
                gyro_sum[axis] += gyro[axis]
            time.sleep(0.01)
            
        self.accel_offset = {k: v/self.calibration_samples for k, v in accel_sum.items()}
        self.gyro_offset = {k: v/self.calibration_samples for k, v in gyro_sum.items()}
    
    def get_filtered_data(self):
        """Return calibrated sensor readings"""
        raw_accel = self.sensor.get_accel_data()
        raw_gyro = self.sensor.get_gyro_data()
        
        return {
            'accelerometer': {
                'x': raw_accel['x'] - self.accel_offset['x'],
                'y': raw_accel['y'] - self.accel_offset['y'],
                'z': raw_accel['z'] - self.accel_offset['z']
            },
            'gyroscope': {
                'x': raw_gyro['x'] - self.gyro_offset['x'],
                'y': raw_gyro['y'] - self.gyro_offset['y'],
                'z': raw_gyro['z'] - self.gyro_offset['z']
            }
        }