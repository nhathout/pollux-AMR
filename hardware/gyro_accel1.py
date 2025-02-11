import mpu6050
import time

# Create a new Mpu6050 object
mpu6050 = mpu6050.mpu6050(0x68)

# Define a function to read the sensor data
def read_sensor_data():
    # Read the accelerometer values
    accelerometer_data = mpu6050.get_accel_data()

    # Read the gyroscope values
    gyroscope_data = mpu6050.get_gyro_data()

    return accelerometer_data, gyroscope_data

# Start a while loop to continuously read the sensor data
while True:

    # Read the sensor data
    accelerometer_data, gyroscope_data = read_sensor_data()

    # Print the sensor data in a more readable format
    print("Accelerometer Data:")
    print(f"  X: {accelerometer_data['x']:.2f} m/s²")
    print(f"  Y: {accelerometer_data['y']:.2f} m/s²")
    print(f"  Z: {accelerometer_data['z']:.2f} m/s²")

    print("Gyroscope Data:")
    print(f"  X: {gyroscope_data['x']:.2f} °/s")
    print(f"  Y: {gyroscope_data['y']:.2f} °/s")
    print(f"  Z: {gyroscope_data['z']:.2f} °/s")

    # Wait for 1 second
    time.sleep(1)
