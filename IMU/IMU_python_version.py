import smbus2
import time

# Constants for the MPU-9250 I2C address
MPU9250_ADDRESS = 0x68

# Register addresses
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

# Initialize the I2C bus
bus = smbus2.SMBus(1)

def write_byte(reg, value):
    bus.write_byte_data(MPU9250_ADDRESS, reg, value)

def read_i2c_word(reg):
    # High byte
    high = bus.read_byte_data(MPU9250_ADDRESS, reg)
    # Low byte
    low = bus.read_byte_data(MPU9250_ADDRESS, reg + 1)
    # Combine into a signed word
    value = (high << 8) + low
    if value >= 0x8000:
        return -((65535 - value) + 1)
    else:
        return value

def setup():
    # Wake up the MPU-9250 since it starts in sleep mode
    write_byte(PWR_MGMT_1, 0x00)
    # Set accelerometer configuration (e.g., range)
    write_byte(ACCEL_CONFIG, 0x18)
    # Set gyroscope configuration (e.g., scale)
    write_byte(GYRO_CONFIG, 0x18)
    # Set sample rate divider
    write_byte(SMPLRT_DIV, 0x07)
    # Set configuration (e.g., low pass filter)
    write_byte(CONFIG, 0x06)
    time.sleep(0.1)

def read_acceleration():
    ax = read_i2c_word(ACCEL_XOUT_H)
    ay = read_i2c_word(ACCEL_XOUT_H + 2)
    az = read_i2c_word(ACCEL_XOUT_H + 4)
    return ax, ay, az

def read_gyroscope():
    gx = read_i2c_word(GYRO_XOUT_H)
    gy = read_i2c_word(GYRO_XOUT_H + 2)
    gz = read_i2c_word(GYRO_XOUT_H + 4)
    return gx, gy, gz

def main():
    setup()
    while True:
        ax, ay, az = read_acceleration()
        gx, gy, gz = read_gyroscope()
        print("Acceleration: Ax={0}, Ay={1}, Az={2}".format(ax, ay, az))
        print("Gyroscope: Gx={0}, Gy={1}, Gz={2}".format(gx, gy, gz))
        time.sleep(0.05)

if __name__ == "__main__":
    main()
