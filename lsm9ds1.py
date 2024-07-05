from machine import I2C, Pin

# LSM9DS1 register addresses and constants
LSM9DS1_ADDRESS = 0x6B
LSM9DS1_WHO_AM_I = 0x0F
LSM9DS1_CTRL_REG1_G = 0x10
LSM9DS1_CTRL_REG6_XL = 0x20
LSM9DS1_OUT_X_L_G = 0x18
LSM9DS1_OUT_X_L_XL = 0x28

class LSM9DS1:
    def __init__(self, i2c):
        self.i2c = i2c

    def write_byte(self, reg, value):
        self.i2c.writeto_mem(LSM9DS1_ADDRESS, reg, bytes([value]))

    def read_byte(self, reg):
        return self.i2c.readfrom_mem(LSM9DS1_ADDRESS, reg, 1)[0]

    def read_bytes(self, reg, count):
        return self.i2c.readfrom_mem(LSM9DS1_ADDRESS, reg, count)

    def init_imu(self):
        # Check WHO_AM_I register
        who_am_i = self.read_byte(LSM9DS1_WHO_AM_I)
        if who_am_i != 0x68:
            print("LSM9DS1 not found!")
            return False

        # Initialize gyroscope
        self.write_byte(LSM9DS1_CTRL_REG1_G, 0xC0)  # 119 Hz, 2000 dps

        # Initialize accelerometer
        self.write_byte(LSM9DS1_CTRL_REG6_XL, 0xC0)  # 119 Hz, +/- 2g

        return True

    def read_gyro(self):
        data = self.read_bytes(LSM9DS1_OUT_X_L_G, 6)
        gx = (data[1] << 8 | data[0])
        gy = (data[3] << 8 | data[2])
        gz = (data[5] << 8 | data[4])
        return gx, gy, gz

    def read_accel(self):
        data = self.read_bytes(LSM9DS1_OUT_X_L_XL, 6)
        ax = (data[1] << 8 | data[0])
        ay = (data[3] << 8 | data[2])
        az = (data[5] << 8 | data[4])
        return ax, ay, az
