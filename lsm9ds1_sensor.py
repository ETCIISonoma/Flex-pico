# lsm9ds1_sensor.py

from machine import I2C, Pin
from lsm9ds1 import LSM9DS1
import time

class LSM9DS1Sensor:
    def __init__(self, scl_pin=21, sda_pin=20, address_gyro=0x6B, address_magnet=0x1E):
        # Initialize I2C and sensor
        self.i2c = I2C(0, scl=Pin(scl_pin), sda=Pin(sda_pin))
        self.address_gyro = address_gyro
        self.address_magnet = address_magnet

        try:
            self.lsm = LSM9DS1(self.i2c, address_gyro=self.address_gyro, address_magnet=self.address_magnet)
        except OSError as e:
            print("Failed to initialize LSM9DS1 sensor:", e)
            raise

        # Add a small delay to ensure sensor is ready
        time.sleep(1)

    def read_y_accel(self):
        mv = memoryview(self.lsm.scratch_int)
        self.lsm.i2c.readfrom_mem_into(self.lsm.address_gyro, LSM9DS1.OUT_XL + 2, mv[:2])
        return mv[0] / self.lsm.scale_accel

    def read_z_accel(self):
        mv = memoryview(self.lsm.scratch_int)
        self.lsm.i2c.readfrom_mem_into(self.lsm.address_gyro, LSM9DS1.OUT_XL + 4, mv[:2])
        return mv[0] / self.lsm.scale_accel

    @staticmethod
    def classify_orientation(accel_y, accel_z):
        if -0.4 < accel_y < 0.4:
            if accel_z > 0.5:
                return "Floor"
            elif accel_z < -0.5:
                return "Ceiling"
            else:
                return "Unknown orientation"
        elif 0.6 < abs(accel_y) < 1.4:
            return "Wall"
        else:
            return "Unknown orientation"

    def get_orientation(self):
        accel_y = self.read_y_accel()
        accel_z = self.read_z_accel()
        return self.classify_orientation(accel_y, accel_z)
