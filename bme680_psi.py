#bme680_psi
import machine
import time
from bme680 import BME680_I2C

class BME680Driver:
    def __init__(self, i2c):
        self.bme680 = BME680_I2C(i2c=i2c, refresh_rate = 1000)

    def pressure_psi(self):
        pressure_hpa = self.bme680.pressure
        pressure_psi = pressure_hpa * 0.0145038  # convert hPa to psi
        return pressure_psi
