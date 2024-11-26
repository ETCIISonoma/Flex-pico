import time
from machine import I2C, Pin

class IRControl:
    def __init__(self, i2c=None, address=0x29):
        if i2c is None:
            i2c = I2C(0, scl=Pin(17), sda=Pin(16), freq=100000)
        self.i2c = i2c
        self._address = address
        self.init_sensor()

    def write_register(self, register, value):
        """Write a byte to the specified 16-bit register."""
        self.i2c.writeto_mem(self._address, register, bytearray([value]), addrsize=16)

    def read_register(self, register):
        """Read a byte from the specified 16-bit register."""
        value = int.from_bytes(self.i2c.readfrom_mem(self._address, register, 1, addrsize=16), 'big')
        return value

    def init_sensor(self):
        """Initialize the sensor with the recommended settings from the datasheet."""
        if self.read_register(0x0016) != 1:
            raise RuntimeError("Failed to reset sensor")

        # Initialization sequence
        settings = [
            (0x0207, 0x01), (0x0208, 0x01), (0x0096, 0x00), (0x0097, 0xfd),
            (0x00e3, 0x00), (0x00e4, 0x04), (0x00e5, 0x02), (0x00e6, 0x01),
            (0x00e7, 0x03), (0x00f5, 0x02), (0x00d9, 0x05), (0x00db, 0xce),
            (0x00dc, 0x03), (0x00dd, 0xf8), (0x009f, 0x00), (0x00a3, 0x3c),
            (0x00b7, 0x00), (0x00bb, 0x3c), (0x00b2, 0x09), (0x00ca, 0x09),
            (0x0198, 0x01), (0x01b0, 0x17), (0x01ad, 0x00), (0x00ff, 0x05),
            (0x0100, 0x05), (0x0199, 0x05), (0x01a6, 0x1b), (0x01ac, 0x3e),
            (0x01a7, 0x1f), (0x0030, 0x00)
        ]
        for reg, val in settings:
            self.write_register(reg, val)

    def range_mm(self):
        """Measure the distance in millimeters."""
        self.write_register(0x0018, 0x01)  # Start ranging
        time.sleep(0.01)
        return self.read_register(0x0062)  # Read range value

    def onSurface(self):
        """Check if the distance is less than 2 inches (approximately 50.8 mm)."""
        distance_mm = self.range_mm()
        return distance_mm < 50.8
