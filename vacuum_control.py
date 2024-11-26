# vacuum_control.py

from machine import Pin
import time

class VacuumControl:
    def __init__(self, pin):
        # Assuming GPIO16 is used for controlling the relay
        self.relay_pin = Pin(pin, Pin.OUT)

    def start_pump(self):
        self.relay_pin.off()  # Turn on the relay to start the pump

    def stop_pump(self):
        self.relay_pin.on()  # Turn off the relay to stop the pump
