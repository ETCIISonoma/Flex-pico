import machine
import time
import bluetooth
import struct
from ble_advertising import advertising_payload
from vl6180x import Sensor
from micropython import const

# VL6180 sensor setup
SCA_PIN = 14
SCL_PIN = 15

# Define I2C pins and initialize I2C interface with custom I2C1 pins
i2c = machine.I2C(1, scl=machine.Pin(SCL_PIN), sda=machine.Pin(SCA_PIN))

# Initialize the VL6180X sensor
sensor = Sensor(i2c)

# Relay setup
RELAY_PIN = 16
relay = machine.Pin(RELAY_PIN, machine.Pin.OUT)
relay.value(1)

# Function to measure distance
def measure_distance():
    try:
        # Read the range in millimeters
        range_mm = sensor.range()
        return range_mm
    except Exception as e:
        print("Error measuring distance:", e)
        return None


# BLE configs
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_FLAG_READ = const(0x0002)
_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)

_IR_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_IR_CHAR_UUID = bluetooth.UUID(0xFF3F)
_RELAY_CHAR_UUID = bluetooth.UUID(0xFF40)

_IR_CHAR = (
    _IR_CHAR_UUID,
    _FLAG_READ | _FLAG_NOTIFY,
)
_RELAY_CHAR = (
    _RELAY_CHAR_UUID,
    _FLAG_WRITE,
)
_IR_SERVICE = (
    _IR_UUID,
    (_IR_CHAR, _RELAY_CHAR),
)

class BLESimplePeripheral:
    def __init__(self, ble, name="Flex F1"):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._handle_ir, self._handle_relay),) = self._ble.gatts_register_services((_IR_SERVICE,))
        self._connections = set()
        self._payload = advertising_payload(name=name, services=[_IR_UUID])
        self._advertise()

    def _irq(self, event, data):
        try:
            if event == _IRQ_CENTRAL_CONNECT:
                conn_handle, _, _ = data
                print("New connection", conn_handle)
                self._connections.add(conn_handle)
            elif event == _IRQ_CENTRAL_DISCONNECT:
                conn_handle, _, _ = data
                print("Disconnected", conn_handle)
                self._connections.remove(conn_handle)
                self._advertise()
            elif event == _IRQ_GATTS_WRITE:
                conn_handle, attr_handle = data
                if attr_handle == self._handle_relay:
                    self._relay_write(conn_handle)
        except Exception as e:
            print("Error in IRQ handler:", e)

    def _advertise(self, interval_us=500000):
        try:
            print("Starting advertising")
            self._ble.gap_advertise(interval_us, adv_data=self._payload)
        except Exception as e:
            print("Error starting advertising:", e)

    def send_distance(self):
        try:
            if self._connections:
                distance = measure_distance()
                if distance is not None:
                    rounded_distance = round(distance / 10, 2)
                    distance_str = f"{rounded_distance:.2f} cm"
                    self._ble.gatts_write(self._handle_ir, distance_str.encode())
                    for conn_handle in self._connections:
                        self._ble.gatts_notify(conn_handle, self._handle_ir, distance_str.encode())
                    # print(f"Sent value: {distance_str}")
        except Exception as e:
            print("Error sending distance:", e)

    def _relay_write(self, conn_handle):
        try:
            value = self._ble.gatts_read(self._handle_relay)
            if value == b'\x01':
                relay.value(0)
                print("Relay ON")
            elif value == b'\x00':
                relay.value(1)
                print("Relay OFF")
        except Exception as e:
            print("Error handling relay write:", e)

def main():
    ble = bluetooth.BLE()
    p = BLESimplePeripheral(ble)

    while True:
        p.send_distance()
        time.sleep(0.2)  # Increase sleep time to ensure BLE operations have enough time

if __name__ == "__main__":
    main()
