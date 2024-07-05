import machine
import time
import bluetooth
import struct
from ble_advertising import advertising_payload
from micropython import const

# Ultrasonic sensor setup
TRIG_PIN = 15
ECHO_PIN = 14

trig = machine.Pin(TRIG_PIN, machine.Pin.OUT)
echo = machine.Pin(ECHO_PIN, machine.Pin.IN)

# Relay setup
RELAY_PIN = 16
relay = machine.Pin(RELAY_PIN, machine.Pin.OUT)

# Function to measure distance
def measure_distance():
    try:
        # Ensure the trigger pin is low
        trig.value(0)
        time.sleep_us(2)

        # Send a 10us pulse to trigger
        trig.value(1)
        time.sleep_us(10)
        trig.value(0)

        # Measure the duration of the echo pulse
        pulse_start = time.ticks_us()
        while echo.value() == 0:
            pulse_start = time.ticks_us()

        pulse_end = time.ticks_us()
        while echo.value() == 1:
            pulse_end = time.ticks_us()

        # Calculate pulse duration
        pulse_duration = time.ticks_diff(pulse_end, pulse_start)

        # Calculate distance (duration in microseconds divided by 58 to get centimeters)
        distance = pulse_duration / 58.0

        return distance
    except Exception as e:
        print("Error measuring distance:", e)
        return None

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_FLAG_READ = const(0x0002)
_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)

_ULTRASONIC_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_ULTRASONIC_CHAR_UUID = bluetooth.UUID(0xFF3F)
_RELAY_CHAR_UUID = bluetooth.UUID(0xFF40)

_ULTRASONIC_CHAR = (
    _ULTRASONIC_CHAR_UUID,
    _FLAG_READ | _FLAG_NOTIFY,
)
_RELAY_CHAR = (
    _RELAY_CHAR_UUID,
    _FLAG_WRITE,
)
_ULTRASONIC_SERVICE = (
    _ULTRASONIC_UUID,
    (_ULTRASONIC_CHAR, _RELAY_CHAR),
)

class BLESimplePeripheral:
    def __init__(self, ble, name="Flex F1"):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._handle_ultrasonic, self._handle_relay),) = self._ble.gatts_register_services((_ULTRASONIC_SERVICE,))
        self._connections = set()
        self._payload = advertising_payload(name=name, services=[_ULTRASONIC_UUID])
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
                    rounded_distance = round(distance, 2)
                    distance_str = f"{rounded_distance:.2f} cm"
                    self._ble.gatts_write(self._handle_ultrasonic, distance_str.encode())
                    for conn_handle in self._connections:
                        self._ble.gatts_notify(conn_handle, self._handle_ultrasonic, distance_str.encode())
                    print(f"Sent value: {distance_str}")
        except Exception as e:
            print("Error sending distance:", e)

    def _relay_write(self, conn_handle):
        try:
            value = self._ble.gatts_read(self._handle_relay)
            if value == b'\x01':
                relay.value(1)
                print("Relay ON")
            elif value == b'\x00':
                relay.value(0)
                print("Relay OFF")
        except Exception as e:
            print("Error handling relay write:", e)

def demo():
    ble = bluetooth.BLE()
    p = BLESimplePeripheral(ble)

    while True:
        p.send_distance()
        time.sleep(0.2)

if __name__ == "__main__":
    demo()
