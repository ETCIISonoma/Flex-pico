import machine
import time
import bluetooth
import struct
from machine import I2C, Pin
from ble_advertising import advertising_payload
from micropython import const
from motor_control import MotorControl
from IR_control import IRControl
from vacuum_control import VacuumControl
from bme680_psi import *

# BLE constants for events and flags
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_FLAG_READ = const(0x0002)
_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)

# BLE UUIDs for the service and characteristics
_MOTOR_CONTROL_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_MOTOR_TORQUE_CHAR_UUID = bluetooth.UUID(0xFF3F)
_SUCTION_STATUS_CHAR_UUID = bluetooth.UUID(0xFF40)
_BUS_VOLTAGE_CHAR_UUID = bluetooth.UUID(0xFF41)
_MOTOR_POWER_CHAR_UUID = bluetooth.UUID(0xFF42)
_REP_COUNT_CHAR_UUID = bluetooth.UUID(0xFF43)  # New characteristic UUID for rep count

# Characteristics definitions
_MOTOR_TORQUE_CHAR = (_MOTOR_TORQUE_CHAR_UUID, _FLAG_WRITE)
_SUCTION_STATUS_CHAR = (_SUCTION_STATUS_CHAR_UUID, _FLAG_WRITE | _FLAG_READ | _FLAG_NOTIFY)
_BUS_VOLTAGE_CHAR = (_BUS_VOLTAGE_CHAR_UUID, _FLAG_READ | _FLAG_NOTIFY)
_MOTOR_POWER_CHAR = (_MOTOR_POWER_CHAR_UUID, _FLAG_READ | _FLAG_NOTIFY)
_REP_COUNT_CHAR = (_REP_COUNT_CHAR_UUID, _FLAG_READ | _FLAG_NOTIFY)  # Define the rep count characteristic

# Service definition
_MOTOR_CONTROL_SERVICE = (
    _MOTOR_CONTROL_UUID,
    (_MOTOR_TORQUE_CHAR, _SUCTION_STATUS_CHAR, _BUS_VOLTAGE_CHAR, _MOTOR_POWER_CHAR, _REP_COUNT_CHAR),  # Add the rep count characteristic
)

class FlexHandler:
    def __init__(self, ble = bluetooth.BLE(), name="Flex F1"):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)  # Set up the IRQ handler for BLE events
        ((self._handle_motor_torque, self._handle_suction_status,
          self._handle_bus_voltage, self._handle_motor_power, self._handle_rep_count),) = self._ble.gatts_register_services((_MOTOR_CONTROL_SERVICE,))
        self._connections = set()
        self._payload = advertising_payload(name=name, services=[_MOTOR_CONTROL_UUID])
        self._advertise()
        self._motor_control = MotorControl()  # Initialize the motor control
        self._vacuum_control = VacuumControl(15) # Initialize the vacuum pump control
        self._ir_control = IRControl(I2C(0, scl=Pin(17), sda=Pin(16), freq=100000)) # Initialize the IR sensor
        self._pressure_control = BME680Driver(machine.I2C(0, scl=machine.Pin(17), sda=machine.Pin(16))) # Initialize the barometer ex _barometer_control.gas()
        self.motor_torque = 0  # Initialize motor torque variable as an unsigned integer
        self.suction_status = 0  # Initialize suction status variable as an unsigned integer
        self.bus_voltage = 0.0  # Initialize bus voltage variable
        self.motor_power = 0.0  # Initialize motor power variable

    def _irq(self, event, data):
        # Handle different BLE events
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
            if attr_handle == self._handle_motor_torque:
                self._handle_motor_torque_write()
            elif attr_handle == self._handle_suction_status:
                self._handle_suction_status_write()

    def _advertise(self, interval_us=500000):
        # Start BLE advertising
        print("Starting advertising")
        self._ble.gap_advertise(interval_us, adv_data=self._payload)

    def _handle_motor_torque_write(self):
        # Handle write to motor torque characteristic
        value = self._ble.gatts_read(self._handle_motor_torque)
        if len(value) == 1:  # Handle 1-byte input
            self.motor_torque = struct.unpack('B', value)[0]
            self._motor_control.set_torque(self.motor_torque / 10)  # Divide by 10 for control
            print(f"Set motor torque to: {self.motor_torque / 10}")
        elif len(value) == 4:  # Handle 4-byte input
            self.motor_torque = struct.unpack('I', value)[0]
            self._motor_control.set_torque(self.motor_torque / 10)  # Divide by 10 for control
            print(f"Set motor torque to: {self.motor_torque / 10}")
        else:
            print("Invalid data length for motor torque write")

    def _read_motor_torque(self):
        # Read motor torque from the connected app
        value = self._ble.gatts_read(self._handle_motor_torque)
        print(f"Raw motor torque value: {value}")
        if len(value) == 1:  # Handle 1-byte input
            self.motor_torque = struct.unpack('B', value)[0]
            self._motor_control.set_torque(self.motor_torque / 10)  # Divide by 10 for control
            print(f"Read motor torque from app: {self.motor_torque / 10}")
        elif len(value) == 4:  # Handle 4-byte input
            self.motor_torque = struct.unpack('I', value)[0]
            self._motor_control.set_torque(self.motor_torque / 10)  # Divide by 10 for control
            print(f"Read motor torque from app: {self.motor_torque / 10}")
        else:
            print("Invalid data length for motor torque read")

    def _read_suction_status(self):
        # Read suction status from the connected app
        value = self._ble.gatts_read(self._handle_suction_status)
        if len(value) == 1:  # Handle 1-byte input
            self.suction_status = struct.unpack('B', value)[0]
            print(f"Read suction status from app: {self.suction_status}")
        elif len(value) == 4:  # Handle 4-byte input
            self.suction_status = struct.unpack('I', value)[0]
            print(f"Read suction status from app: {self.suction_status}")
        else:
            print("Invalid data length for suction status read")

    def _read_bus_voltage(self):
        # Read bus voltage using motor control library
        self.bus_voltage = self._motor_control.read_bus_voltage()
        if self.bus_voltage is not None:
            value = struct.pack('f', self.bus_voltage)
            self._ble.gatts_write(self._handle_bus_voltage, value)
            for conn_handle in self._connections:
                self._ble.gatts_notify(conn_handle, self._handle_bus_voltage, value)
            print(f"Bus Voltage: {self.bus_voltage}")
        else:
            print("Failed to read bus voltage")

    def _handle_suction_status_write(self):
        # Handle write to suction status characteristic
        value = self._ble.gatts_read(self._handle_suction_status)
        if len(value) == 1:  # Handle 1-byte input
            self.suction_status = struct.unpack('B', value)[0]
            self.set_suction_status(self.suction_status)
            print(f"Set suction status to: {self.suction_status}")
        elif len(value) == 4:  # Handle 4-byte input
            self.suction_status = struct.unpack('I', value)[0]
            self.set_suction_status(self.suction_status)
            print(f"Set suction status to: {self.suction_status}")
        else:
            print("Invalid data length for suction status write")

    def set_suction_status(self, status):
        # Ensure the status is within the valid range (0-255 for a single byte)
        if not (0 <= status <= 255):
            print("Invalid suction status value")
            return

        # Update the suction status attribute
        self.suction_status = status

        # Pack the status as a single byte
        value = struct.pack('B', self.suction_status)

        # Write the new value to the characteristic
        self._ble.gatts_write(self._handle_suction_status, value)

        # Notify connected clients about the change
        for conn_handle in self._connections:
            self._ble.gatts_notify(conn_handle, self._handle_suction_status, value)

        print(f"Suction status updated to: {self.suction_status}")

    def _read_motor_power(self):
        # Read motor power using motor control library
        self.motor_power = self._motor_control.read_motor_power()
        if self.motor_power is not None:
            value = struct.pack('f', self.motor_power)
            self._ble.gatts_write(self._handle_motor_power, value)
            for conn_handle in self._connections:
                self._ble.gatts_notify(conn_handle, self._handle_motor_power, value)
            print(f"Motor Power: {self.motor_power}")
        else:
            print("Failed to read motor power")

    def _read_rep_count(self, rep_count):
        # Read rep count from the sensor or control library
        self.rep_count = rep_count  # Rep count from update method call
        if self.rep_count is not None:
            value = struct.pack('B', self.rep_count)
            self._ble.gatts_write(self._handle_rep_count, value)
            for conn_handle in self._connections:
                self._ble.gatts_notify(conn_handle, self._handle_rep_count, value)
            print(f"Rep Count: {self.rep_count}")
        else:
            print("Failed to read rep count")

    def update(self, rep_count):
        # Send bus voltage, motor power, and rep count data
        if self._connections:
            self._read_bus_voltage()
            self._read_motor_power()
            self._read_rep_count(rep_count)
