from machine import Pin, UART
import time

class MotorControl:
    repCount = 0
    def __init__(self, tx_pin=4, rx_pin=5, baudrate=115200):
        self.uart = UART(1, baudrate=baudrate, tx=Pin(tx_pin), rx=Pin(rx_pin)) # Init UART

    def clear_motor_faults(self):
        self.uart.write("sc *48\n") # Clear faults

    def feed_watchdog(self):
        self.uart.write("u 0 *69\n") # Feed watchdog without setting anything

    def read_motor_position(self):
        self.uart.write("r axis0.pos_estimate *74\n") # Request pos
        time.sleep(0.01)
        if self.uart.any():
            response = self.uart.read()
            print(response)
            position_str = response.decode().strip().split('*')[0] # Convert to correct format
            return float(position_str)
        return -1 # Error

    def read_motor_torque_max(self):
        self.uart.write("r axis0.config.torque_soft_max *57\n")
        time.sleep(0.02)
        if self.uart.any():
            response = self.uart.read()
            try:
                torque_str = response.decode().strip().split('*')[0]
                return float(torque_str)
            except ValueError:
                print("motor torque val error" + str(response))
                return -1
        return -1

    def read_motor_torque_min(self):
        self.uart.write("r axis0.config.torque_soft_min *39\n")
        time.sleep(0.02)
        if self.uart.any():
            response = self.uart.read()
            try:
                torque_str = response.decode().strip().split('*')[0]
                return float(torque_str)
            except ValueError:
                print("motor torque val error" + str(response))
                return -1
        return -1

    def set_torque(self, torque):
        torque += 0.2
        try:
            self.uart.write(f"w axis0.config.torque_soft_max {torque}\n")
            self.uart.write(f"w axis0.config.torque_soft_min {-torque}\n")
            print(f"New torque setpoint: {torque}")
        except ValueError:
            print("issue setting torque")
            return -1

    def read_motor_power(self):
        self.uart.write("r axis0.motor.electrical_power *4\n")
        time.sleep(0.02)
        if self.uart.any():
            response = self.uart.read()
            try:
                power_str = response.decode().strip().split('*')[0]
                print("raw power " + str(power_str))
                return float(power_str)
            except ValueError:
                print("motor power val error" + str(response))
                return -1
        return -1

    def read_bus_voltage(self):
        self.uart.write("r vbus_voltage *93\n")
        time.sleep(0.01)
        if self.uart.any():
            response = self.uart.read()
            try:
                voltage_str = response.decode().strip().split('*')[0]
                return float(voltage_str)
            except ValueError:
                print("Error: Invalid voltage value received.")
                return 0
        return -1


    def read_motor_faults(self):
        self.uart.write("r get_drv_fault\n")
        time.sleep(0.01)
        if self.uart.any():
            response = self.uart.read()
            position_str = response.decode().strip().split('\r\n')[0]
            return float(position_str)
        return None

    def home_motor(self):
        margin = 0.2
        self.set_torque(0.3) # Very soft
        self.clear_motor_faults()
        self.uart.write("w axis0.requested_state 8 *42\n") # Closed loop control
        running = False
        while True:
            position = self.read_motor_position() # Read axis0 position
            print("HOMEING: " + str(position))
            if position is not None:
                if position > margin or position < -margin: # Home
                    if not running:
                        self.uart.write("t 0 0 *84\n")
                        # running = True
                else:
                    self.uart.write("w axis0.requested_state 1 *35\n") # Return to idle
                    break
            else:
                return False # Error reading data
            time.sleep(0.05)
        return True # Successful home

    def run_motor(self, torque):
        self.repCount = 0
        if self.home_motor():
            self.set_torque(torque)
            self.uart.write("w axis0.requested_state 8 *42\n")
            self.uart.write("q 0 0 *81 \n")
            print("Motor is now holding torque")

    def stop_motor(self):
        self.uart.write("w axis0.requested_state 1 *35\n")
        print("Motor has been stopped.")

    def read_motor_vel(self):
        self.uart.write("r axis0.vel_estimate *89\n")
        time.sleep(0.02)
        if self.uart.any():
            response = self.uart.read()
            try:
                print("Read Velocity: " + str(response))
                # Decode the response and remove whitespace characters including \r\n
                vel_str = response.decode().strip()
                # Ensure that only the first part before any newline is considered
                vel_str = vel_str.split('*')[0]
                # Convert to float

                return float(vel_str)
            except (ValueError, IndexError) as e:
                print(f"Error: Invalid velocity received. {e}")
                return -1
        time.sleep(0.01)
        return 0

