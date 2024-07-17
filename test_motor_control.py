from motor_control import MotorControl
import time

def main():
    motor = MotorControl()

    # Example usage
    motor.run_motor(0.5)  # Set the torque limits to ±0.5

    t = 10
    while t > 0:
        motor.feed_watchdog()
        t -= 1
        print(motor.read_bus_voltage())
        print(motor.read_motor_power())
        motor.set_torque(t/3)
        print(t)
        time.sleep(0.6)

    motor.stop_motor()

if __name__ == "__main__":
    main()
