from flex_handler import FlexHandler
import time

flex = FlexHandler()
delta = 0.5 # test
isMotorRunning = False

while True:
        flex.update()
        # Check for reads from app and update local variables

        flex._read_motor_torque()
        flex._read_suction_status()
        flex._motor_control.feed_watchdog()
        flex._motor_control.set_torque(flex.motor_torque/10)
        # You can now use the following instance variables:
        # flex.motor_torque, flex.suction_status, flex.bus_voltage, flex.motor_power

        if flex.suction_status == -1: # fault
            flex._motor_control.stop_motor()
            isMotorRunning = False
            flex._vacuum_control.start_pump()
            print("FAULT")

        if flex.suction_status == 0: # off surface
            print("STATUS SET 0")
            isMotorRunning = False
            flex._vacuum_control.stop_pump()
            if flex._ir_control.onSurface():
                flex.set_suction_status(1)

        if flex.suction_status == 1: # resting
            print("STATUS SET 1")
            isMotorRunning = False
            flex._vacuum_control.stop_pump()
            if not flex._ir_control.onSurface():
                flex.set_suction_status(0)

        if flex.suction_status == 2: # transition
            print("STATUS SET 2")
            isMotorRunning = False
            if flex._ir_control.onSurface():
                flex._vacuum_control.start_pump()
                while flex._pressure_control.pressure_psi() >= 5:
                    if delta == 0:
                        flex.set_suction_status(-1)
                        break
                    time.sleep(0.1)
                flex._vacuum_control.stop_pump()
                time.sleep(3)
                while flex._pressure_control.pressure_psi() >= 5:
                    if delta == 0:
                        flex.set_suction_status(-1)
                        break
                if flex.suction_status != -1:
                    flex.set_suction_status(3)

        if flex.suction_status == 3: # on surface
            print("STATUS SET 3")
            if not isMotorRunning:
                flex._motor_control.run_motor(flex.motor_torque/10)
                isMotorRunning = True
            if flex._pressure_control.pressure_psi() >= 5:
                flex._vacuum_control.start_pump()
            else:
                flex._vacuum_control.stop_pump()
            if delta == 0:
                flex.set_suction_status(-1)
                break
            flex._motor_control.feed_watchdog()

        if flex.suction_status == 4: # release
            print("STATUS SET 4")
            flex._motor_control.stop_motor()
            isMotorRunning = False
            if flex._pressure_control.pressure_psi() >= 5:
                flex.set_suction_status(0)

        if flex.suction_status == 5: # clear faults
            print("STATUS SET 5")
            flex._motor_control.stop_motor()
            isMotorRunning = False
            flex._motor_control.clear_motor_faults()
            flex.set_suction_status(0)

        time.sleep(0.2)
