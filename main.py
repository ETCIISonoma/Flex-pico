from flex_handler import FlexHandler
import time

flex = FlexHandler()
delta = 0.5 # test
isMotorRunning = False
outStroke = False
repCount = 0

while True:
        flex.update(repCount)
        # Check for reads from app and update local variables
        flex._motor_control.feed_watchdog()
        flex._read_motor_torque()
        flex._read_suction_status()
        flex._motor_control.set_torque(flex.motor_torque/10)
        # You can now use the following instance variables:
        # flex.motor_torque, flex.suction_status, flex.bus_voltage, flex.motor_power

        if flex.suction_status == 6: # fault
            flex._motor_control.run_motor(0.5)
            time.sleep(5)
            flex._motor_control.stop_motor()
            isMotorRunning = False
            flex._vacuum_control.start_pump()
            print("FAULT")

        if flex.suction_status == 0: # off surface
            print("STATUS SET 0")
            isMotorRunning = False
            flex._motor_control.stop_motor()
            flex._vacuum_control.stop_pump()
            if flex._ir_control.onSurface():
                flex.set_suction_status(1)

        if flex.suction_status == 1: # resting
            print("STATUS SET 1")
            isMotorRunning = False
            flex._motor_control.stop_motor()
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
                        flex.set_suction_status(6)
                        break
                    time.sleep(0.1)
                flex._vacuum_control.stop_pump()
                time.sleep(0.5)
                while flex._pressure_control.pressure_psi() >= 5:
                    if delta == 0:
                        flex.set_suction_status(6)
                        break
                if flex.suction_status != 6:
                    flex.set_suction_status(3)

        if flex.suction_status == 3: # on surface
            print("STATUS SET 3, outstroke:" + str(outStroke))
            vel = flex._motor_control.read_motor_vel()
            print("Pwr"+ str(vel))
            if not isMotorRunning:
                flex._motor_control.run_motor(flex.motor_torque/10)
                isMotorRunning = True
                repCount = 0
            pressure = flex._pressure_control.pressure_psi()
            if pressure >= 7:
                flex._vacuum_control.start_pump()
            if pressure <= 5:
                flex._vacuum_control.stop_pump()
            if pressure >= 10:
                flex.set_suction_status(6)
            print("zao shang hao zhongguo")
            print(vel)
            print(outStroke)
            if vel > 0.5 and not outStroke:
                outStroke = True
                print("OUTSTROKE BEGIN")
                repCount += 1
            elif vel < -0.5 and outStroke and flex._motor_control.read_motor_position() < 1:
                outStroke = False
                print("RESET OUTSTROKE")

        if flex.suction_status == 4: # release
            print("STATUS SET 4")
            flex._motor_control.home_motor()
            isMotorRunning = False
            if not flex._ir_control.onSurface():
                flex.set_suction_status(0)

        if flex.suction_status == 5: # clear faults
            print("STATUS SET 5")
            flex._motor_control.stop_motor()
            isMotorRunning = False
            flex._motor_control.clear_motor_faults()
            flex.set_suction_status(0)
