import sys
sys.path.append('../../arlobot_bringup/scripts/hal/hw')
import time
from psochw import PsocHw, PsocHwError
from i2c import I2CBus, I2CBusError

def WriteLinearBias(value):
    # Write the value to the serial port
    pass

def WriteAngularBias(value):
    # Write the value to the serial port
    pass

if __name__ == "__main__":
    # This application implements the action and interactions between Psoc and the Raspberry Pi for the purposes of
    # calibrating the following:
    #
    #   1. Motor Speed - mapping count/sec to PWM values
    #   2. PID Gains - determine gains which minimize velocity error (using TWIDDLE)
    #   3. Linear Bias - moves forward 1 meter at 0.15 meter per sec.  User measures actual distance and inputs bias
    #   4. Angular Bias - rotates 360 degress at 0.7 radians per sec.  User measure actual rotation and inputs bias

    # Program Options
    #
    #  python arlobot_calibrate.py [action]
    #       where [action] is 'motor', 'pid', 'linear', 'angular'

    try:
        i2c_bus = I2CBus(I2CBus.DEV_I2C_1)
    except RuntimeError:
        raise I2CBusError()

    try:
        psoc = PsocHw(i2c_bus, PsocHw.PSOC_ADDR)
    except PsocHwError:
        print("Failed to create PsocHw instance")

    port = sys.argv[0]
    baud = sys.argv[1]
    action = sys.argv[2]

    if action == 'motor':
        # Initiate motor calibration by setting the motor calibration bit.  No other interaction is required.
        psochw.EnableMotorCalibration()
        while not psochw.MotorsCalibrated():
            time.sleep(1)

    elif action == 'pid':
        # Initiate pid calibration by setting the pid calibration bit.  No other interaction is required.
        psochw.EnablePidGainCalibration()
        while not psochw.PidsCalibrated():
            time.sleep(1)

    elif action == 'linear':
        # Initiate linear bias calibration by setting the linear calibration bit.  Once the motion completes, arlobot will
        # wait 60 seconds for a transmission over the serial port.  The transmission content is one floating point value
        # representing the actual distance traveled (in meter per second).

        psochw.EnableLinearCalibration()
        value = raw_input("Enter the actual linear travel (in meters): ")
        WriteLinearBias(float(value))

    elif action == 'angular':
        # Initiate angular bias calibration by setting the angular calibration bit.  Once the motion completes, arlobot
        # will wait 60 seconds for a transmission over the serial port.  The transmission content is on floating point
        # value representing the actual rotation (in degrees).

        psochw.EnableAngularCalibration()
        value = raw_input("Enter the actual angular rotation (in degrees): ")
        WriteAngularBias(float(value))

    pass