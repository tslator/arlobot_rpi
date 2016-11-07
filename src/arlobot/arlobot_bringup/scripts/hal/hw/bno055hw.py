from __future__ import print_function

import time
from Adafruit_BNO055 import BNO055

# This is calibration data obtained by following the calibation routine below as part of the self test
# This data is loaded by the BNO055Hw module when it is initialized.
cal_data = [2, 0, 226, 255, 9, 0, 253, 255, 49, 0, 23, 255, 0, 0, 254, 255, 0, 0, 232, 3, 237, 3]

class BNO055HwError(Exception):
    pass


class BNO055Hw:
    def __init__(self):
        self._bno = BNO055.BNO055(rst='P9_12')

        # Initialize the BNO055 and stop if something went wrong.
        if not self._bno.begin():
            raise BNO055HwError('Failed to initialize BNO055! Is the sensor connected?')

        try:
            self._bno.set_calibration(cal_data)
        except ValueError:
            raise BNO055HwError("Calibration data is not in correct format: {}".format(cal_data))

    def show_revision(self):
        """Return a tuple with revision information about the BNO055 chip.  Will
        return 5 values:
          - Software revision
          - Bootloader version
          - Accelerometer ID
          - Magnetometer ID
          - Gyro ID
        """
        sw, bl, accel, mag, gyro = self._bno.get_revision()
        print('Software version:   {0}'.format(sw))
        print('Bootloader version: {0}'.format(bl))
        print('Accelerometer ID:   0x{0:02X}'.format(accel))
        print('Magnetometer ID:    0x{0:02X}'.format(mag))
        print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))


    def show_system_status(self, run_self_test=True):
        """Return a tuple with status information.  Three values will be returned:
          - System status register value with the following meaning:
              0 = Idle
              1 = System Error
              2 = Initializing Peripherals
              3 = System Initialization
              4 = Executing Self-Test
              5 = Sensor fusion algorithm running
              6 = System running without fusion algorithms
          - Self test result register value with the following meaning:
              Bit value: 1 = test passed, 0 = test failed
              Bit 0 = Accelerometer self test
              Bit 1 = Magnetometer self test
              Bit 2 = Gyroscope self test
              Bit 3 = MCU self test
              Value of 0x0F = all good!
          - System error register value with the following meaning:
              0 = No error
              1 = Peripheral initialization error
              2 = System initialization error
              3 = Self test result failed
              4 = Register map value out of range
              5 = Register map address out of range
              6 = Register map write error
              7 = BNO low power mode not available for selected operation mode
              8 = Accelerometer power mode not available
              9 = Fusion algorithm configuration error
             10 = Sensor configuration error

        If run_self_test is passed in as False then no self test is performed and
        None will be returned for the self test result.  Note that running a
        self test requires going into config mode which will stop the fusion
        engine from running.
        """
        # Return the results as a tuple of all 3 values.
        status, self_test, error = self._bno.get_system_status(run_self_test)
        print('System status: {0}'.format(status))
        print('Self test result (0x0F is normal): 0x{0:02X}'.format(0 if not self_test else self_test))
        # Print out an error if system status is in error mode.
        if status == 0x01:
            print('System error: {0}'.format(error))
            print('See datasheet section 4.3.59 for the meaning.')

    def get_calibration_status(self):
        return dict(zip(['sys', 'gyro', 'accel', 'mag'], self._bno.get_calibration_status()))

    def show_calibration_status(self):
        """Read the calibration status of the sensors and return a 4 tuple with
        calibration status as follows:
          - System, 3=fully calibrated, 0=not calibrated
          - Gyroscope, 3=fully calibrated, 0=not calibrated
          - Accelerometer, 3=fully calibrated, 0=not calibrated
          - Magnetometer, 3=fully calibrated, 0=not calibrated
        """
        cal_sys, cal_gyro, cal_accel, cal_mag = self._bno.get_calibration_status()
        print('Sys_cal={} Gyro_cal={} Accel_cal={} Mag_cal={}'.format(cal_sys, cal_gyro, cal_accel, cal_mag))

    def show_calibration(self):
        print(self._bno.get_calibration())

    def read_orientation(self):
        """Return the current orientation as a tuple of X, Y, Z, W quaternion
        values.
        """
        w, x, y, z = self._bno.read_quaternion()
        return dict(zip(['x', 'y', 'z', 'w'], [x, y, z, w]))

    def read_acceleration(self):
        """Return the current linear acceleration (acceleration from movement,
        not from gravity) reading as a tuple of X, Y, Z values in meters/second^2.
        """
        x, y, z = self._bno.read_linear_acceleration()
        return dict(zip(['x', 'y', 'z'], [x, y, z]))

    def read_gyroscope(self):
        """Return the current gyroscope (angular velocity) reading as a tuple of
        X, Y, Z values in degrees per second.
        """
        x, y, z = self._bno.read_gyroscope()
        return dict(zip(['x', 'y', 'z'], [x, y, z]))

    def read_magnetometer(self):
        """Return the current magnetometer reading as a tuple of X, Y, Z values
        in micro-Teslas.
        """
        x, y, z = self._bno.read_magnetometer()
        return dict(zip(['x', 'y', 'z'], [x, y, z]))

    def read_euler(self):
        """Return the current absolute orientation as a tuple of heading, roll,
        and pitch euler angles in degrees.
        """
        yaw, roll, pitch = self._bno.read_euler()
        return dict(zip(['heading', 'yaw', 'roll', 'pitch'], [yaw, yaw, roll, pitch]))

    def read_temperature(self):
        """Return the current temperature in Celsius and Fahrenheit.
        """
        temp_celsius = self._bno.read_temp()
        return {'c': float(temp_celsius), 'f': (float(temp_celsius) * 9.0/5.0) + 32.0}

#------------------------------------------------------------------------------------
# Module Test
#------------------------------------------------------------------------------------

def test(bnohw):
    def test_show_revision():
        bnohw.show_revision()

    def test_show_system_status(selftest=False):
        bnohw.show_system_status(selftest)

    def test_show_calibration_status():
        bnohw.show_calibration_status()

    def test_show_calibration():
        bnohw.show_calibration()

    def test_read_orientation():
        values = bnohw.read_orientation()
        print(values)

    def test_read_acceleration():
        values = bnohw.read_acceleration()
        print(values)

    def test_read_gyroscope():
        values = bnohw.read_gyroscope()
        print(values)

    def test_read_magnetometer():
        values = bnohw.read_magnetometer()
        print(values)

    def test_read_euler():
        euler = bnohw.read_euler()
        print("heading/yaw: {heading:.3f}, pitch: {pitch:.3f}, roll: {roll:.3f}".format(heading=euler['heading'], pitch=euler['pitch'], roll=euler['roll']))
        #print(values)

    def test_read_temperature():
        values = bnohw.read_temperature()
        print(values)


    print("Test Case 1: show revision ...")
    test_show_revision()

    print("Test Case 2: show system status (no self test) ...")
    test_show_system_status()

    print("Test Case 3: show calibration status ...")
    #test_show_calibration_status()

    print("Test Case 4: show calibration ...")
    #test_show_calibration()

    print("Test Case 5: read orientation ...")
    test_read_orientation()

    print("Test Case 6: read acceleration ...")
    test_read_acceleration()

    print("Test Case 7: read gyroscope ...")
    test_read_gyroscope()

    print("Test Case 8: read magnetometer ...")
    test_read_magnetometer()

    print("Test Case 9: read euler ...")
    test_read_euler()

    print("Test Case 10: read temperature ...")
    test_read_temperature()

    print("Test Case 11: show system status (self test) ...")
    test_show_system_status(True)

def perform_calibration(bnohw):
    def wait_for_valid_status(key, get_cal_status):
        status = 0
        count = 0
        while status != 3 or count < 50:
            status = get_cal_status()[key]
            print(status, end='\r')
            time.sleep(0.1)
            if status != 3:
                count = 0
            else:
                count += 1
        print("")
        print(key, get_cal_status()[key], count)

    def wait_for_system_calibration(bnohw):
        print("System: once gyro, mag, and accel are calibrated, system will become calibrated")
        wait_for_valid_status('sys', bnohw.get_calibration_status)

    def check_system_calibration(bnohw):
        cal_status = bnohw.get_calibration_status()
        if cal_status['gyro'] == 3 and cal_status['mag'] == 3 and cal_status['accel'] == 3:
            wait_for_system_calibration(bnohw)
            bnohw.show_calibration()
            print("device is calibrated")
            return True

    def wait_for_gyro_calibration(bnohw):
        print("Gyro: let the BNO055 sit on a flat surface until the gyro calibration value equals 3")
        wait_for_valid_status('gyro', bnohw.get_calibration_status)

    def wait_for_accel_calibration(bnohw):
        print("Accel: rotate the BNO055 by 45 degrees pausing 2-3 second between each move")
        wait_for_valid_status('accel', bnohw.get_calibration_status)

    def wait_for_mag_calibration(bnohw):
        print("Mag: move the BNO055 in a figure 8 until the mag calibration value equals 3")
        wait_for_valid_status('mag', bnohw.get_calibration_status)

    result = check_system_calibration(bnohw)
    if result:
        print("device is calibrated")
        return result
    
    wait_for_gyro_calibration(bnohw)
    wait_for_accel_calibration(bnohw)
    wait_for_mag_calibration(bnohw)

    result = check_system_calibration(bnohw)

    bnohw.show_calibration_status()
    if result:
        print("device is calibrated")
    else:
        print("device is not calibrated")
    
    return result

if __name__ == "__main__":
    bnohw = BNO055Hw()
    #bnohw.show_revision()
    #bnohw.show_system_status()
    #bnohw.show_calibration_status()
    
    cal_result = perform_calibration(bnohw)

    if cal_result:
        for i in range(100):
            euler = bnohw.read_euler()
            print("heading/yaw: {heading:.3f}, pitch: {pitch:.3f}, roll: {roll:.3f}".format(heading=euler['heading'], pitch=euler['pitch'], roll=euler['roll']), end='\r')
    print("\n\n") 
    test(bnohw)
