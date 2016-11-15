from __future__ import print_function

import time
import sys
from Adafruit_BNO055 import BNO055

# This is calibration data obtained by following running a calibation routine composed of the following:
#   1. Gyroscope Calibration - leave the unit on a level surface until the gryo status reports 3
#   2. Acclerometer Calibration - rotate the unit in increments of 45 degrees holding each position for 2-3 seconds
#      until the accel status reports 3
#   3. Magnetometer Calibration - move the unit is a figure 8 motion until the mag status reports 3
#   4. System Calibration - leave the unit motionless until the system status reports 3
# This data is loaded by the BNO055Hw module when it is initialized.
cal_data = [2, 0, 226, 255, 9, 0, 253, 255, 49, 0, 23, 255, 0, 0, 254, 255, 0, 0, 232, 3, 237, 3]


class BNO055HwError(Exception):
    pass


class BNO055Hw:
    __RETRIES = 3

    def __init__(self, do_cal_check=True, retries=__RETRIES):
        """
        Instantiate and initialize the BNO055, load calibration data and confirm the device is calibrated
        :param do_cal_check:
        :except BNO055HwError - if initialization fails
                RuntimeError - if initialization fails, raises BNO055HwError
                ValueError - if the calibration is not in the correct format, raises BNO055HwError
                BNO055HwError - if calibration confirmation fails
        """
        try:
            self._bno = BNO055.BNO055(rst='P9_12')
        except RuntimeError as e:
            raise BNO055HwError(e.args)

        # Initialize the BNO055 and stop if something went wrong.
        # Note: Sometimes after a reset a runtime error can occur but usually only once.  The following retry seems to
        # get around this error.
        try:
            for i in range(retries):
                result = self._bno.begin()
                if result:
                    break
                time.sleep(0.05)
            if not result:
                raise BNO055HwError('Failed to initialize BNO055! Is the sensor connected?')
        except RuntimeError as e:
            raise BNO055HwError(e.args)

        # Load the calibration data into the BNO055
        try:
            self._bno.set_calibration(cal_data)
        except ValueError:
            raise BNO055HwError("Calibration data is not in correct format: {}".format(cal_data))
        
        # Allow the calibration check to be bypassed so that calibration can performed
        if do_cal_check:
            # Confirm that the gyro, accel, mag and system are reported as calibrated
            # Note: A timeout of 10 seconds limits this checking
            self._cal_confirmed = self._confirm_calibration()
            if not self._cal_confirmed:
                raise BNO055HwError("Failed to confirm device calibration")

    def _wait_for_valid_status(self, key, timeout=10):
        """
        Checks that the specified calibration status is valid (equal to 3) for 5 seconds and limits the checking to
        the specified timeout (default 10 seconds)
        :param key:
        :param timeout:
        :return: True if the status is valid; otherwise, False
        """
        status = 0
        count = 0
        start_time = time.time()
        delta_time = start_time
        # Stay in the loop until there is a valid status for 5 seconds or the timeout expires
        while status != 3 or count < 50 or delta_time > timeout:
            status = self.get_calibration_status()[key]
            time.sleep(0.1)
            if status != 3:
                count = 0
            else:
                count += 1
            delta_time = time.time() - start_time

        if status == 3 and count <= 50 and delta_time < timeout:
            return True
        else:
            return False

    def _confirm_calibration(self):
        """
        Confirms that the gyroscope, magnetometer, accelerometer and system calibration are valid
        :return: True if calibration is confirmed; otherwise, False
        """
        cal_status = self.get_calibration_status()
        if cal_status['gyro'] == 3 and cal_status['mag'] == 3 and cal_status['accel'] == 3:
            return self._wait_for_valid_status('sys')
        return False

    def calibration_confirmed(self):
        return self._cal_confirmed

    def show_revision(self, redirect=sys.stdout):
        """
        Return a tuple with revision information about the BNO055 chip.  Will
        return 5 values:
          - Software revision
          - Bootloader version
          - Accelerometer ID
          - Magnetometer ID
          - Gyro ID
        """
        sw, bl, accel, mag, gyro = self._bno.get_revision()
        print('Software version:   {0}'.format(sw), file=redirect)
        print('Bootloader version: {0}'.format(bl), file=redirect)
        print('Accelerometer ID:   0x{0:02X}'.format(accel), file=redirect)
        print('Magnetometer ID:    0x{0:02X}'.format(mag), file=redirect)
        print('Gyroscope ID:       0x{0:02X}\n'.format(gyro), file=redirect)


    def show_system_status(self, run_self_test=True, redirect=sys.stdout):
        """
        Return a tuple with status information.  Three values will be returned:
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
        print('System status: {0}'.format(status), file=redirect)
        print('Self test result (0x0F is normal): 0x{0:02X}'.format(0 if not self_test else self_test), file=redirect)
        # Print out an error if system status is in error mode.
        if status == 0x01:
            print('System error: {0}'.format(error), file=redirect)
            print('See datasheet section 4.3.59 for the meaning.', file=redirect)

    def get_calibration_status(self):
        """
        Read the calibration status of the sensors and return a 4 tuple with
        calibration status as follows:
          - System, 3=fully calibrated, 0=not calibrated
          - Gyroscope, 3=fully calibrated, 0=not calibrated
          - Accelerometer, 3=fully calibrated, 0=not calibrated
          - Magnetometer, 3=fully calibrated, 0=not calibrated
        """
        return dict(zip(['sys', 'gyro', 'accel', 'mag'], self._bno.get_calibration_status()))

    def show_calibration_status(self, redirect=sys.stdout):
        """
        Displays the current calibration status
        Note: Output can be redirected

        :param redirect: allows the print output to be redirected as needed
        :return: None
        """
        cal_sys, cal_gyro, cal_accel, cal_mag = self._bno.get_calibration_status()
        print('Sys_cal={} Gyro_cal={} Accel_cal={} Mag_cal={}'.format(cal_sys, cal_gyro, cal_accel, cal_mag), file=redirect)

    def show_calibration(self, redirect=sys.stdout):
        """
        Displays the current calibration data
        Note: Output can be redirected
        :param redirect: allows the print output to be redirected as needed
        :return: None
        """
        print(self._bno.get_calibration(), file=redirect)

    def read_orientation(self):
        """
        Return the current orientation as a tuple of X, Y, Z, W quaternion
        values.
        """
        w, x, y, z = self._bno.read_quaternion()
        return dict(zip(['x', 'y', 'z', 'w'], [x, y, z, w]))

    def read_acceleration(self):
        """
        Return the current linear acceleration (acceleration from movement,
        not from gravity) reading as a tuple of X, Y, Z values in meters/second^2.
        """
        x, y, z = self._bno.read_linear_acceleration()
        return dict(zip(['x', 'y', 'z'], [x, y, z]))

    def read_gyroscope(self):
        """
        Return the current gyroscope (angular velocity) reading as a tuple of
        X, Y, Z values in degrees per second.
        """
        x, y, z = self._bno.read_gyroscope()
        return dict(zip(['x', 'y', 'z'], [x, y, z]))

    def read_magnetometer(self):
        """
        Return the current magnetometer reading as a tuple of X, Y, Z values
        in micro-Teslas.
        """
        x, y, z = self._bno.read_magnetometer()
        return dict(zip(['x', 'y', 'z'], [x, y, z]))

    def read_euler(self):
        """
        Return the current absolute orientation as a tuple of heading, roll,
        and pitch euler angles in degrees.
        """
        yaw, roll, pitch = self._bno.read_euler()
        return dict(zip(['heading', 'yaw', 'roll', 'pitch'], [yaw, yaw, roll, pitch]))

    def read_temperature(self):
        """
        Return the current temperature in Celsius and Fahrenheit.
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
        print("heading/yaw: {heading:.3f}, pitch: {pitch:.3f}, roll: {roll:.3f}".format(heading=euler['heading'],
                                                                                        pitch=euler['pitch'],
                                                                                        roll=euler['roll']))
        #print(values)

    def test_read_temperature():
        values = bnohw.read_temperature()
        print(values)

    #-------------------------------------------------------------------------------------------------------------------
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

    def wait_for_gyro_calibration():
        print("Gyro: let the BNO055 sit on a flat surface until the gyro calibration value equals 3")
        bnohw._wait_for_valid_status('gyro')

    def wait_for_accel_calibration(bnohw):
        print("Accel: rotate the BNO055 by 45 degrees pausing 2-3 second between each move")
        bnohw._wait_for_valid_status('accel')

    def wait_for_mag_calibration(bnohw):
        print("Mag: move the BNO055 in a figure 8 until the mag calibration value equals 3")
        bnohw._wait_for_valid_status('mag')

    def wait_for_system_calibration(bnohw):
        print("System: once gyro, mag, and accel are calibrated, system will become calibrated")
        bnohw._wait_for_valid_status('sys')

    def check_system_calibration(bnohw):
        if bnohw._confirm_calibration():
            print("device is calibrated")
            return True
        return False

    #-------------------------------------------------------------------------------------------------------------------
    # Check to see if the system is calibrated; if it is then return True
    #-------------------------------------------------------------------------------------------------------------------
    result = check_system_calibration(bnohw)
    if result:
        print("device is calibrated")
        return result

    #-------------------------------------------------------------------------------------------------------------------
    # Perform Gyroscope calibration
    #-------------------------------------------------------------------------------------------------------------------
    wait_for_gyro_calibration()
    #-------------------------------------------------------------------------------------------------------------------
    # Perform Accelerometer calibration
    #-------------------------------------------------------------------------------------------------------------------
    wait_for_accel_calibration()
    #-------------------------------------------------------------------------------------------------------------------
    # Perform Magnetometer calibration
    #-------------------------------------------------------------------------------------------------------------------
    wait_for_mag_calibration()
    #-------------------------------------------------------------------------------------------------------------------
    # Wait for system calibration to be reported
    #-------------------------------------------------------------------------------------------------------------------
    result = check_system_calibration()

    #-------------------------------------------------------------------------------------------------------------------
    # Display calibration status
    #-------------------------------------------------------------------------------------------------------------------
    bnohw.show_calibration_status()
    if result:
        print("device is calibrated")
        bnohw.show_calibration()
    else:
        print("device is not calibrated")

    return result


if __name__ == "__main__":
    do_cal_check = True
    if len(sys.argv) > 1:
        if sys.argv[1] == 'calibrate':
            do_cal_check = False

    try:
        bnohw = BNO055Hw(do_cal_check)
    except BNO055HwError as e:
        print(e.args)

    if not do_cal_check:
        print("The device is not calibrated.  Please write a little application that can be used to calibrate this device")
        perform_calibration(bnohw)
        sys.exit(0)


    for i in range(20):
        euler = bnohw.read_euler()
        print("heading/yaw: {heading:.3f}, pitch: {pitch:.3f}, roll: {roll:.3f}".format(heading=euler['heading'], pitch=euler['pitch'], roll=euler['roll']))
    print("\n\n") 

    test(bnohw)
