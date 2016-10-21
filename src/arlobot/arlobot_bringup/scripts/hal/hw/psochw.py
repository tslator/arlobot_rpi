"""
This module provides an abstraction for accessing the Freesoc hardware via the I2C bus
It provides accessors for motor, debug, and calibration control and reading status and
distance sensor data.
"""
from __future__ import print_function

import time
import math

class PsocHwError(Exception):
    pass


class PsocHw:

    # Address offsets for the Psoc I2C registers
    __DEVICE_CONTROL_OFFSET = 0
    __DEBUG_CONTROL_OFFSET = 2
    __LEFT_WHEEL_VELOCITY_OFFSET = 4
    __RIGHT_WHEEL_VELOCITY_OFFSET = 8
    __DEVICE_STATUS_OFFSET = 12
    __CALIBRATION_STATUS_OFFSET = 14
    __ODOMETRY_OFFSET = 16
    __FRONT_ULTRASONIC_DISTANCE_OFFSET = 36 
    __REAR_ULTRASONIC_DISTANCE_OFFSET = 68
    __FRONT_INFRARED_DISTANCE_OFFSET = 100
    __REAR_INFRARED_DISTANCE_OFFSET = 132
    __HEARTBEAT_OFFSET = 164


    # Map of the register functionality and offsets
    __REGISTER_MAP = {#---------- READ/WRITE ------------
                      'DEVICE_CONTROL': __DEVICE_CONTROL_OFFSET,
                      #      - Bit 0: enable / disable the HB25 motors
                      #      - Bit 1: clear odometry
                      #      - Bit 2: clear calibration

                      'DEBUG_CONTROL': __DEBUG_CONTROL_OFFSET,
                      #      - Bit 0: Enable/Disable Encoder Debug
                      #      - Bit 1: Enable/Disable PID debug
                      #      - Bit 2: Enable/Disable Motor debug
                      #      - Bit 3: Enable/Disable Odometry debug
                      #       - Bit 4: Enable/Disable Sample debug

                      'LEFT_WHEEL_VELOCITY': __LEFT_WHEEL_VELOCITY_OFFSET,
                      'RIGHT_WHEEL_VELOCITY': __RIGHT_WHEEL_VELOCITY_OFFSET,

                      #----------------------------------
                      #------ READ/WRITE Boundary -------
                      #----------------------------------

                      #----------- READ ONLY ------------
                      'DEVICE_STATUS' : __DEVICE_STATUS_OFFSET,
                      #      - Bit 0: HB25 Motor Controller Initialized
                      'CALIBRATION_STATUS': __CALIBRATION_STATUS_OFFSET,
                      #      - Bit 0: Wheel Velocity, i.e., Count/Sec-to-PWM
                      #      - Bit 1: PID
                      'ODOMETRY': __ODOMETRY_OFFSET,
                      'FRONT_ULTRASONIC_DISTANCE': __FRONT_ULTRASONIC_DISTANCE_OFFSET,
                      'REAR_ULTRASONIC_DISTANCE': __REAR_ULTRASONIC_DISTANCE_OFFSET,
                      'FRONT_INFRARED_DISTANCE': __FRONT_INFRARED_DISTANCE_OFFSET,
                      'REAR_INFRARED_DISTANCE': __REAR_INFRARED_DISTANCE_OFFSET,
                      'HEARTBEAT': __HEARTBEAT_OFFSET}

    __CONTROL_DISABLE_MOTORS_BIT = 0x0001
    __CONTROL_CLEAR_ODOMETRY_BIT = 0x0002
    __CONTROL_CLEAR_CALIBRATION_BIT = 0x0004

    __DEVICE_STATUS_MOTORS_INIT_BIT = 0x0001

    __DBG_ENCODERS_BIT = 0x0001
    __DBG_PIDS_BIT = 0x0002
    __DBG_MOTORS_BIT = 0x0004
    __DBG_ODOM_BIT = 0x0008
    __DBG_SAMPLE_BIT = 0x0010

    __CAL_MOTORS_BIT = 0x0001
    __CAL_PID_GAINS_BIT = 0x0002

    __CAL_MOTORS_CONTROL_BIT = __CAL_MOTORS_BIT
    __CAL_PID_GAINS_CONTROL_BIT = __CAL_PID_GAINS_BIT

    __CAL_MOTORS_STATUS_BIT = __CAL_MOTORS_BIT
    __CAL_PID_GAINS_STATUS_BIT = __CAL_PID_GAINS_BIT

    DISABLE_MOTORS = __CONTROL_DISABLE_MOTORS_BIT
    CLEAR_ODOMETRY = __CONTROL_CLEAR_ODOMETRY_BIT
    CLEAR_CALIBRATION = __CONTROL_CLEAR_CALIBRATION_BIT

    PSOC_ADDR = 0x08

    def __init__(self, i2cbus, address):
        self._address = address
        self._i2c_bus = i2cbus

        self._debug_mask = 0

    # The following methods are divided into two sections: read-write and read-only
    # The read-write methods support calls that access the read-write section of the Psoc I2C interface
    # The read-only methods support calls that access the read-only section of the Psoc I2C interface

    #------------ Read / Write ---------------

    def _set_device_control(self, value):
        '''
        Set or clear the bits in the control value based on the request
        :param value:
        :return:
        '''
        self._i2c_bus.WriteUint16(self._address, PsocHw.__REGISTER_MAP['DEVICE_CONTROL'], value)

    def _set_debug_control(self, bit, set_clear):
        if set_clear:
            self._debug_mask |= bit
        else:
            self._debug_mask &= ~bit
        self._i2c_bus.WriteUint16(self._address, PsocHw.__REGISTER_MAP['DEBUG_CONTROL'], self._debug_mask)

    def DisableMotors(self):
        self._set_device_control(PsocHw.DISABLE_MOTORS)

    def ClearOdometry(self):
        self._set_device_control(PsocHw.__CONTROL_CLEAR_ODOMETRY_BIT)

    def ClearCalibration(self):
        self._set_device_control(PsocHw.__CONTROL_CLEAR_CALIBRATION_BIT)

    def EncoderDebug(self, value=True):
        self._set_debug_control(PsocHw.__DBG_ENCODERS_BIT, value)

    def PidDebug(self, value=True):
        self._set_debug_control(PsocHw.__DBG_PIDS_BIT, value)

    def MotorDebug(self, value=True):
        self._set_debug_control(PsocHw.__DBG_MOTORS_BIT, value)

    def OdometryDebug(self, value=True):
        self._set_debug_control(PsocHw.__DBG_ODOM_BIT, value)

    def SampleDebug(self, value=True):
        self._set_debug_control(PsocHw.__DBG_SAMPLE_BIT, value)

    def SetSpeed(self, left_speed, right_speed):
        '''
        '''
        # Note:  We can save some I2C protocol overhead by sending both values together as a multi-byte transaction
        self._i2c_bus.WriteArray(self._address,
                                 PsocHw.__REGISTER_MAP['LEFT_WHEEL_VELOCITY'],
                                 [left_speed, right_speed],
                                 'f')
        #print("{}: PsocHw SetSpeed({}, {})".format(time.time(), left_speed, right_speed))

    #------------ Read Only ---------------

    def _get_device_status(self):
        '''
        '''
        return self._i2c_bus.ReadUint16(self._address, PsocHw.__REGISTER_MAP['DEVICE_STATUS'])

    def _get_calibration_status(self):
        '''
        '''
        return self._i2c_bus.ReadUint16(self._address, PsocHw.__REGISTER_MAP['CALIBRATION_STATUS'])

    def GetDeviceStatus(self):
        return self._get_device_status()

    def GetCalibrationStatus(self):
        '''
        '''
        return self._get_calibration_status()

    def MotorsInitialized(self):
        '''
        '''
        result = self._get_device_status() & PsocHw.__DEVICE_STATUS_MOTORS_INIT_BIT
        return result != 0

    def MotorsCalibrated(self):
        '''
        '''
        result = self._get_calibration_status() & PsocHw.__CAL_MOTORS_STATUS_BIT
        return result != 0

    def PidsCalibrated(self):
        result = self._get_calibration_status() & PsocHw.__CAL_PID_GAINS_STATUS_BIT
        return result != 0

    def Calibrated(self):
        '''
        The robot is considered calibrated if either the motors or pids are calibrated
        Detailed calibration information is available in the accessors above
        '''
        result = self._get_calibration_status() & (PsocHw.__CAL_MOTORS_STATUS_BIT | PsocHw.__CAL_PID_GAINS_STATUS_BIT)
        return result != 0

    def GetOdometry(self):
        odometry = self._i2c_bus.ReadArray(self._address, PsocHw.__REGISTER_MAP['ODOMETRY'], 5, 'f')

        return odometry

    def GetUltrasonicDistances(self):
        '''
        '''
        front = self._i2c_bus.ReadArray(self._address, PsocHw.__REGISTER_MAP['FRONT_INFRARED_DISTANCE'], 8, 'f')
        rear = self._i2c_bus.ReadArray(self._address, PsocHw.__REGISTER_MAP['REAR_INFRARED_DISTANCE'], 8, 'f')

        return front + rear

    def GetInfraredDistances(self):
        '''
        '''
        front = self._i2c_bus.ReadArray(self._address, PsocHw.__REGISTER_MAP['FRONT_ULTRASONIC_DISTANCE'], 8, 'f')
        rear = self._i2c_bus.ReadArray(self._address, PsocHw.__REGISTER_MAP['REAR_ULTRASONIC_DISTANCE'], 8, 'f')

        return front + rear

    def GetHeartbeat(self):
        '''
        '''
        return self._i2c_bus.ReadUint32(self._address, PsocHw.__REGISTER_MAP['HEARTBEAT'])


#---------------------------------------------------------------------------------------------------------------------
# Module Test
#---------------------------------------------------------------------------------------------------------------------

def test(psoc):
    from time import sleep
    indent = ' '*4
    def test_read_heartbeat():
        for i in range(5):
            print("{}Heartbeat: {}".format(indent, psoc.GetHeartbeat() ))
            sleep(0.5)

    def test_read_infrared_distances():
        print("{}Infrared Distances: ".format(indent))
        for idx, entry in enumerate(psoc.GetInfraredDistances()):
            print("{}{}: {}".format(indent*2, idx, entry))

    def test_read_ultrasonic_distances():
        print("{}Ultrasonic Distances: ".format(indent))
        for idx, entry in enumerate(psoc.GetUltrasonicDistances()):
            print("{}{}: {}".format(indent*2, idx, entry))

    def test_read_odometry():
        print("{}Odometry: ".format(indent))
        for idx, entry in enumerate(psoc.GetOdometry()):
            print("{}{}: {}".format(indent*2, idx, entry))

    def test_read_status():
        print("{}Calibrated: {}".format(indent, psoc.Calibrated()))
        print("{}Pids Calibrated: {}".format(indent, psoc.PidsCalibrated()))
        print("{}Motors Calibrated: {}".format(indent, psoc.MotorsCalibrated()))
        print("{}Motors Initialized: {}".format(indent, psoc.MotorsInitialized()))
        print("{}Get Calibration Status: {:2x}".format(indent, psoc.GetCalibrationStatus()))
        print("{}Get Device Status: {:2x}".format(indent, psoc.GetDeviceStatus()))

    def test_set_debug():    
        '''
        This test turns on/off debug in the Psoc.  To monitor the results, it is necessary to start a second
        ssh session and open the serial port.  Each test case enables, wait, and disables each debug features
        '''
        delay_time = 2.0
        print("{}Enable encoder debug:".format(indent))
        psoc.EncoderDebug()
        sleep(delay_time)
        print("{}Disable encoder debug:".format(indent))
        psoc.EncoderDebug(False)
        
        print("{}Enable pid debug:".format(indent))
        psoc.PidDebug()
        sleep(delay_time)
        print("{}Disable pid debug:".format(indent))
        psoc.PidDebug(False)
        
        print("{}Enable motor debug:".format(indent))
        psoc.MotorDebug()
        sleep(delay_time)
        print("{}Disable motor debug:".format(indent))
        psoc.MotorDebug(False)
        
        print("{}Enable odometry debug:".format(indent))
        psoc.OdometryDebug()
        sleep(delay_time)
        print("{}Disable odometry debug:".format(indent))
        psoc.OdometryDebug(False)
        
        print("{}Enable sample debug:".format(indent))
        psoc.SampleDebug()
        sleep(delay_time)
        print("{}Disable sample debug:".format(indent))
        psoc.SampleDebug(False)
        
    def test_set_speed():
        psoc.MotorDebug()
        print("{}Set left motor speed {:.3f} ...".format(indent, 0.100))
        psoc.SetSpeed(0.100, 0.0)
        sleep(2.0)

        print("{}Set right motor speed {:.3f} ...".format(indent, 0.100))
        psoc.SetSpeed(0.0, 0.100)
        sleep(2.0)

        print("{}Set left motor speed {:.3f} ...".format(indent, -0.100))
        psoc.SetSpeed(-0.100, 0.0)
        sleep(2.0)

        print("{}Set right motor speed {:.3f} ...".format(indent, -0.100))
        psoc.SetSpeed(0.0, -0.100)
        sleep(2.0)

        print("{}Set left/right motor speed {:.3f},{:.3f} ...".format(indent, 0.100, 0.100))
        psoc.SetSpeed(0.100, 0.100)
        sleep(2.0)
        
        print("{}Set left/right motor speed {:.3f},{:.3f} ...".format(indent, 0.100, -0.100))
        psoc.SetSpeed(0.100, -0.100)
        sleep(2.0)

        print("{}Set left/right motor speed {:.3f},{:.3f} ...".format(indent, -0.100, 0.100))
        psoc.SetSpeed(-0.100, 0.100)
        sleep(2.0)

        print("{}Set left/right motor speed {:.3f},{:.3f} ...".format(indent, -0.100, -0.100))
        psoc.SetSpeed(-0.100, -0.100)
        sleep(2.0)

        print("{}Set left/right motor speed {:.3f},{:.3f} ...".format(indent, 0.0, 0.0))
        psoc.SetSpeed(0.0, 0.0)
        psoc.MotorDebug(False)
    
    def test_set_control():
        print("{}Clear odometry ...".format(indent))
        print(psoc.GetOdometry())
        psoc.ClearOdometry()
        print(psoc.GetOdometry())

    
    # Just in case ;-)
    psoc.SetSpeed(0.0,0.0)

    print("Test Case 1: read heartbeat ...")
    test_read_heartbeat()

    print("Test Case 2: read infrared distances ...")
    test_read_infrared_distances()

    print("Test Case 3: read ultrasonic distances ...")
    test_read_ultrasonic_distances()

    print("Test Case 4: read odometry ...")
    test_read_odometry()

    print("Test Case 5: read status ...")
    test_read_status()

    print("Test Case 6: set debug ...")
    test_set_debug()

    print("Test Case 7: set speed ...")
    test_set_speed()
    
    print("Test Case 8: set control ...")
    test_set_control()

if __name__ == "__main__":
    from i2c import I2CBus, I2CBusError
    try:
        i2c_bus = I2CBus(I2CBus.DEV_I2C_1)
    except RuntimeError:
        raise I2CBusError()

    try:
        psoc = PsocHw(i2c_bus, PsocHw.PSOC_ADDR)
    except PsocHwError:
        print("Failed to create PsocHw instance")

    psoc.SetSpeed(0.0, 0.0)
    
    test(psoc)

