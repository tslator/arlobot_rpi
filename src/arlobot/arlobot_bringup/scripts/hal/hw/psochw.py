from __future__ import print_function

import time
import math

class PsocHwError(Exception):
    pass


class PsocHw:
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
    __CAL_SAMPLE_BIT = 0x0010

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

        # The following are needed as a workaround for the I2C bus issue that returns wildly large values for the odometry
        # values.  A comparison between I2C and serial and only the I2C bus shows these odd values.  The workaround is to
        # use the last value in the case where the new value is unusable.  It should be pretty close and it can't really
        # be worse.
        self._last_x_dist = 0
        self._last_y_dist = 0
        self._last_heading = 0
        self._last_linear_vel = 0
        self._last_angular_vel = 0

    #------------ Read / Write ---------------

    def _set_device_control(self, value):
        '''
        Set or clear the bits in the control value based on the request
        :param value:
        :return:
        '''
        self._i2c_bus.WriteUint16(self._address, PsocHw.__REGISTER_MAP['DEVICE_CONTROL'], value)

    def _set_debug_control(self, value):
        self._i2c_bus.WriteUint16(self._address, PsocHw.__REGISTER_MAP['DEBUG_CONTROL'], value)

    def DisableMotors(self):
        self._set_device_control(PsocHw.DISABLE_MOTORS)

    def ClearOdometry(self):
        self._set_device_control(PsocHw.__CONTROL_CLEAR_ODOMETRY_BIT)

    def ClearCalibration(self):
        self._set_device_control(PsocHw.__CONTROL_CLEAR_CALIBRATION_BIT)

    def EncoderDebug(self, value=True):
        bitmask = PsocHw.__DBG_ENCODERS_BIT
        if not value:
            bitmask = ~PsocHw.__DBG_ENCODERS_BIT
        self._set_calibration_control(bitmask)

    def PidDebug(self, value=True):
        bitmask = PsocHw.__DBG_PIDS_BIT
        if not value:
            bitmask = ~PsocHw.__DBG_PIDS_BIT
        self._set_calibration_control(bitmask)

    def MotorDebug(self, value=True):
        bitmask = PsocHw.__DBG_MOTORS_BIT
        if not value:
            bitmask = ~PsocHw.__DBG_MOTORS_BIT
        self._set_calibration_control(bitmask)

    def OdometryDebug(self, value=True):
        bitmask = PsocHw.__DBG_ODOM_BIT
        if not value:
            bitmask = ~PsocHw.__DBG_ODOM_BIT
        self._set_calibration_control(bitmask)

    def SampleDebug(self, value=True):
        bitmask = PsocHw.__CAL_SAMPLE_BIT
        if not value:
            bitmask = ~PsocHw.__CAL_SAMPLE_BIT
        self._set_calibration_control(bitmask)

    def SetSpeed(self, left_speed, right_speed):
        '''
        '''
        # Note:  We can save some I2C protocol overhead by sending both values together as a multi-byte transaction
        self._i2c_bus.WriteArray(self._address,
                                 PsocHw.__REGISTER_MAP['LEFT_WHEEL_VELOCITY'],
                                 [left_speed, right_speed],
                                 'f')
        print("{}: PsocHw SetSpeed({}, {})".format(time.time(), left_speed, right_speed))

    #------------ Read Only ---------------

    def _get_device_status(self):
        '''
        '''
        return self._i2c_bus.ReadUint16(self._address, PsocHw.__REGISTER_MAP['DEVICE_STATUS'])

    def _get_calibration_status(self):
        '''
        '''
        return self._i2c_bus.ReadUint16(self._address, PsocHw.__REGISTER_MAP['CALIBRATION_STATUS'])

    def GetCalibrationStatus(self):
        '''
        '''
        return self._get_calibration_status()

    def MotorsInitialized(self):
        '''
        '''
        result = self._get_device_status()
        return result & PsocHw.__DEVICE_STATUS_MOTORS_INIT_BIT

    def MotorsCalibrated(self):
        '''
        '''
        result = self._get_calibration_status()
        return result & PsocHw.__CAL_MOTORS_STATUS_BIT

    def PidsCalibrated(self):
        result = self._get_calibration_status()
        return result & PsocHw.__CAL_PID_GAINS_STATUS_BIT

    def IsCalibrated(self):
        result = self._get_calibration_status()
        return result == PsocHw.__CAL_MOTORS_STATUS_BIT | PsocHw.__CAL_PID_GAINS_STATUS_BIT

    def GetOdometry(self):
        odometry = self._i2c_bus.ReadArray(self._address, PsocHw.__REGISTER_MAP['ODOMETRY'], 5, 'f')

        return odometry

    def GetUlrasonicDistances(self):
        '''
        '''
        # Each of the values is a short value
        #front = self._i2c_bus.ReadArray(self._address, PsocHw.__REGISTER_MAP['FRONT_INFRARED_DISTANCE'], 8, 'H')
        #rear = self._i2c_bus.ReadArray(self._address, PsocHw.__REGISTER_MAP['REAR_INFRARED_DISTANCE'], 8, 'H')

        #return front + rear
        return [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def GetInfraredDistances(self):
        '''
        '''
        # Each of the values is a byte value
        #front = self._i2c_bus.ReadArray(self._address, PsocHw.__REGISTER_MAP['FRONT_ULTRASONIC_DISTANCE'], 8, 'B')
        #rear = self._i2c_bus.ReadArray(self._address, PsocHw.__REGISTER_MAP['REAR_ULTRASONIC_DISTANCE'], 8, 'B')

        #return front + rear
        return [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def GetHeartbeat(self):
        '''
        '''
        return self._i2c_bus.ReadUint32(self._address, PsocHw.__REGISTER_MAP['HEARTBEAT'])


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

    # Test speed minimum value
    psoc.SetSpeed(0.1, 0)
    time.sleep(2.0)
    odometry = tuple(psoc.GetOdometry())
    print("psoc.GetOdometry: {} {} {} {}".format(*odometry))
    #assert(result == -100)

    
    # Test speed median value
    psoc.SetSpeed(0, 0)
    time.sleep(2.0)
    odometry = tuple(psoc.GetOdometry())
    print("psoc.GetOdometry: {} {} {} {}".format(*odometry))
    #assert(result == 0)

    # Test speed maximum value
    psoc.SetSpeed(-0.1, 0)
    time.sleep(2.0)
    odometry = tuple(psoc.GetOdometry())
    print("psoc.GetOdometry: {} {} {} {}".format(*odometry))
    #assert(result == 100)

    psoc.SetSpeed(0, 0)


    # Test read device status
    result = psoc._get_status()
    print("{:x}".format(result))

    # Test read odometry
    odom = psoc.GetOdometry()
    print(odom)

    # Test getting infrared distances
    distances = psoc.GetInfraredDistances()
    print(distances)

    # Test getting ultrasonic distances
    distances = psoc.GetUlrasonicDistances()
    print(distances)

    status = psoc._get_status()
    print("{:x}".format(status))
