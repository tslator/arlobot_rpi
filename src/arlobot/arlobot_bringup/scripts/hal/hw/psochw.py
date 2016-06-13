from __future__ import print_function

import time
from utils import meter_to_millimeter, millimeter_to_meter


class PsocHwError(Exception):
    pass


class PsocHw:

    __CONTROL_REGISTER_OFFSET = 0
    __LEFT_COMMANDED_VELOCITY_OFFSET = 2
    __RIGHT_COMMANDED_VELOCITY_OFFSET = 4
    __CALIBRATION_PORT_OFFSET = 6
    __DEVICE_STATUS_OFFSET = 8
    __LEFT_MEASURED_VELOCITY_OFFSET = 10
    __RIGHT_MEASURED_VELOCITY_OFFSET = 12
    __ODOMETRY_OFFSET = 14
    __FRONT_ULTRASONIC_DISTANCE_OFFSET = 34
    __REAR_ULTRASONIC_DISTANCE_OFFSET = 42
    __FRONT_INFRARED_DISTANCE_OFFSET = 50
    __REAR_INFRARED_DISTANCE_OFFSET = 58
    __HEARTBEAT_OFFSET = 66


    __REGISTER_MAP = {#---------- READ/WRITE ------------
                      'CONTROL_REGISTER': __CONTROL_REGISTER_OFFSET,
                            """
                            - Bit 0: enable / disable the HB25 motors
                            - Bit 1: clear encoder count
                            - Bit 2: calibrate - requests the Psoc to start calibrating
                            - Bit 3: upload calibration
                            - Bit 4: download calibration
                            """
                      'LEFT_COMMANDED_VELOCITY': __LEFT_COMMANDED_VELOCITY_OFFSET,
                      'RIGHT_COMMANDED_VELOCITY': __RIGHT_COMMANDED_VELOCITY_OFFSET,
                      'CALIBRATION_PORT' : __CALIBRATION_PORT_OFFSET,

                      #----------------------------------
                      #------ READ/WRITE Boundary -------
                      #----------------------------------

                      #----------- READ ONLY ------------
                      'DEVICE_STATUS' : __DEVICE_STATUS_OFFSET,
                            """
                            - Bit 0: HB25 Motor Controller Initialized
                            - Bit 1: Calibrated - indicates whether the calibration values
                                     have been loaded; 0 - no, 1 - yes
                            - Bit 2: Calibrating - indicates when the Psoc is in calibration;
                                     0 - no, 1 - yes
                            """
                      'LEFT_MEASURED_VELOCITY': __LEFT_MEASURED_VELOCITY_OFFSET,
                      'RIGHT_MEASURED_VELOCITY': __RIGHT_MEASURED_VELOCITY_OFFSET,
                      'ODOMETRY': __ODOMETRY_OFFSET,
                      'FRONT_ULTRASONIC_DISTANCE': __FRONT_ULTRASONIC_DISTANCE_OFFSET,
                      'REAR_ULTRASONIC_DISTANCE': __REAR_ULTRASONIC_DISTANCE_OFFSET,
                      'FRONT_INFRARED_DISTANCE': __FRONT_INFRARED_DISTANCE_OFFSET,
                      'REAR_INFRARED_DISTANCE': __REAR_INFRARED_DISTANCE_OFFSET,
                      'HEARTBEAT': __HEARTBEAT_OFFSET}

    __CONTROL_DISABLE_MOTORS_BIT = 0x0001
    __CONTROL_CLEAR_ODOMETRY_BIT = 0x0002
    __CONTROL_CALIBRATE_BIT = 0x0004

    __STATUS_MOTORS_INIT_BIT = 0x0001
    __STATUS_CALIBRATED_BIT = 0x0002
    __STATUS_CALIBRATING_BIT = 0x0004

    DISABLE_MOTORS = __CONTROL_DISABLE_MOTORS_BIT
    CLEAR_ODOMETRY = __CONTROL_CLEAR_ODOMETRY_BIT
    PERFORM_CALIBRATION = __CONTROL_CALIBRATE_BIT

    PSOC_ADDR = 0x08

    def __init__(self, i2cbus, address):
        self._address = address
        self._i2c_bus = i2cbus

    #------------ Read / Write ---------------

    def _set_control(self, value):
        '''
        Set or clear the bits in the control value based on the request
        :param value:
        :return:
        '''
        self._i2c_bus.WriteUint16(self._address, self.__REGISTER_MAP['CONTROL_REGISTER'], value)

    def DisableMotors(self):
        self._set_control(PsocHw.DISABLE_MOTORS)

    def ClearOdometry(self):
        self._set_control(PsocHw.__CONTROL_CLEAR_ODOMETRY_BIT)

    def SetSpeed(self, left_speed, right_speed):
        '''
        '''
        self._i2c_bus.WriteInt16(self._address, self.__REGISTER_MAP['LEFT_COMMANDED_VELOCITY'], meter_to_millimeter(left_speed))
        self._i2c_bus.WriteInt16(self._address, self.__REGISTER_MAP['RIGHT_COMMANDED_VELOCITY'], meter_to_millimeter(right_speed))

    def SetCalibrationPort(self, value):
        '''
        '''
        self._i2c_bus.WriteUint16(self._address, self.__REGISTER_MAP['CALIBRATION_PORT'], value)

    def GetCalibrationPort(self):
        '''
        '''
        return self._i2c_buf.ReadUint16(self._address, self.__REGISTER_MAP['CALIBRATION_PORT'])

    #------------ Read Only ---------------

    def _get_status(self):
        '''
        '''
        return self._i2c_bus.ReadUint16(self._address, self.__REGISTER_MAP['DEVICE_STATUS'])

    def MotorsInitialized(self):
        '''
        '''
        result = self._get_status()
        return result & PsocHw.__STATUS_MOTORS_INIT_BIT

    def MotorsCalibrated(self):
        '''
        '''
        result = self._get_status()
        return result & PsocHw.__STATUS_CALIBRATED_BIT

    def MotorsCalibrating(self):
        '''
        '''
        result = self._get_status()
        return result & PsocHw.__STATUS_CALIBRATING_BIT

    def GetSpeed(self):
        '''
        '''
        left_speed = self._i2c_bus.ReadInt16(self._address, self.__REGISTER_MAP['LEFT_MEASURED_VELOCITY'])
        right_speed = self._i2c_bus.ReadInt16(self._address, self.__REGISTER_MAP['RIGHT_MEASURED_VELOCITY'])
        return millimeter_to_meter(left_speed), millimeter_to_meter(right_speed)

    def GetOdometry(self):
        # Each of the values is a floating point value
        #  - x distance
        #  - y distance
        #  - heading
        #  - linear velocity
        #  - angular velocity
        return self._i2c_bus.ReadArray(self._address, self._REGISTER_MAP['ODOMETRY'], 5, 'f')

    def GetUlrasonicDistances(self):
        '''
        '''
        # Each of the values is a short value
        front = self._i2c_bus.ReadArray(self._address, self.__REGISTER_MAP['FRONT_INFRARED_DISTANCE'], 8, 'H')
        rear = self._i2c_bus.ReadArray(self._address, self.__REGISTER_MAP['REAR_INFRARED_DISTANCE'], 8, 'H')

        return front + rear

    def GetInfraredDistances(self):
        '''
        '''
        # Each of the values is a byte value
        front = self._i2c_bus.ReadArray(self._address, self.__REGISTER_MAP['FRONT_ULTRASONIC_DISTANCE'], 8, 'B')
        rear = self._i2c_bus.ReadArray(self._address, self.__REGISTER_MAP['REAR_ULTRASONIC_DISTANCE'], 8, 'B')

        return front + rear

    def GetHeartbeat(self):
        '''
        '''
        return self._i2c_bus.ReadUint32(self._address, self.__REGISTER_MAP['HEARTBEAT'])


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
    psoc.SetSpeed(200, 200)
    left, right = psoc.GetSpeed()
    print("psoc.GetSpeed: ", left, right)
    #assert(result == -100)

    # Test speed median value
    psoc.SetSpeed(0)
    left, right = psoc.GetSpeed()
    print("psoc.GetSpeed: ", left, right)
    #assert(result == 0)

    # Test speed maximum value
    psoc.SetSpeed(-200, -200)
    left, right = psoc.GetSpeed()
    print("psoc.GetSpeed: ", left, right)
    #assert(result == 100)

    # Test read/write to calibration port
    psoc.SetCalibrationPort(0xff)
    result = psoc.GetCalibrationPort()

    # Test read device status
    result = psoc._get_status()

    # Test read odometry
    odom = psoc.GetOdometry()

    # Test getting infrared distances
    distances = psoc.GetInfraredDistances()
    print(distances)

    # Test getting ultrasonic distances
    distances = psoc.GetUlrasonicDistances()
    print(distances)

    status = psoc._get_status()
    print(status)
