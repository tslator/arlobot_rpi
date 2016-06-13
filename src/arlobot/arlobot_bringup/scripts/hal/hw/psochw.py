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
                      'CONTROL_REGISTER': PsocHw.__CONTROL_REGISTER_OFFSET,
                            """
                            - Bit 0: enable / disable the HB25 motors
                            - Bit 1: clear encoder count
                            - Bit 2: calibrate - requests the Psoc to start calibrating
                            - Bit 3: upload calibration
                            - Bit 4: download calibration
                            """
                      'LEFT_COMMANDED_VELOCITY': 2,
                      'RIGHT_COMMANDED_VELOCITY': 4,
                      'CALIBRATION_PORT' : 6,

                      #----------------------------------
                      #------ READ/WRITE Boundary -------
                      #----------------------------------

                      #----------- READ ONLY ------------
                      'DEVICE_STATUS' : 8,
                            """
                            - Bit 0: HB25 Motor Controller Initialized
                            - Bit 1: Calibrated - indicates whether the calibration values
                                     have been loaded; 0 - no, 1 - yes
                            - Bit 2: Calibrating - indicates when the Psoc is in calibration;
                                     0 - no, 1 - yes
                            """
                      'LEFT_MEASURED_VELOCITY': 10,
                      'RIGHT_MEASURED_VELOCITY': 12,
                      'ODOMETRY': 14,
                      'FRONT_ULTRASONIC_DISTANCE': 34,
                      'REAR_ULTRASONIC_DISTANCE': 42,
                      'FRONT_INFRARED_DISTANCE': 50,
                      'REAR_INFRARED_DISTANCE': 58,
                      'HEARTBEAT': 66}

    __MOTOR_CONTROLLER_BIT = 0x0001
    __CLEAR_ENCODER_BIT = 0x0002
    __CALIBRATE_BIT = 0x0004
    __UPLOAD_CALIBRATION_BIT = 0x0008
    __DOWNLOAD_CALIBRATION_BIT = 0x0010

    __MOTOR_CONTROLLER_INIT_BIT = 0x0001
    __CALIBRATED_BIT = 0x0002
    __CALIBRATING_BIT = 0x0004

    MOTOR_CONTROLLER_ON = __MOTOR_CONTROLLER_BIT
    CLEAR_ENCODER_COUNT = __CLEAR_ENCODER_BIT
    PERFORM_CALIBRATION = __CALIBRATE_BIT
    UPLOAD_CALIBRATION = __UPLOAD_CALIBRATION_BIT
    DOWNLOAD_CALIBRATION = __DOWNLOAD_CALIBRATION_BIT

    PSOC_ADDR = 0x08

    def __init__(self, i2cbus, address):
        self._address = address
        self._i2c_bus = i2cbus

    #------------ Read / Write ---------------

    def SetControl(self, value):
        '''
        Set or clear the bits in the control value based on the request
        :param value:
        :return:
        '''
        self._i2c_bus.WriteUint16(self._address, self.__REGISTER_MAP['CONTROL_REGISTER'], value)

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

    def GetStatus(self):
        '''
        '''
        return self._i2c_bus.ReadUint16(self._address, self.__REGISTER_MAP['DEVICE_STATUS'])

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
        front = self._i2c_bus.ReadArray(self._address, self.__REGISTER_MAP['FRONT_INFRARED_DISTANCE'], 8, 's')
        rear = self._i2c_bus.ReadArray(self._address, self.__REGISTER_MAP['REAR_INFRARED_DISTANCE'], 8, 's')

        return front + rear

    def GetInfraredDistances(self):
        '''
        '''
        # Each of the values is a byte value
        front = self._i2c_bus.ReadArray(self._address, self.__REGISTER_MAP['FRONT_ULTRASONIC_DISTANCE'], 8, 'b')
        rear = self._i2c_bus.ReadArray(self._address, self.__REGISTER_MAP['REAR_ULTRASONIC_DISTANCE'], 8, 'b')

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
    result = psoc.GetStatus()

    # Test read odometry
    odom = psoc.GetOdometry()

    # Test getting infrared distances
    distances = psoc.GetInfraredDistances()
    print(distances)

    # Test getting ultrasonic distances
    distances = psoc.GetUlrasonicDistances()
    print(distances)

    status = psoc.GetStatus()
    print(status)
