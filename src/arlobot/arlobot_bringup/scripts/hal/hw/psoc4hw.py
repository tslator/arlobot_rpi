from __future__ import print_function

import time
from utils import meter_to_millimeter, millimeter_to_meter


class Psoc4HwError(Exception):
    pass


class Psoc4Hw:
    __REGISTER_MAP = {#---------- READ/WRITE ------------
                      'CONTROL_REGISTER': 0,
                            """
                            - Bit 0: enable / disable the HB25 motors
                            - Bit 1: clear encoder count
                            - Bit 2: calibrate - requests the Psoc to start calibrating
                            - Bit 3: upload calibration
                            - Bit 4: download calibration
                            """
                      'COMMANDED_VELOCITY': 2,
                      'COMMANDED_ACCELERATION': 4,
                      'CALIBRATION_PORT' : 6,

                      #----------------------------------
                      #------ READ/WRITE Boundary -------
                      #----------------------------------

                      #----------- READ ONLY ------------
                      'DEVICE_STATUS' : 6,
                            """
                            - Bit 0: HB25 Motor Controller Initialized
                            - Bit 1: Calibrated - indicates whether the calibration values
                                     have been loaded; 0 - no, 1 - yes
                            - Bit 2: Calibrating - indicates when the Psoc is in calibration;
                                     0 - no, 1 - yes
                            """
                      'MEASURED_COUNT': 8,
                      'MEASURED_VELOCITY': 12,
                      'MEASURED_CNTS_PER_SEC': 14,
                      'ODOMETRY': 16,
                      'ULTRASONIC_DISTANCE': 36,
                      'INFRARED_DISTANCE': 52,
                      'TEST': 60}

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

    LEFT_PSOC4_ADDR = 0x09
    RIGHT_PSOC4_ADDR = 0x08

    def __init__(self, i2cbus, address):
        self._address = address
        self._i2c_bus = i2cbus

    def SetControl(self, value):
        '''
        Set or clear the bits in the control value based on the request
        :param value:
        :return:
        '''
        self._i2c_bus.WriteUint16(self._address, self.__REGISTER_MAP['CONTROL_REGISTER'], value)

    def SetSpeed(self, speed):
        '''
        '''
        self._i2c_bus.WriteInt16(self._address, self.__REGISTER_MAP['COMMANDED_VELOCITY'], meter_to_millimeter(speed))

    def SetAccel(self, acceleration):
        '''
        '''
        self._i2c_bus.WriteInt16(self._address, self.__REGISTER_MAP['COMMANDED_ACCELERATION'], acceleration)

    def SetCalibrationPort(self, value):
        '''
        '''
        self._i2c_bus.WriteUint16(self._addres, self.__REGISTER_MAP['CALIBRATION_PORT'], value)

    def GetCalibrationPort(self):
        '''
        '''
        return self._i2c_buf.ReadUint16(self._address, self.__REGISTER_MAP['CALIBRATION_PORT'])

    def GetCount(self):
        '''
        '''
        return self._i2c_bus.ReadInt32(self._address, self.__REGISTER_MAP['MEASURED_COUNT'])

    def GetSpeed(self):
        '''
        '''
        speed = self._i2c_bus.ReadInt16(self._address, self.__REGISTER_MAP['MEASURED_VELOCITY'])
        return millimeter_to_meter(speed)

    def GetCountsPerSec(self):
        '''
        '''
        cnts_per_sec = self._i2c_buf.ReadInt16(self._address, self.__REGISTER_MAP['MEASURED_CNTS_PER_SEC'])

    def GetOdometry(self):
        # Each of the values is a floating point value
        values = self._i2c_bus.ReadArray(self._address, self._REGISTER_MAP['ODOMETRY'], 5, 'f')

    def GetInfraredDistances(self):
        '''
        '''
        # Each of the values is a byte value
        return self._i2c_bus.ReadArray(self._address, self.__REGISTER_MAP['ULTRASONIC_DISTANCE'], 8, 'b')

    def GetUlrasonicDistances(self):
        '''
        '''
        # Each of the values is a short value
        return self._i2c_bus.ReadArray(self._address, self.__REGISTER_MAP['INFRARED_DISTANCE'], 8, 's')

    def GetStatus(self):
        '''
        '''
        return self._i2c_bus.ReadUint16(self._address, self.__REGISTER_MAP['DEVICE_STATUS'])


if __name__ == "__main__":
    from i2c import I2CBus, I2CBusError
    try:
        i2c_bus = I2CBus(I2CBus.DEV_I2C_1)
    except RuntimeError:
        raise I2CBusError()

    try:
        psoc4 = Psoc4Hw(i2c_bus, Psoc4Hw.LEFT_PSOC4_ADDR)
    except Psoc4HwError:
        print("Failed to create Psoc4Hw instance")

    # Test speed minimum value
    psoc4.SetSpeed(-100)
    result = psoc4.GetSpeed()
    print("psoc4.GetSpeed: ", result)
    #assert(result == -100)

    # Test speed median value
    psoc4.SetSpeed(0)
    result = psoc4.GetSpeed()
    print("psoc4.GetSpeed: ", result)
    #assert(result == 0)

    # Test speed maximum value
    psoc4.SetSpeed(100)
    result = psoc4.GetSpeed()
    print("psoc4.GetSpeed: ", result)
    #assert(result == 100)

    # Test acceleration minimum value
    psoc4.SetAccel(-1000)
    # Test acceleration median value
    psoc4.SetAccel(0)
    # Test acceleration maximum value
    psoc4.SetAccel(1000)

    # Test count is positive after forward move
    start = psoc4.GetCount()
    print("psoc4.GetCount: ", start)
    psoc4.SetSpeed(100)
    time.sleep(5)
    stop = psoc4.GetCount()
    print("psoc4.GetCount: ", stop)
    psoc4.SetSpeed(0)
    #assert(start - stop > 0)

    # Test count is negative after reverse move
    start = psoc4.GetCount()
    print("psoc4.GetCount: ", start)
    psoc4.SetSpeed(-100)
    time.sleep(5)
    end = psoc4.GetSpeed()
    print("psoc4.GetSpeed: ", end)
    psoc4.SetSpeed(0)
    #assert(start - end < 0)

    # Test getting infrared distances
    distances = psoc4.GetInfraredDistances()
    print(distances)

    # Test getting ultrasonic distances
    distances = psoc4.GetUlrasonicDistances()
    print(distances)

    status = psoc4.GetStatus()
    print(status)
