from __future__ import print_function

import time


class Psoc4HwError(Exception):
    pass


class Psoc4Hw:
    __REGISTER_MAP = {#---------- READ/WRITE ------------
                      'CONTROL_REGISTER': 0,
                      'COMMANDED_VELOCITY': 2,
                      'COMMANDED_ACCELERATION': 4,

                      #----------------------------------
                      #------ READ/WRITE Boundary -------
                      #----------------------------------

                      #----------- READ ONLY ------------
                      'DEVICE_STATUS' : 6,
                      'MEASURED_COUNT': 8,
                      'MEASURED_VELOCITY': 12,
                      'ULTRASONIC_DISTANCE': 14,
                      'INFRARED_DISTANCE': 30,
                      'TEST': 38}

    __MOTOR_CONTROLLER_BIT = 0x0001
    __ENCODER_BIT = 0x0002

    MOTOR_CONTROLLER_ON = __MOTOR_CONTROLLER_BIT
    MOTOR_CONTROLLER_OFF = ~MOTOR_CONTROLLER_ON

    LEFT_PSOC4_ADDR = 0x08
    RIGHT_PSOC4_ADDR = 0x09

    def __init__(self, i2cbus, address):
        self._address = address
        self._control = self.MOTOR_CONTROLLER_ON

        self._i2c_bus = i2cbus

    def SetControl(self, value):
        '''
        Set or clear the bits in the control value based on the request
        :param value:
        :return:
        '''
        if value == self.MOTOR_CONTROLLER_ON:
            self._control |= value
        elif value == self.MOTOR_CONTROLLER_OFF:
            self._control &= ~(value)

        self._i2c_bus.WriteUint16(self._address, self.__REGISTER_MAP['CONTROL_REGISTER'], self._control)

    def SetSpeed(self, speed):
        '''
        w aa oo vv
        aa - address
        oo - offset
        vv - velocity
        '''
        self._i2c_bus.WriteUint16(self._address, self.__REGISTER_MAP['COMMANDED_VELOCITY'], speed)

    def SetAccel(self, acceleration):
        '''
        w aa oo cc
        '''
        self._i2c_bus.WriteUint16(self._address, self.__REGISTER_MAP['COMMANDED_ACCELERATION'], acceleration)

    def GetCount(self):
        '''
        w aa r oo cc cc cc cc
        '''
        return self._i2c_bus.ReadUint32(self._address, self.__REGISTER_MAP['MEASURED_COUNT'])

    def GetSpeed(self):
        '''
        w aa r oo vv
        '''
        return self._i2c_bus.ReadUint16(self._address, self.__REGISTER_MAP['MEASURED_VELOCITY'])

    def GetInfraredDistances(self):
        '''
        w aa r oo 11 22 33 44 55 66 77 88
        '''
        return self._i2c_bus.ReadArray(self._address, self.__REGISTER_MAP['ULTRASONIC_DISTANCE'])

    def GetUlrasonicDistances(self):
        '''
        w aa r oo 11 22 33 44 55 66 77 88
        '''
        return self._i2c_bus.ReadArray(self._address, self.__REGISTER_MAP['INFRARED_DISTANCE'])

    def GetStatus(self):
        '''
        w aa r oo vv vv
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
