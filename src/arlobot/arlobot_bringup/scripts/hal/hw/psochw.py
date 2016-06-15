from __future__ import print_function

import time
import math

class PsocHwError(Exception):
    pass


class PsocHw:

    __CONTROL_REGISTER_OFFSET = 0
    __LINEAR_COMMANDED_VELOCITY_OFFSET = 2
    __ANGULAR_COMMANDED_VELOCITY_OFFSET = 6
    __CALIBRATION_PORT_OFFSET = 10
    __DEVICE_STATUS_OFFSET = 12
    __ODOMETRY_OFFSET = 14
    __FRONT_ULTRASONIC_DISTANCE_OFFSET = 34
    __REAR_ULTRASONIC_DISTANCE_OFFSET = 42
    __FRONT_INFRARED_DISTANCE_OFFSET = 50
    __REAR_INFRARED_DISTANCE_OFFSET = 58
    __HEARTBEAT_OFFSET = 66


    __REGISTER_MAP = {#---------- READ/WRITE ------------
                      'CONTROL_REGISTER': __CONTROL_REGISTER_OFFSET,
                      #      - Bit 0: enable / disable the HB25 motors
                      #      - Bit 1: clear odometry
                      #      - Bit 2: calibrate - requests the Psoc to start calibrating
                      
                      'LINEAR_COMMANDED_VELOCITY': __LINEAR_COMMANDED_VELOCITY_OFFSET,
                      'ANGULAR_COMMANDED_VELOCITY': __ANGULAR_COMMANDED_VELOCITY_OFFSET,
                      'CALIBRATION_PORT' : __CALIBRATION_PORT_OFFSET,

                      #----------------------------------
                      #------ READ/WRITE Boundary -------
                      #----------------------------------

                      #----------- READ ONLY ------------
                      'DEVICE_STATUS' : __DEVICE_STATUS_OFFSET,
                      #      - Bit 0: HB25 Motor Controller Initialized
                      #      - Bit 1: Calibrated - indicates whether the calibration values
                      #               have been loaded; 0 - no, 1 - yes
                      #      - Bit 2: Calibrating - indicates when the Psoc is in calibration;
                      #               0 - no, 1 - yes
                      
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
        self._last_x_dist = 0
        self._last_y_dist = 0

    #------------ Read / Write ---------------

    def _set_control(self, value):
        '''
        Set or clear the bits in the control value based on the request
        :param value:
        :return:
        '''
        self._i2c_bus.WriteUint16(self._address, PsocHw.__REGISTER_MAP['CONTROL_REGISTER'], value)

    def DisableMotors(self):
        self._set_control(PsocHw.DISABLE_MOTORS)

    def ClearOdometry(self):
        self._set_control(PsocHw.__CONTROL_CLEAR_ODOMETRY_BIT)

    def SetSpeed(self, linear_speed, angular_speed):
        '''
        '''
        self._i2c_bus.WriteFloat(self._address, PsocHw.__REGISTER_MAP['LINEAR_COMMANDED_VELOCITY'], linear_speed)
        self._i2c_bus.WriteFloat(self._address, PsocHw.__REGISTER_MAP['ANGULAR_COMMANDED_VELOCITY'], angular_speed)

    def SetCalibrationPort(self, value):
        '''
        '''
        self._i2c_bus.WriteUint16(self._address, PsocHw.__REGISTER_MAP['CALIBRATION_PORT'], value)

    def GetCalibrationPort(self):
        '''
        '''
        return self._i2c_bus.ReadUint16(self._address, PsocHw.__REGISTER_MAP['CALIBRATION_PORT'])

    #------------ Read Only ---------------

    def _get_status(self):
        '''
        '''
        return self._i2c_bus.ReadUint16(self._address, PsocHw.__REGISTER_MAP['DEVICE_STATUS'])

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

    def GetOdometry(self):
        # Each of the values is a floating point value
        #  - x distance
        #  - y distance
        #  - heading
        #  - linear velocity
        #  - angular velocity

        # Note: There is an issue with the I2C data where sometimes the values are incredibly large.  Comparing the I2C
        # output simultaneously with the RS-232 output, it appears this is an aberation of the I2C bus.  A simple way
        # around this is to qualify the values received.  Some values, like heading, linear and angular velocity have
        # known limits, but x and y distance are unbounded.  For these though, we still know the worst case increment
        # based on the largest possible linear velocity.
        MAX_LINEAR_DIST = 0.7579
        MIN_LINEAR_DIST = -MAX_LINEAR_DIST
        MAX_LINEAR_VEL = 0.7579
        MIN_LINEAR_VEL = -MAX_LINEAR_VEL
        MAX_ANGULAR_VEL = (2 * 0.7579)/0.403
        MIN_ANGULAR_VEL = -MAX_ANGULAR_VEL

        x_dist, y_dist, heading, linear_vel, angular_vel = self._i2c_bus.ReadArray(self._address, PsocHw.__REGISTER_MAP['ODOMETRY'], 5, 'f')

        delta_x_dist = self._last_x_dist - x_dist
        self._last_x_dist = x_dist
        delta_y_dist = self._last_y_dist - y_dist
        self._last_y_dist = y_dist
   
        if delta_x_dist > MAX_LINEAR_DIST or delta_x_dist < MIN_LINEAR_DIST:
            raise PsocHwError("x dist out-of-range: {}".format(x_dist))
        if delta_y_dist > MAX_LINEAR_DIST or delta_y_dist < MIN_LINEAR_DIST:
            raise PsocHwError("y dist out-of-range: {}".format(y_dist))
        if heading < -math.pi or heading > math.pi:
            raise PsocHwError("heading out-of-range: {}".format(heading))
        if linear_vel < MIN_LINEAR_VEL or linear_vel > MAX_LINEAR_VEL:
            raise PsocHwError("linear vel out-of-range: {}".format(linear_vel))
        if angular_vel < MIN_ANGULAR_VEL or angular_vel > MAX_ANGULAR_VEL:
            raise PsocHwError("angular vel out-of-range: {}".format(angular_vel))
     
        return x_dist, y_dist, heading, linear_vel, angular_vel

    def GetUlrasonicDistances(self):
        '''
        '''
        # Each of the values is a short value
        front = self._i2c_bus.ReadArray(self._address, PsocHw.__REGISTER_MAP['FRONT_INFRARED_DISTANCE'], 8, 'H')
        rear = self._i2c_bus.ReadArray(self._address, PsocHw.__REGISTER_MAP['REAR_INFRARED_DISTANCE'], 8, 'H')

        return front + rear

    def GetInfraredDistances(self):
        '''
        '''
        # Each of the values is a byte value
        front = self._i2c_bus.ReadArray(self._address, PsocHw.__REGISTER_MAP['FRONT_ULTRASONIC_DISTANCE'], 8, 'B')
        rear = self._i2c_bus.ReadArray(self._address, PsocHw.__REGISTER_MAP['REAR_ULTRASONIC_DISTANCE'], 8, 'B')

        return front + rear

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
    x_dist, y_dist, heading, linear, angular = tuple(psoc.GetOdometry())
    print("psoc.GetOdometry: ", x_dist, y_dist, heading, linear, angular)
    #assert(result == -100)

    
    # Test speed median value
    psoc.SetSpeed(0, 0)
    time.sleep(2.0)
    x_dist, y_dist, heading, linear, angular = tuple(psoc.GetOdometry())
    print("psoc.GetOdometry: ", x_dist, y_dist, heading, linear, angular)
    #assert(result == 0)

    # Test speed maximum value
    psoc.SetSpeed(-0.1, 0)
    time.sleep(2.0)
    x_dist, y_dist, heading, linear, angular = tuple(psoc.GetOdometry())
    print("psoc.GetOdometry: ", x_dist, y_dist, heading, linear, angular)
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
