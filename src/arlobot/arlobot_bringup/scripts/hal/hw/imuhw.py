from __future__ import print_function

# can we access the parameter configuration to select the hardware for importing?
#import lm303chw as hardware
from bno055hw import BNO055Hw as Hardware
from bno055hw import BNO055HwError as HardwareError

class ImuHwError(Exception):
    pass

class ImuHw:

    def __init__(self, i2cbus):
        self._i2c_bus = i2cbus
        try:
            self._hardware = Hardware(i2cbus)
        except HardwareError:
            raise ImuHwError("Failed to instantiate hardware")

    def _read_imu(self):
        """
        Returns a dictionary: { 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0},
                                'linear_accel':  {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                'magnetic_field': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                                'euler': {'heading': 0.0, 'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0},
                                'temp': {'f':0.0, 'c':0.0}
                              }
        """

        try:
            orientation_result = self._hardware.read_orientation()
        except AttributeError:
            orientation_result = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
        try:
            acceleration_result = self._hardware.read_acceleration()
        except AttributeError:
            acceleration_result = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        try:
            gyro_result = self._hardware.read_gyroscope()
        except AttributeError:
            gyro_result = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        try:
            mag_result = self._hardware.read_magnetometer()
        except AttributeError:
            mag_result = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        try:
            euler_result = self._hardware.read_euler()
        except AttributeError:
            euler_result = {'heading': 0.0, 'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0}
        try:
            temp_in_celcius = self._hardware.read_temperature()
            temp_in_fahrenheit = (temp_in_celcius - 32.0)*9.0/5.0
            temp_result = {'f': temp_in_fahrenheit, 'c': temp_in_celcius}
        except AttributeError:
            temp_result = {'f': 0.0, 'c': 0.0}

        return {'orientation': orientation_result,
                'linear_accel': acceleration_result,
                'angular_velocity': gyro_result,
                'magnetic_field': mag_result,
                'euler': euler_result,
                'temperature': temp_result}

    def GetOrientation(self):
        imu_data = self._read_imu()
        return imu_data['orientation']

    def GetLinearAccel(self):
        imu_data = self._read_imu()
        return imu_data['linear_accel']

    def GetAngularVelocity(self):
        imu_data = self._read_imu()
        return imu_data['angular_velocity']

    def GetMagneticField(self):
        imu_data = self._read_imu()
        return imu_data['magnetic_field']

    def GetEuler(self):
        imu_data = self._read_imu()
        return imu_data['euler']

    def GetTemp(self):
        imu = self._read_imu()
        return imu['temp']

    def GetImuData(self):
        return self._read_imu()