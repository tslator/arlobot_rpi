from __future__ import print_function

# can we access the parameter configuration to select the hardware for importing?
#import lm303chw as hardware
import bno055hw as hardware

class HardwareError(Exception):
    pass

class ImuHwError(Exception):
    pass

class ImuHw:

    def __init__(self, i2cbus):
        self._i2c_bus = i2cbus
        hardware.initialize(i2cbus)

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
        orientation_result = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}
        acceleration_result = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        gyro_result = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        mag_result = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        euler_result = {'heading': 0.0, 'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0}
        temp_result = {'f': 0.0, 'c': 0.0}

        if hasattr(hardware, "read_orientation"):
            orientation_result = hardware.read_orientation()
        if hasattr(hardware, "read_acceleration"):
            acceleration_result = hardware.read_acceleration()
        if hasattr(hardware, "read_gyroscope"):
            gyro_result = hardware.read_gyroscope()
        if hasattr(hardware, "read_magnetometer"):
            mag_result = hardware.read_magnetometer()
        if hasattr(hardware, 'read_euler'):
            euler_result = hardware.read_euler()
        if hasattr(hardware, "read_temperature"):
            temp_in_celcius = hardware.read_temperature()
            temp_in_fahrenheit = (temp_in_celcius - 32.0)*9.0/5.0
            temp_result = {'f': temp_in_fahrenheit, 'c': temp_in_celcius}

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