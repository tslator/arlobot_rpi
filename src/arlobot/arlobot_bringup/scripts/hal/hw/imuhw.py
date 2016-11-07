from __future__ import print_function

from bno055hw import BNO055Hw as Hardware
from bno055hw import BNO055HwError as HardwareError

class ImuHwError(Exception):
    pass

class ImuHw:

    def __init__(self, i2cbus=None):
        try:
            self._hardware = Hardware()
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
            temp_result = self._hardware.read_temperature()
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
        return imu['temperature']

    def GetImuData(self):
        return self._read_imu()

#--------------------------------------------------------------------------------------------------
# Module Test
#--------------------------------------------------------------------------------------------------

def test(imuhw):
    def test_get_orientation():
        print(imuhw.GetOrientation())

    def test_get_linear_accel():
        print(imuhw.GetLinearAccel())

    def test_get_angular_velocity():
        print(imuhw.GetAngularVelocity())

    def test_get_magnetic_field():
        print(imuhw.GetMagneticField())

    def test_get_euler():
        print(imuhw.GetEuler())

    def test_get_temp():
        print(imuhw.GetTemp())

    def test_get_imu_data():
        print(imuhw.GetImuData())

    print("Test Case: 1 get orientation")
    test_get_orientation()

    print("Test Case 2: get linear accel")
    test_get_linear_accel()

    print("Test Case 3: get angular velocity")
    test_get_angular_velocity()

    print("Test Case 4: get magnetic field")
    test_get_magnetic_field()

    print("Test Case 5: get euler")
    test_get_euler()

    print("Test Case 6: get temp")
    test_get_temp()

    print("Test Case 7: get imu data")
    test_get_imu_data()


if __name__ == "__main__":
    print("Instantiating ImuHw ... this takes a few seconds.")
    try:
        imuhw = ImuHw()
    except ImuHwError as e:
        print(e.args)
        sys.exit(1)

    print("Performing module tests ...")
    test(imuhw)    
