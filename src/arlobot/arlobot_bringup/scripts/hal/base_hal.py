#!/usr/bin/env python
from __future__ import print_function

from time import time, sleep
from math import pi
from random import randrange
import rospy

from hal_protocol import HALProtocol, HALProtocolError
from hw.imuhw import ImuHw, ImuHwError
from hw.psochw import PsocHw, PsocHwError
from hw.i2c import I2CBus, I2CBusError
from scripts.utils.arlothreading import StoppableThread
from threading import RLock


class BaseHardwareAbstractionLayerError(HALProtocolError):
    pass


class BaseHardwareAbstractionLayer(HALProtocol):
    def __init__(self, simulated=False):
        HALProtocol.__init__(self, "Base HAL")

        self._simulated = simulated

        psoc_i2c_device = rospy.get_param("Psoc I2C Device", I2CBus.DEV_I2C_1)
        imu_i2c_device = rospy.get_param("Imu I2C Device", I2CBus.DEV_I2C_1)

        psoc_addr = rospy.get_param("Psoc I2C Address", PsocHw.PSOC_ADDR)

        # In the simulated mode, the HAL launches two threads (one for left and right wheels) that act independently
        # like an actual robot would in that each wheel calculates and sends an encoder count.  Unlike the real robot,
        # the count is derived from the commanded speed, but its reasonably effective and exercises the entire Arlobot
        # stack all the way to the HAL.
        if self._simulated:
            # Parameters need to calculate encoder counts
            self._diameter = rospy.get_param("Wheel Diameter")
            self._circumference = pi * self._diameter
            self._tick_per_revolution = rospy.get_param("Tick Per Revolution")
            self._meter_per_revolution = self._circumference
            self._tick_per_meter = self._tick_per_revolution / self._meter_per_revolution
            self._track_width = rospy.get_param("Track Width", 1)

            # Muxtex lock for exchanging data between the HAL and the wheel threads
            self._lock = RLock()

            # Variables for exchanging speed and count values
            self._linear_speed = 0
            self._angular_speed = 0
            self._left_speed = 0
            self._right_speed = 0
            self._left_count = 0
            self._right_count = 0

            self._last_left_time = 0
            self._last_right_time = 0

            self._left_worker = StoppableThread("Left Wheel", self.__left_wheel_work)
            self._right_worker = StoppableThread("Right Wheel", self.__right_wheel_work)
        else:
            try:
                self._imu = ImuHw()
            except ImuHwError as e:
                raise BaseHardwareAbstractionLayerError("Failed to instantiate ImuHw: {}".format(e.args))

            try:
                self._i2c_bus_1 = I2CBus(psoc_i2c_device)
            except RuntimeError as e:
                raise BaseHardwareAbstractionLayerError("Failed to instantiate I2CBus {} - {}".format(psoc_i2c_device, e.args))
            
            try:
                self._psoc = PsocHw(self._i2c_bus_1, psoc_addr)
            except PsocHwError as e:
                raise BaseHardwareAbstractionLayerError("Failed to instantiate PsocHw - {}".format(e.args))

    def __left_wheel_work(self):
        now = time()
        delta = now - self._last_left_time
        with self._lock:
            self._left_count += self._left_speed * delta * self._tick_per_meter
        self._last_left_time = now

    def __right_wheel_work(self):
        now = time.time()
        delta = now - self._last_right_time
        with self._lock:
            self._right_count += self._right_speed * delta * self._tick_per_meter
        self._last_right_time = now

    def _init(self):
        # The XV11 runs its own thread to read from the serial port - it needs to be started
        if self._simulated:
            pass
        else:
            #self._xv11.Start()
            pass

    def _start(self):
        if self._simulated:
            self._left_worker.start()
            self._right_worker.start()

    def _stop(self):
        if self._simulated:
            self._left_worker.stop()
            self._right_worker.stop()
        else:
            #self._xv11.Stop()
            pass

    def _shutdown(self):
        if self._simulated:
            pass

    def SetSpeed(self, left, right):
        if self._simulated:
            with self._lock:
                self._left_speed = left
                self._right_speed = right
        else:
            self._psoc.SetSpeed(left, right)

        rospy.logdebug("HAL.SetSpeed: {:6.2f}, {:6.2f}".format(left, right))

    def GetOdometry(self):
        if self._simulated:
            with self._lock:
                left_speed = 0
                right_speed = 0
                left_distance = 0
                right_distance = 0
                heading = 0
                return [left_speed, right_speed, left_distance, right_distance, heading]
        else:
            return self._psoc.GetOdometry()

    def GetInfrared(self):
        distances = []
        if self._simulated:
            front = [float(randrange(10,81,1))/100 for _ in range (8)]
            rear = [float(randrange(10,81,1))/100 for _ in range (8)]
            distances = front + rear
        else:
            distances = self._psoc.GetInfraredDistances()
        return distances

    def GetUltrasonic(self):
        distances = []
        if self._simulated:
            front = [float(randrange(2,501,1))/100 for _ in range (8)]
            rear = [float(randrange(2,501,1))/100 for _ in range (8)]
            distances = front + rear
        else:
            # Note: There are no left/right infrared sensors, but there are front and back
            distances = self._psoc.GetUltrasonicDistances()

        return distances

    def GetImuSensor(self):
        if self._simulated:
            imu_data = { 'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0},
                         'linear_accel':  {'x': 0.0, 'y': 0.0, 'z': 0.0},
                         'angular_velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                         'magnetic_field': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                         'euler': {'heading': 0.0, 'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0},
                         'temperature': {'f':0.0, 'c':0.0}
                         }
        else:
            imu_data = self._imu.GetImuData()

        return imu_data

    def GetLaserScan(self):
        if self._simulated:
            ranges = [float(randrange(6,501,1))/100 for _ in range(360)]
            intensities = [1.0 for _ in range(360)]
            time_offset = 0.0
        else:
            #ranges, intensities, time_offset = self._xv11.GetLaserScan()
            ranges = [0]*360
            intensities = [0]*360
            time_offset = 0.0

        return ranges, intensities, time_offset

    def GetVoltage(self):
        """
        Consider using an A/D-based current/voltage sensor:
            http://www.robotshop.com/en/current-voltage-sensors.html
        """
        return 0.0

    def GetCurrent(self):
        """
        Consider using an A/D-based current/voltage sensor:
            http://www.robotshop.com/en/current-voltage-sensors.html
        """
        return 0.0

    def GetTemp(self):
        if self._simulated:
            temp = 0
        else:
            temp = self._imu.GetTemp()
        return temp

    def GetPsocCalibration(self):
        if self._simulated:
            return {'calibrated': True,
                    'motors': True,
                    'pids': True,
                    'linear': True,
                    'angular': True
                   }
        else:
            motors = self._psoc.MotorsCalibrated()
            pids = self._psoc.PidsCalibrated()
            linear = self._psoc.LinearCalibrated()
            angular = self._psoc.AngularCalibrated()
            # A dict of the psoc calibration details:
            return {'calibrated': motors and pids and linear and angular,
                    'motors': motors,
                    'pids': pids,
                    'linear': linear,
                    'angular': angular
                   }

    def GetImuCalibration(self):
        if self._simulated:
            return {'calibrated': True,
                    'sys': True,
                    'gyro': True,
                    'accel': True,
                    'mag': True}
        else:
            cal = self._imu.GetCalibration()
            return {'calibrated': cal['sys'] and cal['gyro'] and cal['accel'] and cal['mag'],
                    'sys': cal['sys'],
                    'gyro': cal['gyro'],
                    'accel': cal['accel'],
                    'mag': cal['mag']}


if __name__ == "__main__":
    base_hal = BaseHardwareAbstractionLayer(False)
    print("Base HAL starting up")
    base_hal.Startup()
    while not base_hal.Ready(True): pass
    print("Base HAL is ready")
    print("Setting speed - left: 0.5, right: 0.5")
    base_hal.SetSpeed(0.5,0.5)
    print("Wait 10 seconds")
    sleep(10)
    print("Setting speed - left: 0, right: 0")
    base_hal.SetSpeed(0,0)
    print("Base HAL Shutting down")
    base_hal.Shutdown()
    while not base_hal.Ready(False): pass
    print("Base HAL Shutdown")
