#!/usr/bin/env python
from __future__ import print_function

import time
import math
import random
import rospy

from hal import HardwareAbstractionLayer, HardwareAbstractionLayerError
from hw.imuhw import ImuHw, ImuHwError
from hw.psochw import PsocHw, PsocHwError
from hw.xv11hw import Xv11Hw, Xv11HwError
from hw.i2c import I2CBus
from utils import Worker
from threading import RLock


class BaseHardwareAbstractionLayerError(HardwareAbstractionLayerError):
    pass


class BaseHardwareAbstractionLayer(HardwareAbstractionLayer):
    def __init__(self, simulated=True):
        HardwareAbstractionLayer.__init__(self, "Base HAL")

        self._simulated = simulated

        i2c_device = rospy.get_param("I2C Device", I2CBus.DEV_I2C_1)

        psoc_addr = rospy.get_param("Psoc I2C Address", PsocHw.PSOC_ADDR)

        imu_mag_addr = rospy.get_param("IMU Mag I2C Address", ImuHw.MAG_ADDR)
        imu_acc_addr = rospy.get_param("IMU Acc I2C Address", ImuHw.ACC_ADDR)

        xv11_port = rospy.get_param("XV11 Port", Xv11Hw.PORT)
        xv11_baud = rospy.get_param("XV11 Baud", Xv11Hw.BAUD)

        # In the simulated mode, the HAL launches two threads (one for left and right wheels) that act independently
        # like an actual robot would in that each wheel calculates and sends an encoder count.  Unlike the real robot,
        # the count is derived from the commanded speed, but its reasonably effective and exercises the entire Arlobot
        # stack all the way to the HAL.
        if self._simulated:
            # Parameters need to calculate encoder counts
            self._diameter = rospy.get_param("Wheel Diameter")
            self._circumference = math.pi * self._diameter
            self._tick_per_revolution = rospy.get_param("Tick Per Revolution")
            self._meter_per_revolution = self._circumference
            self._tick_per_meter = self._tick_per_revolution / self._meter_per_revolution

            # Muxtex lock for exchanging data between the HAL and the wheel threads
            self._lock = RLock()

            # Variables for exchanging speed and count values
            self._left_speed = 0
            self._right_speed = 0
            self._left_count = 0
            self._right_count = 0

            self._last_left_time = 0
            self._last_right_time = 0

            self._left_worker = Worker("Left Wheel", self.__left_wheel_work)
            self._right_worker = Worker("Right Wheel", self.__right_wheel_work)
        else:
            try:
                self._i2c_bus = I2CBus(i2c_device)
            except RuntimeError:
                raise BaseHardwareAbstractionLayerError("Failed to instantiate I2CBus")

            # Instantiate Psoc class
            try:
                self._psoc = PsocHw(self._i2c_bus, psoc_addr)
            except PsocHwError:
                raise BaseHardwareAbstractionLayerError("Failed to instantiate PsocHw")

            '''
            try:
                self._imu = ImuHw(self._i2c_bus, imu_mag_addr, imu_acc_addr)
            except ImuHwError:
                raise BaseHardwareAbstractionLayerError("Failed to instantiate ImuHw")
            '''

            '''
            try:
                self._xv11 = Xv11Hw(xv11_port, xv11_baud)
            except Xv11HwError:
                raise BaseHardwareAbstractionLayerError("Failed to instantiate Xv11Hw")
            '''

    def __left_wheel_work(self):
        now = time.time()
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
        else:
            # Before shutting down, turn off the motor controllers
            self._psoc.DisableMotors()

    def SetSpeed(self, left, right):
        if self._simulated:
            with self._lock:
                self._left_speed = left
                self._right_speed = right
        else:
            self._psoc.SetSpeed(left, right)

        rospy.logdebug("HAL.SetSpeed: {:6.2f}, {:6.2f}".format(left, right))

    def GetCount(self):
        if self._simulated:
            with self._lock:
                # Note: This is a simulation of the encoder counts so quantize them to integer values
                left = self._left_count
                right = self._right_count
        else:
            left, right = self._psoc.GetCount()

        rospy.logdebug("HAL.GetCount: {:6.2f}, {:6.2f}".format(left, right))
        return left, right

    def GetSpeed(self):
        if self._simulated:
            with self._lock:
                left = self._left_speed
                right = self._right_speed
        else:
            left, right = self._psoc.GetSpeed()
        return left, right

    def GetOdometry(self):
        return self._psoc.GetOdometry()

    def GetInfrared(self):
        distances = []
        if self._simulated:
            front = [float(random.randrange(10,81,1))/100 for _ in range (8)]
            rear = [float(random.randrange(10,81,1))/100 for _ in range (8)]
            distances = front + rear
        else:
            distances = self._psoc.GetInfraredDistances()
        return distances

    def GetUltrasonic(self):
        distances = []
        if self._simulated:
            front = [float(random.randrange(2,501,1))/100 for _ in range (8)]
            rear = [float(random.randrange(2,501,1))/100 for _ in range (8)]
            distances = front + rear
        else:
            # Note: There are no left/right infrared sensors, but there are front and back
            distances = self._psoc.GetUlrasonicDistances()

        return distances

    def GetImuSensor(self):
        if self._simulated:
            imu_data = { 'accel' : {'x': 0.0, 'y': 0.0, 'z': 0.0},
                         'mag' : {'x': 0.0, 'y': 0.0, 'z': 0.0},
                         'temp': {'f':0.0, 'c':0.0}}
        else:
            #imu_data = self._imu.GetImuData()
            imu_data = { 'accel' : {'x' : 0.0, 'y' : 0.0, 'z' : 0.0},
                         'mag' : {'x': 0.0, 'y': 0.0, 'z': 0.0},
                         'temp' : {'f': 0.0, 'c': 0.0}}

        return imu_data

    def GetLaserScan(self):
        if self._simulated:
            ranges = [float(random.randrange(6,501,1))/100 for _ in range(360)]
            intensities = [1.0 for _ in range(360)]
            time_offset = 0.0
        else:
            #ranges, intensities, time_offset = self._xv11.GetLaserScan()
            ranges = [0]*360
            intensities = [0]*360
            time_offset = 0.0

        return ranges, intensities, time_offset

    def GetVoltage(self):
        '''
        Originally, the power pi was to be the source of current measurement, but it has not lived up to its reputation
        so an alternate approach is needed

        Consider using an A/D-based current/voltage sensor:
            http://www.robotshop.com/en/current-voltage-sensors.html
        '''
        return 0.0

    def GetCurrent(self):
        '''
        Originally, the power pi was to be the source of current measurement, but it has not lived up to its reputation
        so an alternate approach is needed
        '''
        return 0.0

    def GetTemp(self):
        if self._simulated:
            temp = 0
        else:
            #temp = self._imu.GetTemp()
            temp = 0
        return temp


if __name__ == "__main__":
    base_hal = BaseHardwareAbstractionLayer(False)
    print("Base HAL starting up")
    base_hal.Startup()
    while not base_hal.Ready(True): pass
    print("Base HAL is ready")
    print("Setting speed - left: 0.5, right: 0.5")
    base_hal.SetSpeed(0.5,0.5)
    print("Wait 10 seconds")
    time.sleep(10)
    print("Setting speed - left: 0, right: 0")
    base_hal.SetSpeed(0,0)
    print("Base HAL Shutting down")
    base_hal.Shutdown()
    while not base_hal.Ready(False): pass
    print("Base HAL Shutdown")
