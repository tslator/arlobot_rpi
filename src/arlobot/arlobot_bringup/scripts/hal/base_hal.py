#!/usr/bin/env python
from __future__ import print_function

import time
import math
import random
import rospy

from hal import HardwareAbstractionLayer, HardwareAbstractionLayerError
from hw.imuhw import ImuHw, ImuHwError
from hw.powerpihw import PowerPiHw, PowerPiHwError
from hw.psoc4hw import Psoc4Hw, Psoc4HwError
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

        left_psoc4_addr = rospy.get_param("Left Psoc4 I2C Address", Psoc4Hw.LEFT_PSOC4_ADDR)
        right_psoc4_addr = rospy.get_param("Right Psoc4 I2C Address", Psoc4Hw.RIGHT_PSOC4_ADDR)

        imu_mag_addr = rospy.get_param("IMU Mag I2C Address", ImuHw.MAG_ADDR)
        imu_acc_addr = rospy.get_param("IMU Acc I2C Address", ImuHw.ACC_ADDR)

        powerpi_avr_addr = rospy.get_param("Power Pi AVR I2C Address", PowerPiHw.AVR_ADDR)
        powerpi_ina219_addr = rospy.get_param("Power Pi INA219 I2C Address", PowerPiHw.INC219_ADDR)

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

            # Instantiate left and right Psoc4 classes
            try:
                self._left_psoc4 = Psoc4Hw(self._i2c_bus, left_psoc4_addr)
            except Psoc4HwError:
                raise BaseHardwareAbstractionLayerError("Failed to instantiate Psoc4Hw")

            try:
                self._right_psoc4 = Psoc4Hw(self._i2c_bus, right_psoc4_addr)
            except Psoc4HwError:
                raise BaseHardwareAbstractionLayerError("Failed to instantiate Psoc4Hw")

            try:
                self._imu = ImuHw(self._i2c_bus, imu_mag_addr, imu_acc_addr)
            except ImuHwError:
                raise BaseHardwareAbstractionLayerError("Failed to instantiate ImuHw")

            try:
                self._powerpi = PowerPiHw(self._i2c_bus, powerpi_avr_addr, powerpi_ina219_addr)
            except PowerPiHwError:
                raise BaseHardwareAbstractionLayerError("Failed to instantiate PowerPiHw")

            try:
                self._xv11 = Xv11Hw(xv11_port, xv11_baud)
            except Xv11HwError:
                raise BaseHardwareAbstractionLayerError("Failed to instantiate Xv11Hw")

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
            self._xv11.Start()

    def _start(self):
        if self._simulated:
            self._left_worker.start()
            self._right_worker.start()

    def _stop(self):
        if self._simulated:
            self._left_worker.stop()
            self._right_worker.stop()
        else:
            self._xv11.Stop()

    def _shutdown(self):
        if self._simulated:
            pass
        else:
            # Before shutting down, turn off the motor controllers
            self._left_psoc4.SetControl(self._left_psoc4.MOTOR_CONTROLLER_OFF)
            self._right_psoc4.SetControl(self._right_psoc4.MOTOR_CONTROLLER_OFF)

    def SetSpeed(self, left, right):
        if self._simulated:
            with self._lock:
                self._left_speed = left
                self._right_speed = right

            #rospy.logdebug("HAL.SetSpeed: {:6.2f}, {:6.2f}".format(left, right))
        else:
            self._left_psoc4.SetSpeed(left)
            self._right_psoc4.SetSpeed(right)

    def SetAccel(self, left, right):
        if self._simulated:
            pass
        else:
            self._left_psoc4.SetAccel(left)
            self._right_psoc4.SetAccel(right)

    def GetCount(self):
        if self._simulated:
            with self._lock:
                # Note: This is a simulation of the encoder counts so quantize them to integer values
                left = self._left_count
                right = self._right_count
            #rospy.logdebug("HAL.GetCount: {:6.2f}, {:6.2f}".format(left, right))
        else:
            left = self._left_psoc4.GetCount()
            right = self._right_psoc4.GetCount()
        return left, right

    def GetSpeed(self):
        if self._simulated:
            with self._lock:
                left = self._left_speed
                right = self._right_speed
        else:
            left = self._left_psoc4.GetSpeed()
            right = self._right_psoc4.GetSpeed()
        return left, right

    def GetInfrared(self):
        if self._simulated:
            front = [float(random.randrange(10,81,1))/100 for _ in range (8)]
            back = [float(random.randrange(10,81,1))/100 for _ in range (8)]
        else:
            # Note: There are no left/right infrared sensors, but there are front and back
            front = self._left_psoc4.GetInfraredDistances()
            back = self._right_psoc4.GetInfraredDistances()
        return front + back

    def GetUltrasonic(self):
        if self._simulated:
            front = [float(random.randrange(2,501,1))/100 for _ in range (8)]
            back = [float(random.randrange(2,501,1))/100 for _ in range (8)]
        else:
            # Note: There are no left/right infrared sensors, but there are front and back
            front = self._left_psoc4.GetUlrasonicDistances()
            back = self._right_psoc4.GetUlrasonicDistances()

        return front + back

    def GetImuSensor(self):
        if self._simulated:
            imu_data = { 'accel' : {'x': 0.0, 'y': 0.0, 'z': 0.0},
                         'mag' : {'x': 0.0, 'y': 0.0, 'z': 0.0},
                         'temp': {'f':0.0, 'c':0.0}}
        else:
            imu_data = self._imu.GetImuData()

        return imu_data

    def GetLaserScan(self):
        if self._simulated:
            ranges = [float(random.randrange(6,501,1))/100 for _ in range(360)]
            intensities = [1.0 for _ in range(360)]
            time_offset = 0.0
        else:
            ranges, intensities, time_offset = self._xv11.GetLaserScan()
        return ranges, intensities, time_offset

    def GetVoltage(self):
        if self._simulated:
            voltage = 5.0
        else:
            voltage = self._powerpi.GetVoltage()
        return voltage

    def GetCurrent(self):
        if self._simulated:
            current = 2.1
        else:
            current = self._powerpi.GetCurrent()
        return current

    def GetTemp(self):
        if self._simulated:
            temp = 0
        else:
            temp = self._imu.GetTemp()
        return temp


if __name__ == "__main__":
    base_hal = BaseHardwareAbstractionLayer()

    base_hal.Startup()
    while not base_hal.Ready(): pass
    base_hal.SetSpeed({"left" : 1.0, "right" : 1.0})
    base_hal.Shutdown()
    while not base_hal.Ready(): pass
