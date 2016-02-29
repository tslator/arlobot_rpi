#!/usr/bin/env python
from __future__ import print_function

import time
import math
import rospy

from hw.imuhw import ImuHw, ImuHwError
from hw.powerpihw import PowerPiHw
from hw.psoc4hw import Psoc4Hw, Psoc4HwError
from hw.usb2gpiohw import UsbToGpioHw, UsbToGpioHwError
from hw.xv11hw import Xv11Hw, Xv11HwError
from hw.i2c import I2CBus
from utils import Worker
from threading import RLock


class HardwareAbstractionLayerError(Exception):
    pass


class HardwareAbstractionLayer(object):

    __HAL_STATE_INITIAL = "INITIAL"
    __HAL_STATE_STOPPED = "STOPPED"
    __HAL_STATE_RUNNING = "RUNNING"
    __HAL_STATE_SHUTDOWN = "SHUTDOWN"

    def __init__(self, simulated=True):
        self._simulated = simulated
        self._hal_state = HardwareAbstractionLayer.__HAL_STATE_INITIAL

        i2c_device = rospy.get_param("~baseconfig/rpiI2CDevice", I2CBus.DEV_I2C_1)

        left_psoc4_addr = rospy.get_param("~baseconfig/leftboardI2CAddress", Psoc4Hw.LEFT_PSOC4_ADDR)
        right_psoc4_addr = rospy.get_param("~baseconfig/rightboardI2CAddress", Psoc4Hw.RIGHT_PSOC4_ADDR)

        imu_mag_addr = rospy.get_param("~baseconfig/imuI2CMagAddress", ImuHw.MAG_ADDR)
        imu_acc_addr = rospy.get_param("~baseconfig/imuI2CAccAddress", ImuHw.ACC_ADDR)
        powerpi_avr_addr = rospy.get_param("~baseconfig/powerI2CAVR", PowerPiHw.AVR_ADDR)
        powerpi_ina219_addr = rospy.get_param("~baseconfig/powerI2CINA219", PowerPiHw.INC219_ADDR)
        xv11_port = rospy.get_param("~baseconfig/xv11Port", Xv11Hw.PORT)
        xv11_baud = rospy.get_param("~baseconfig/xv11Baud", Xv11Hw.BAUD)

        if self._simulated:
            self._diameter = 0.1524
            self._circumference = math.pi * self._diameter
            self._tick_per_revolution = 72
            self._meter_per_revolution = self._circumference
            self._tick_per_meter = self._tick_per_revolution / self._meter_per_revolution
            self._lock = RLock()

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
                raise HardwareAbstractionLayer("Failed to instantiate I2CBus")

            # Instantiate left and right Psoc4 classes
            try:
                self._left_psoc4 = Psoc4Hw(self._i2c_bus, left_psoc4_addr)
            except Psoc4HwError:
                raise HardwareAbstractionLayerError("Failed to instantiate Psoc4Hw")

            try:
                self._right_psoc4 = Psoc4Hw(self._i2c_bus, right_psoc4_addr)
            except Psoc4HwError:
                raise HardwareAbstractionLayerError("Failed to instantiate Psoc4Hw")

            try:
                self._imu = ImuHw(self._i2c_bus, imu_mag_addr, imu_acc_addr)
            except ImuHwError:
                raise HardwareAbstractionLayerError("Failed to instantiate ImuHw")

            try:
                self._powerpi = PowerPiHw(self._i2c_bus, powerpi_avr_addr, powerpi_ina219_addr)
            except PowerPiHw:
                raise HardwareAbstractionLayerError("Failed to instantiate PowerPiHw")

            try:
                self._xv11 = Xv11Hw(xv11_port, xv11_baud)
            except Xv11HwError:
                raise HardwareAbstractionLayerError("Failed to instantiate Xv11Hw")

            try:
                self._usb2gpio = UsbToGpioHw()
            except UsbToGpioHwError:
                raise HardwareAbstractionLayerError("Failed to instantiate UsbToGpioHw")


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

    def __millimeter_to_meter(self, millimeter):
        return millimeter / 1000.0

    def __meter_to_millimeter(self, millimeter):
        return int(millimeter * 1000)

    def _initialize(self):
        # For doing any initialization that is subject to startup.  Can't think of anything right now.
        pass

    def Startup(self):
        rospy.loginfo("HAL starting up ...")

        if self._hal_state is HardwareAbstractionLayer.__HAL_STATE_RUNNING:
            rospy.logwarn("Attempt to startup HAL while it is running - No action taken")
            return

        # Handle case when state is INITIAL - the shared object state needs to be setup
        if self._hal_state is HardwareAbstractionLayer.__HAL_STATE_INITIAL:
            self._initialize()
            rospy.loginfo("HAL initialized")

            self._hal_state = HardwareAbstractionLayer.__HAL_STATE_STOPPED
            rospy.loginfo("HAL is stopped")

        # Handle case when the shared object state is initialize and ready for the next level of startup
        if self._hal_state is HardwareAbstractionLayer.__HAL_STATE_STOPPED:

            # Do other initialization/startup here, like starting the wheel threads when using stubs
            if self._simulated:
                self._left_worker.start()
                self._right_worker.start()

            self._hal_state = HardwareAbstractionLayer.__HAL_STATE_RUNNING

            rospy.loginfo("HAL is running")

    def Ready(self, startup_shutdown):
        if startup_shutdown:
            return self._hal_state == HardwareAbstractionLayer.__HAL_STATE_RUNNING
        else:
            return self._hal_state == HardwareAbstractionLayer.__HAL_STATE_SHUTDOWN

    def Shutdown(self):
        rospy.loginfo("HAL shutting down ...")

        if self._simulated:
            self._left_worker.stop()
            self._right_worker.stop()

        self._hal_state = HardwareAbstractionLayer.__HAL_STATE_STOPPED
        rospy.loginfo("HAL is stopped")

        # Is there anything we need to do here?  What about sending a command to the Psoc4 to powerdown the HB25's

        self._hal_state = HardwareAbstractionLayer.__HAL_STATE_SHUTDOWN

        rospy.loginfo("HAL is shutdown")

    def SetSpeed(self, left, right):
        if self._simulated:
            with self._lock:
                self._left_speed = left
                self._right_speed = right

            #rospy.loginfo("HAL.SetSpeed: {:6.2f}, {:6.2f}".format(left, right))
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
            #rospy.loginfo("HAL.GetCount: {:6.2f}, {:6.2f}".format(left, right))
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
            front = []
            back = []
        else:
            # Note: There are no left/right infrared sensors, but there are front and back
            front = self._left_psoc4.GetInfraredDistances()
            back = self._right_psoc4.GetInfraredDistances()
        return front + back

    def GetUltrasonic(self):
        if self._simulated:
            front = []
            back = []
        else:
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
            ranges = []
            intensities = []
            time_offset = 0
        else:
            ranges, intensities, time_offset = self._xv11.GetLaserScan()
        return ranges, intensities, time_offset

    def GetRpiVoltage(self):
        if self._simulated:
            voltage = 5.0
        else:
            voltage = self._powerpi.GetVoltage()
        return voltage

    def GetRpiCurrent(self):
        if self._simulated:
            voltage = 5.0
        else:
            voltage = self._powerpi.GetCurrent()
        return voltage

    def GetBattVoltage(self):
        if self._simulated:
            voltage = 12.0
        else:
            voltage = 12.0
        return voltage

    def GetBattCurrent(self):
        if self._simulated:
            current = 5.0
        else:
            current = 5.0
        return current

    def GetTemp(self):
        if self._simulated:
            imu_temp = {'f' : 0, 'c': 0}
            pp_temp = {'f' : 0, 'c': 0}
        else:
            imu_temp = self._imu.GetTemp()
            pp_temp = self._powerpi.GetTemp()
        return {"imu" : imu_temp, "pp" : pp_temp}

    def PowerOnBaseNode(self):
        if self._simulated:
            pass
        else:
            self._usb2go.SetOutput(self.__RASPBERRY_PI_POWER, True)

        # Note, some additional checking can be done on the PowerPi.  Maybe we should stay here until the PowerPi
        # sends a good status wrt to power and then return

    def PowerOffBaseNode(self):
        if self._simulated:
            pass
        else:
            self._usb2go.SetOutput(self.__RASPBERRY_PI_POWER, False)

        # Note, some additional functionality is provided via the PowerPi which would allow the Raspberry Pi to be
        # shutdown cleanly.

if __name__ == "__main__":
    hal = HardwareAbstractionLayer()

    hal.Startup()
    while not hal.Ready(): pass
    hal.SetSpeed({"left" : 1.0, "right" : 1.0})
    hal.Shutdown()
    while not hal.Ready(): pass