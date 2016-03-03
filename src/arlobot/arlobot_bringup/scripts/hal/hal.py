#!/usr/bin/env python
from __future__ import print_function

import time
import math
import random
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

    def Startup(self):
        '''
        Calls to startup are ignore if the HAL is already running.
        The HAL start life in the INITIAL state and can perform additional initialization outside of the class __init__
        before transitioning to the STOPPED state
        Once in the STOPPED state, the HAL can perform any work necessary prior to transitioning to the RUNNING state, e.g.,
        in simulated mode, start the left/right wheel threads
        Once the HAL transitions to the RUNNING state is can be used by clients
        :return:
        '''
        rospy.loginfo("HAL starting up ...")

        if self._hal_state is HardwareAbstractionLayer.__HAL_STATE_RUNNING:
            rospy.logwarn("Attempt to startup HAL while it is running - No action taken")
            return

        # Handle case when state is INITIAL - the shared object state needs to be setup
        if self._hal_state is HardwareAbstractionLayer.__HAL_STATE_INITIAL:

            # For doing any initialization that is subject to startup.  Can't think of anything right now.

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
        '''
        Checks if the HAL has completed the associated startup or shutdown request
        :param startup_shutdown: indicates whether the check is for startup or shutdown
        :return:
        '''
        if startup_shutdown:
            return self._hal_state == HardwareAbstractionLayer.__HAL_STATE_RUNNING
        else:
            return self._hal_state == HardwareAbstractionLayer.__HAL_STATE_SHUTDOWN

    def Shutdown(self):
        '''
        Allows the HAL to be shutdown.  There is presently no scenario envisioned where the HAL needs to be shutdown
        while the robot continues to operate.  Generally speaking, the HAL's life cycle is tied to the robot's.  However,
        for completeness it is here and it may serve a purpose at some point.  In the meantime, it functions to ensure
        the hardware is kept in a consistent state while debugging.
        :return:
        '''
        rospy.loginfo("HAL shutting down ...")

        if self._simulated:
            self._left_worker.stop()
            self._right_worker.stop()

        self._hal_state = HardwareAbstractionLayer.__HAL_STATE_STOPPED
        rospy.loginfo("HAL is stopped")

        # Before shutting down, turn off the motor controllers
        self._left_psoc4.SetControl(self._left_psoc4.MOTOR_CONTROLLER_OFF)
        self._right_psoc4.SetControl(self._right_psoc4.MOTOR_CONTROLLER_OFF)

        self._hal_state = HardwareAbstractionLayer.__HAL_STATE_SHUTDOWN

        rospy.loginfo("HAL is shutdown")

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


if __name__ == "__main__":
    hal = HardwareAbstractionLayer()

    hal.Startup()
    while not hal.Ready(): pass
    hal.SetSpeed({"left" : 1.0, "right" : 1.0})
    hal.Shutdown()
    while not hal.Ready(): pass