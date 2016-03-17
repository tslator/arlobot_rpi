#! /usr/bin/env python

import rospy
from base_hal import BaseHardwareAbstractionLayer, BaseHardwareAbstractionLayerError
from hal_service_node import HALServiceNode, HALServiceNodeError
from msgs.srv import *


class BaseHALServiceNodeError(HALServiceNodeError):
    pass


class BaseHALServiceNode(HALServiceNode):
    """
    The HALServer is responsible for startup and shutdown of the HAL (HardwareAbstractionLayer).
    - Upon startup, the HALServer creates an instance of the HAL, invokes HAL startup, and waits for the HAL to report it
    is ready.
    - Once the HAL is ready, the HALServer publishes the state
    - Upon shutdown, the HALServer invokes HAL shutdown and waits for the HAL to report it has shutdown.
    - Once the HAL is shutdown, the HALServer publishes the state (although, the message is not guaranteed to be received
    due to shutdown being invoked)
    """

    NAME = "base_hal_server_node"

    def __init__(self):
        '''
        Initialize the node
        Create an instance of HAL
        :return:
        '''
        HALServiceNode.__init__(self, BaseHALServiceNode.NAME, "BaseHAL")

        self._timeout_rate = rospy.get_param("Base HAL Service Timeout Rate")
        self._timeout = rospy.get_param("Base HAL Service Timeout")
        simulated = rospy.get_param("Base HAL Simulate")

        self._wait_rate = rospy.Rate(self._timeout_rate)
        self._services = {}

        try:
            self._hal = BaseHardwareAbstractionLayer(simulated)
        except BaseHardwareAbstractionLayerError:
            raise BaseHALServiceNodeError

    def _run(self):
        self._services['BaseHALSetSpeed'] = rospy.Service('BaseHALSetSpeed', HALSetTwoValues, self._hal_set_speed)
        self._services['BaseHALSetAccel'] = rospy.Service('BaseHALSetAccel', HALSetTwoValues, self._hal_set_accel)
        self._services['BaseHALGetCount'] = rospy.Service('BaseHALGetCount', HALGetTwoValues, self._hal_get_count)
        self._services['BaseHALGetSpeed'] = rospy.Service('BaseHALGetSpeed', HALGetTwoValues, self._hal_get_speed)
        self._services['BaseHALGetInfrared'] = rospy.Service('BaseHALGetInfrared', HALGetFloatArray, self._hal_get_infrared)
        self._services['BaseHALGetUltrasonic'] = rospy.Service('BaseHALGetUltrasonic', HALGetFloatArray, self._hal_get_ultrasonic)
        self._services['BaseHALGetImu'] = rospy.Service('BaseHALGetImu', HALGetImu, self._hal_get_imu)
        self._services['BaseHALGetLaserScan'] = rospy.Service('BaseHALGetLaserScan', HALGetLaserScan, self._hal_get_laser_scan)
        self._services['BaseHALGetVoltage'] = rospy.Service('BaseHALGetVoltage', HALGetOneValue, self._hal_get_voltage)
        self._services['BaseHALGetCurrent'] = rospy.Service('BaseHALGetCurrent', HALGetOneValue, self._hal_get_current)
        self._services['BaseHALGetTemp'] = rospy.Service('BaseHALGetTemp', HALGetOneValue, self._hal_get_temp)

    def _hal_set_speed(self, request):
        success = False
        left = request.value_1
        right = request.value_2
        try:
            self._hal.SetSpeed(left, right)
            success = True
        except BaseHardwareAbstractionLayerError:
            raise BaseHALServiceNodeError("Failure calling HAL::SetSpeed")
        return HALSetTwoValuesResponse(success=success)

    def _hal_set_accel(self, request):
        success = False
        left = request.value_1
        right = request.value_2
        try:
            self._hal.SetAccel(left, right)
            success = True
        except BaseHardwareAbstractionLayerError:
            raise BaseHALServiceNodeError("Failure calling HAL::SetAccel")
        return HALSetTwoValuesResponse(success=success)

    def _hal_get_count(self, request):
        left = 0
        right = 0
        try:
            left, right = self._hal.GetCount()
        except BaseHardwareAbstractionLayerError:
            raise BaseHALServiceNodeError("Failure calling HAL::GetCount")
        return HALGetTwoValuesResponse(value_1 = left, value_2 = right)

    def _hal_get_speed(self, request):
        left = 0
        right = 0
        try:
            left, right = self._hal.GetSpeed()
        except BaseHardwareAbstractionLayerError:
            raise BaseHALServiceNodeError("Failure calling HAL::GetSpeed")
        return HALGetTwoValuesResponse(value_1 = left, value_2 = right)

    def _hal_get_infrared(self, request):
        values = []
        try:
            values = self._hal.GetInfrared()
        except BaseHardwareAbstractionLayerError:
            raise BaseHALServiceNodeError("Failure calling HAL:GetInfrared")
        return HALGetFloatArrayResponse(values = values)

    def _hal_get_ultrasonic(self, request):
        values = []
        try:
            values = self._hal.GetUltrasonic()
        except BaseHardwareAbstractionLayerError:
            raise BaseHALServiceNodeError("Failure calling HAL::GetUltrasonic")
        return HALGetFloatArrayResponse(values=values)

    def _hal_get_laser_scan(self, request):
        ranges = []
        intensities = []
        time_increment = 0
        try:
            ranges, intensities, time_increment = self._hal.GetLaserScan()
        except BaseHardwareAbstractionLayerError:
            raise BaseHALServiceNodeError("Failure calling HAL::GetLaserScan")
        return HALGetLaserScanResponse(ranges=ranges, intensities=intensities, time_increment=time_increment)

    def _hal_get_imu(self, request):
        accel = []
        mag = []
        temp = []
        try:
            results = self._hal.GetImuSensor()
            accel = [ results['accel']['x'], results['accel']['y'], results['accel']['z'] ]
            mag = [ results['mag']['x'], results['mag']['y'], results['mag']['z'] ]
            temp = [ results['temp']['f'], results['temp']['c'] ]
        except BaseHardwareAbstractionLayerError:
            raise BaseHALServiceNodeError("Failure calling HAL::GetImuSensor")
        return HALGetImuResponse(accel=accel, mag=mag, temp=temp)

    def _hal_get_voltage(self, request):
        rospy.loginfo("HALGetVoltage ENTER")
        voltage = 0
        try:
            voltage = self._hal.GetVoltage()
        except BaseHardwareAbstractionLayerError:
            raise BaseHALServiceNodeError("Failure calling HAL::GetVoltage")
        rospy.loginfo("HALGetVoltage EXIT")
        return HALGetOneValueResponse(value=voltage)

    def _hal_get_current(self, request):
        rospy.loginfo("HALGetCurrent ENTER")
        current = 0
        try:
            current = self._hal.GetCurrent()
        except BaseHardwareAbstractionLayerError:
            raise BaseHALServiceNodeError("Failure calling HAL::GetCurrent")
        rospy.loginfo("HALGetCurrent EXIT")
        return HALGetOneValueResponse(value=current)

    def _hal_get_temp(self, request):
        rospy.loginfo("HALGetTemp ENTER")
        temp = 0
        try:
            temp = self._hal.GetTemp()
        except BaseHardwareAbstractionLayerError:
            raise BaseHALServiceNodeError("Failure calling HAL::GetTemp")
        rospy.loginfo("HALGetTemp EXIT")
        return HALGetOneValueResponse(value = temp)


if __name__ ==  "__main__":
    hal_service = BaseHALServiceNode()
    hal_service.Start()
    hal_service.Run()
