#! /usr/bin/env python

from __future__ import print_function

import rospy


from hal_service_node import HALServiceNode, HALServiceNodeError
from pc_hal import PCHardwareAbstractionLayer, PCHardwareAbstractionLayerError
from arlobot_msgs.srv import *



class PCHALServiceNodeError(HALServiceNodeError):
    pass


class PCHALServiceNode(HALServiceNode):
    NAME = "pc_hal_server_node"

    __DEFAULT_TIMEOUT = 5

    def __init__(self):
        '''
        Initialize the node
        Create an instance of HAL
        :return:
        '''
        HALServiceNode.__init__(self, PCHALServiceNode.NAME, "PCHAL")

        self._timeout_rate = rospy.get_param("PC HAL Service Timeout Rate")
        self._timeout = rospy.get_param("PC HAL Service Timeout")

        try:
            self._hal = PCHardwareAbstractionLayer()
        except PCHardwareAbstractionLayerError:
            raise PCHALServiceNodeError

        self._wait_rate = rospy.Rate(self._timeout_rate)

    def _run(self):
        self._services['PCHALGetVoltage'] = rospy.Service('PCHALGetVoltage', HALGetFloatArray, self._hal_get_voltages)
        self._services['PCHALGetTemp'] = rospy.Service('PCHALGetTemp', HALGetFloatArray, self._hal_get_temp)
        self._services['PCHALGetSpeed'] = rospy.Service('PCHALGetSpeed', HALGetFloatArray, self._hal_get_speed)
        self._services['PCHALGetAccel'] = rospy.Service('PCHALGetAccel', HALGetFloatArray, self._hal_get_accel)
        self._services['PCHALGetTilt'] = rospy.Service('PCHALGetTilt', HALGetFloatArray, self._hal_get_tilt)
        self._services['PCHALSetTilt'] = rospy.Service('PCHALSetTilt', HALSetFloatArray, self._hal_set_tilt)
        self._services['PCHALSetLed'] = rospy.Service('PCHALSetLed', HALSetFloatArray, self._hal_set_led)

    def _hal_get_voltages(self, request):
        values = []
        try:
            values = self._hal.GetVoltages()
        except PCHardwareAbstractionLayerError:
            raise PCHALServiceNodeError("Failure calling HAL::GetUltrasonic")
        return HALGetFloatArrayResponse(values=values)

    def _hal_get_temp(self, request):
        rospy.loginfo("PCHALGetVoltage ENTER")
        values = []
        try:
            temp = self._hal.GetVoltage()
            values.append(temp['c'])
        except PCHardwareAbstractionLayerError:
            raise PCHALServiceNodeError("Failure calling HAL::GetVoltage")
        rospy.loginfo("PCHALGetVoltage EXIT")
        return HALGetFloatArrayResponse(values=values)

    def _hal_get_speed(self, request):
        rospy.loginfo("PCHALGetSpeed ENTER")
        values = []
        try:
            left, right = self._hal.GetSpeed()
            values = [left, right]
        except PCHardwareAbstractionLayerError:
            raise PCHALServiceNodeError("Failure calling HAL::GetSpeed")
        return HALGetFloatArrayResponse(values = values)

    def _hal_get_accel(self, request):
        rospy.loginfo("PCHALGetAccel ENTER")
        values = []
        try:
            accel = self._hal.GetAccel()
            values = [accel['x'], accel['y'], accel['z']]
        except PCHardwareAbstractionLayerError:
            raise PCHALServiceNodeError("Failure calling HAL::GetAccel")
        return HALGetFloatArrayResponse(values = values)

    def _hal_get_tilt(self, request):
        rospy.loginfo("PCHALGetTilt ENTER")
        values = []
        try:
            tilt, status = self._hal.GetTilt()
            values = [tilt, status]
        except PCHardwareAbstractionLayerError:
            raise PCHALServiceNodeError("Failure calling HAL::GetTilt")
        rospy.logionfo("PCHALGetTilt EXIT")
        return HALGetFloatArrayResponse(values = values)

    def _hal_set_tilt(self, request):
        rospy.loginfo("PCHALSetTilt ENTER")
        success = False
        try:
            self._hal.SetTilt(request.values[0])
            success = True
        except PCHardwareAbstractionLayerError:
            raise PCHALServiceNodeError("Failure calling HAL::SetTilt")
        rospy.loginfo("PCHALSetTilt EXIT")
        return HALSetFloatArrayResponse(success=success)

    def _hal_set_led(self, request):
        rospy.loginfo("PCHALSetLed ENTER")
        success = False
        try:
            self._hal.SetLed(request.values[0])
            success = True
        except PCHardwareAbstractionLayerError:
            raise PCHALServiceNodeError("Failure calling HAL:SetLed")
        rospy.loginfo("PCHALSetLed EXIT")
        return HALSetFloatArrayResponse(success=success)


if __name__ == "__main__":
    hal_service = PCHALServiceNode()
    hal_service.Start()
    hal_service.Run()
