#! /usr/bin/env python

from __future__ import print_function

import rospy


from hal_service_node import HALServiceNode, HALServiceNodeError
from pc_hal import PCHardwareAbstractionLayer, PCHardwareAbstractionLayerError
from msgs.srv import *



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
        self._services['PCHALGetTemp'] = rospy.Service('PCHALGetTemp', HALGetOneValue, self._hal_get_temp)

    def _hal_get_voltages(self, request):
        values = []
        try:
            values = self._hal.GetVoltages()
        except PCHardwareAbstractionLayerError:
            raise PCHALServiceNodeError("Failure calling HAL::GetUltrasonic")
        return HALGetFloatArrayResponse(values=values)

    def _hal_get_temp(self, request):
        rospy.loginfo("PCHALGetVoltage ENTER")
        voltage = 0
        try:
            voltage = self._hal.GetVoltage()
        except PCHardwareAbstractionLayerError:
            raise PCHALServiceNodeError("Failure calling HAL::GetVoltage")
        rospy.loginfo("PCHALGetVoltage EXIT")
        return HALGetOneValueResponse(value=voltage)


if __name__ == "__main__":
    hal_service = PCHALServiceNode()
    hal_service.Start()
    hal_service.Run()
