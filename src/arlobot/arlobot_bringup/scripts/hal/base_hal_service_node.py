#! /usr/bin/env python

import rospy
from arlobot_msgs.srv import *
from hal_service_node import HALServiceNode, HALServiceNodeError
from base_hal import BaseHardwareAbstractionLayer, BaseHardwareAbstractionLayerError


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
        self._services['BaseHALSetSpeed'] = rospy.Service('BaseHALSetSpeed', HALSetFloatArray, self._hal_set_speed)
        self._services['BaseHALGetOdometry'] = rospy.Service('BaseHALGetOdometry', HALGetFloatArray, self._hal_get_odometry)
        self._services['BaseHALGetInfrared'] = rospy.Service('BaseHALGetInfrared', HALGetFloatArray, self._hal_get_infrared)
        self._services['BaseHALGetUltrasonic'] = rospy.Service('BaseHALGetUltrasonic', HALGetFloatArray, self._hal_get_ultrasonic)
        self._services['BaseHALGetImu'] = rospy.Service('BaseHALGetImu', HALGetImu, self._hal_get_imu)
        self._services['BaseHALGetLaserScan'] = rospy.Service('BaseHALGetLaserScan', HALGetLaserScan, self._hal_get_laser_scan)
        self._services['BaseHALGetVoltage'] = rospy.Service('BaseHALGetVoltage', HALGetFloatArray, self._hal_get_voltage)
        self._services['BaseHALGetCurrent'] = rospy.Service('BaseHALGetCurrent', HALGetFloatArray, self._hal_get_current)
        self._services['BaseHALGetTemp'] = rospy.Service('BaseHALGetTemp', HALGetFloatArray, self._hal_get_temp)

    def _hal_set_speed(self, request):
        success = False
        left = request.values[0]
        right = request.values[1]
        try:
            self._hal.SetSpeed(left, right)
            success = True
        except BaseHardwareAbstractionLayerError:
            raise BaseHALServiceNodeError("Failure calling HAL::SetSpeed")
        return HALSetFloatArrayResponse(success=success)

    def _hal_get_odometry(self, request):
        values = []
        try:
            values = self._hal.GetOdometry()
        except BaseHardwareAbstractionLayerError:
            raise BaseHALServiceNodeError("Failure calling HAL:GetOdometry")
        return HALGetFloatArrayResponse(values = values)

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
        orientation = []
        linear_accel = []
        angular_velocity = []
        magnetic_field = []
        euler = []
        temperature = []
        
	try:
            results = self._hal.GetImuSensor()
            orientation = [ results['orientation'][k] for k in ['x', 'y', 'z', 'w'] ]
            linear_accel = [ results['linear_accel'][k] for k in ['x', 'y', 'z'] ]
            angular_velocity = [ results['angular_velocity'][k] for k in ['x', 'y', 'z'] ]
            magnetic_field = [ results['magnetic_field'][k] for k in ['x', 'y','z'] ]
            euler = [ results['euler'][k] for k in ['heading', 'yaw', 'roll', 'pitch'] ]
            temperature = [ results['temp'][k] for k in ['f', 'c'] ]
        except BaseHardwareAbstractionLayerError:
            raise BaseHALServiceNodeError("Failure calling HAL::GetImuSensor")
        return HALGetImuResponse(orientation=orientation, 
                                 linear_accel=linear_accel, 
                                 angular_velocity=angular_velocity, 
                                 magnetic_field=magnetic_field,
                                 euler=euler, 
                                 temperature=temperature)

    def _hal_get_voltage(self, request):
        values = []
        try:
            values.append(self._hal.GetVoltage())
        except BaseHardwareAbstractionLayerError:
            raise BaseHALServiceNodeError("Failure calling HAL::GetVoltage")
        return HALGetFloatArrayResponse(values=values)

    def _hal_get_current(self, request):
        values = []
        try:
            values.append(self._hal.GetCurrent())
        except BaseHardwareAbstractionLayerError:
            raise BaseHALServiceNodeError("Failure calling HAL::GetCurrent")
        return HALGetFloatArrayResponse(values=values)

    def _hal_get_temp(self, request):
        values = []
        try:
            values.append(self._hal.GetTemp())
        except BaseHardwareAbstractionLayerError:
            raise BaseHALServiceNodeError("Failure calling HAL::GetTemp")
        return HALGetFloatArrayResponse(values = values)


if __name__ ==  "__main__":
    hal_service = BaseHALServiceNode()
    hal_service.Start()
    hal_service.Run()
