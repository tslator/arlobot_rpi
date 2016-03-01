#!/usr/bin/env python

import rospy
from hal import HardwareAbstractionLayer, HardwareAbstractionLayerError
from msgs.srv import *
from std_srvs.srv import Trigger, TriggerResponse


class HALServiceNodeError(Exception):
    pass


class HALServiceNode:
    """
    The HALServer is responsible for startup and shutdown of the HAL (HardwareAbstractionLayer).
    - Upon startup, the HALServer creates an instance of the HAL, invokes HAL startup, and waits for the HAL to report it
    is ready.
    - Once the HAL is ready, the HALServer publishes the state
    - Upon shutdown, the HALServer invokes HAL shutdown and waits for the HAL to report it has shutdown.
    - Once the HAL is shutdown, the HALServer publishes the state (although, the message is not guaranteed to be received
    due to shutdown being invoked)
    """

    __NAME = "hal_server_node"
    __DEFAULT_TIMEOUT = 5
    __DEFAULT_TIMEOUT_RATE = 1

    def __init__(self):
        '''
        Initialize the node
        Create an instance of HAL
        :return:
        '''
        enable_debug = rospy.get_param('debug', False)
        if enable_debug:
            rospy.init_node(HALServiceNode.__NAME, log_level=rospy.DEBUG)
        else:
            rospy.init_node(HALServiceNode.__NAME)
        rospy.on_shutdown(self.Shutdown)

        self._timeout_rate_value = rospy.get_param("hal_service_wait_rate", HALServiceNode.__DEFAULT_TIMEOUT_RATE)
        HALServiceNode.__DEFAULT_TIMEOUT = rospy.get_param("hal_service_timeout", HALServiceNode.__DEFAULT_TIMEOUT)
        simulated = rospy.get_param("hal_simulation", True)

        self._wait_rate = rospy.Rate(self._timeout_rate_value)
        self._services = {}

        try:
            self._hal = HardwareAbstractionLayer(simulated)
        except HardwareAbstractionLayerError:
            raise HALServiceNodeError


    def _wait(self, startup_shutdown, timeout):
        '''
        Call HAL.Ready and wait for response
        :param timeout:
        :return:
        '''
        ticks = self._timeout_rate_value * timeout
        while not self._hal.Ready(startup_shutdown) and ticks > 0:
            rospy.loginfo("Waiting for HAL ...")
            ticks -= 1
            self._wait_rate.sleep()
        else:
            if ticks <= 0:
                rospy.loginfo("Timeout waiting for HAL")
                return False
            else:
                return True

    def _service_status(self, request):
        '''
        This method is used to determine if the HAL service is available.  It is only a check of the service node and
        does not communicate with the HAL
        :param timeout:
        :return:
        '''
        return TriggerResponse(success=True, message="")

    def _startup(self, timeout=__DEFAULT_TIMEOUT):
        '''
        Call HAL.Startup and wait for response
        :param timeout:
        :return:
        '''
        rospy.loginfo("Starting up HAL ...")
        self._hal.Startup()
        success = self._wait(True, timeout)

        if (success):
            rospy.loginfo("Successfully started HAL")
        else:
            rospy.logfatal("Failed to start HAL")

        return success

    def _shutdown(self, timeout=__DEFAULT_TIMEOUT):
        rospy.loginfo("Shutting down HAL ...")
        self._hal.Shutdown()
        success = self._wait(False, timeout)
        message = ""

        if (success):
            rospy.loginfo("Successfully shutdown HAL")
        else:
            rospy.logwarn("Failed to shutdown HAL")

        return success

    def _hal_set_speed(self, request):
        success = False
        left = request.value_1
        right = request.value_2
        try:
            self._hal.SetSpeed(left, right)
            success = True
        except HardwareAbstractionLayerError:
            raise HALServiceNodeError("Failure calling HAL::SetSpeed")
        return HALSetTwoValuesResponse(success=success)

    def _hal_set_accel(self, request):
        success = False
        left = request.value_1
        right = request.value_2
        try:
            self._hal.SetAccel(left, right)
            success = True
        except HardwareAbstractionLayerError:
            raise HALServiceNodeError("Failure calling HAL::SetAccel")
        return HALSetTwoValuesResponse(success=success)

    def _hal_get_count(self, request):
        left = 0
        right = 0
        try:
            left, right = self._hal.GetCount()
        except HardwareAbstractionLayerError:
            raise HALServiceNodeError("Failure calling HAL::GetCount")
        return HALGetTwoValuesResponse(value_1 = left, value_2 = right)

    def _hal_get_speed(self, request):
        left = 0
        right = 0
        try:
            left, right = self._hal.GetSpeed()
        except HardwareAbstractionLayerError:
            raise HALServiceNodeError("Failure calling HAL::GetSpeed")
        return HALGetTwoValuesResponse(value_1 = left, value_2 = right)

    def _hal_get_infrared(self, request):
        values = []
        try:
            values = self._hal.GetInfrared()
        except HardwareAbstractionLayerError:
            raise HALServiceNodeError("Failure calling HAL:GetInfrared")
        return HALGetFloatArrayResponse(values = values)

    def _hal_get_ultrasonic(self, request):
        values = []
        try:
            values = self._hal.GetUltrasonic()
        except HardwareAbstractionLayerError:
            raise HALServiceNodeError("Failure calling HAL::GetUltrasonic")
        return HALGetFloatArrayResponse(values=values)

    def _hal_get_laser_scan(self, request):
        ranges = []
        intensities = []
        time_increment = 0
        try:
            ranges, intensities, time_increment = self._hal.GetLaserScan()
        except HardwareAbstractionLayerError:
            raise HALServiceNodeError("Failure calling HAL::GetLaserScan")
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
        except HardwareAbstractionLayerError:
            raise HALServiceNodeError("Failure calling HAL::GetImuSensor")
        return HALGetImuResponse(accel=accel, mag=mag, temp=temp)

    def _hal_get_rpi_voltage(self, request):
        rospy.loginfo("HALGetRpiVoltage ENTER")
        voltage = 0
        try:
            voltage = self._hal.GetRpiVoltage()
        except HardwareAbstractionLayerError:
            raise HALServiceNodeError("Failure calling HAL::GetRpiVoltage")
        rospy.loginfo("HALGetRpiVoltage EXIT")
        return HALGetOneValueResponse(value=voltage)

    def _hal_get_rpi_current(self, request):
        rospy.loginfo("HALGetRpiCurrent ENTER")
        current = 0
        try:
            current = self._hal.GetRpiCurrent()
        except HardwareAbstractionLayerError:
            raise HALServiceNodeError("Failure calling HAL::GetRpiCurrent")
        rospy.loginfo("HALGetRpiCurrent EXIT")
        return HALGetOneValueResponse(value=current)

    def _hal_get_batt_voltage(self, request):
        rospy.loginfo("HALGetBattVoltage ENTER")
        voltage = 0
        try:
            voltage = self._hal.GetBattVoltage()
        except HardwareAbstractionLayerError:
            raise HALServiceNodeError("Failure calling HAL::GetBattVoltage")
        rospy.loginfo("HALGetBattVoltage EXIT")
        return HALGetOneValueResponse(value=voltage)

    def _hal_get_batt_current(self, request):
        rospy.loginfo("HALGetBattCurent ENTER")
        current = 0
        try:
            current = self._hal.GetBattCurrent()
        except HardwareAbstractionLayerError:
            raise HALServiceNodeError("Failure calling HAL::GetBattCurrent")
        rospy.loginfo("HALGetBattCurrent EXIT")
        return HALGetOneValueResponse(value=current)

    def _hal_get_temp(self, request):
        rospy.loginfo("HALGetTemp ENTER")
        imu_temp = {}
        pp_temp = {}
        try:
            temp = self._hal.GetTemp()
            imu_temp = temp['imu']
            pp_temp = temp['pp']
        except HardwareAbstractionLayerError:
            raise HALServiceNodeError("Failure calling HAL::GetTemp")
        rospy.loginfo("HALGetTemp EXIT")
        return HALGetTwoValuesResponse(value_1=imu_temp['c'], value_2=pp_temp['c'])

    def Start(self):
        rospy.loginfo("HAL Service Node starting ...")
        if not self._startup():
            raise HALServiceNodeError("Failed to start HAL")
        rospy.loginfo("HAL Service node started")

    def Run(self):
        rospy.loginfo("HAL Service Node running")
        self._services['HALServiceStatus'] = rospy.Service('HALServiceStatus', Trigger, self._service_status)
        self._services['HALSetSpeed'] = rospy.Service('HALSetSpeed', HALSetTwoValues, self._hal_set_speed)
        self._services['HALSetAccel'] = rospy.Service('HALSetAccel', HALSetTwoValues, self._hal_set_accel)
        self._services['HALGetCount'] = rospy.Service('HALGetCount', HALGetTwoValues, self._hal_get_count)
        self._services['HALGetSpeed'] = rospy.Service('HALGetSpeed', HALGetTwoValues, self._hal_get_speed)
        self._services['HALGetInfrared'] = rospy.Service('HALGetInfrared', HALGetFloatArray, self._hal_get_infrared)
        self._services['HALGetUltrasonic'] = rospy.Service('HALGetUltrasonic', HALGetFloatArray, self._hal_get_ultrasonic)
        self._services['HALGetImu'] = rospy.Service('HALGetImu', HALGetImu, self._hal_get_imu)
        self._services['HALGetLaserScan'] = rospy.Service('HALGetLaserScan', HALGetLaserScan, self._hal_get_laser_scan)
        self._services['HALGetRpiVoltage'] = rospy.Service('HALGetRpiVoltage', HALGetOneValue, self._hal_get_rpi_voltage)
        self._services['HALGetRpiCurrent'] = rospy.Service('HALGetRpiCurrent', HALGetOneValue, self._hal_get_rpi_current)
        self._services['HALGetBattVoltage'] = rospy.Service('HALGetBattVoltage', HALGetOneValue, self._hal_get_batt_voltage)
        self._services['HALGetBattCurrent'] = rospy.Service('HALGetBattCurrent', HALGetOneValue, self._hal_get_batt_current)
        self._services['HALGetTemp'] = rospy.Service('HALGetTemp', HALGetTwoValues, self._hal_get_temp)

        rospy.spin()
        rospy.logwarn("HAL Service Node: shutdown invoked, exiting")

    def Shutdown(self):
        rospy.loginfo("HAL Service Node shutting down")

        for key in self._services.keys():
            self._services[key].shutdown()
        if not self._shutdown():
            raise HALServiceNodeError("Failed to shutdown HAL")
        rospy.loginfo("HAL Service Node is shutdown")


if __name__ ==  "__main__":
    hal_service = HALServiceNode()
    hal_service.Start()
    hal_service.Run()
