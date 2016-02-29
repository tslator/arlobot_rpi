#! /usr/bin/env python


import rospy

from msgs.srv import HALTriggerWithTimeout, HALSetTwoValues, HALGetFloatArray, HALGetLaserScan, \
                     HALGetImu, HALGetTwoValues, HALGetOneValue
from std_srvs.srv import Trigger


class HALProxyError(Exception):
    pass


class HALProxy:

    def __init__(self, use_stub=True):
        # Create a connection to the HAL Service

        # Note: The HAL service is a collection of services which facilitate access to the hardware resources
        # Instead of calling wait_for_service on every service message, is it possible to call wait_for_service
        # on a single service message, specifically, the one service message that guarantees that the HAL has been
        # started, initialized and is ready for operation.  Then, it shouldn't be necessary to call wait_for_service
        # on the balance of the API, yes?
        rospy.wait_for_service('HALServiceStatus')
        try:
            hal_service_status = rospy.ServiceProxy('HALServiceStatus', Trigger)
            response = hal_service_status()
            if not response.success:
                raise HALProxyError("Failed response from HALServiceStatus")
        except rospy.ServiceException, e:
            raise HALProxyError(e)

        # Note: We explicitly do not call wait_for_service below because once the HAL is determined to be ready via
        # wait_for_service('HALStatus'), all of the other services will also be ready

    def SetSpeed(self, speeds):
        set_speed = rospy.ServiceProxy('HALSetSpeed', HALSetTwoValues)
        left_speed = speeds['left']
        right_speed = speeds['right']
        response = set_speed(left_speed, right_speed)
        return response.success

    def SetAccel(self, accels):
        set_accel = rospy.ServiceProxy('HALSetAccel', HALSetTwoValues)
        left_accel = accels['left']
        right_accel = accels['right']
        response = set_accel(left_accel, right_accel)
        return response.success

    def GetCount(self):
        get_count = rospy.ServiceProxy('HALGetCount', HALGetTwoValues)
        response = get_count()
        left_count = response.value_1
        right_count = response.value_2
        return {'left' : left_count, 'right' : right_count}

    def GetSpeed(self):
        get_speed = rospy.ServiceProxy('HALGetSpeed', HALGetTwoValues)
        response = get_speed()
        left_speed = response.value_1
        right_speed = response.value_2
        return {'left' : left_speed, 'right' : right_speed}

    def GetUltrasonic(self):
        get_ultrasonic = rospy.ServiceProxy('HALGetUltrasonic', HALGetFloatArray)
        response = get_ultrasonic()
        return list(response.values)

    def GetInfrared(self):
        get_infrared = rospy.ServiceProxy('HALGetInfrared', HALGetFloatArray)
        response = get_infrared()
        return list(response.values)

    def GetLaserScan(self):
        get_laser_scan = rospy.ServiceProxy('HALGetLaserScan', HALGetLaserScan)
        response = get_laser_scan()
        return list(response.ranges), list(response.intensities), response.time_increment

    def GetImu(self):
        get_imu = rospy.ServiceProxy('HALGetImu', HALGetImu)
        response = get_imu()
        return { 'accel' : {'x' : response.accel[0], 'y' : response.accel[1], 'z' : response.accel[2]},
                 'mag' : {'x' : response.mag[0], 'y' : response.mag[1], 'z' : response.mag[2]},
                 'temp' : {'f' : response.temp[0], 'c' : response.temp[1]}}

    def GetRpiVoltage(self):
        get_rpi_current = rospy.ServiceProxy('HALGetRpiVoltage', HALGetOneValue)
        response = get_rpi_current()
        return response.value

    def GetRpiCurrent(self):
        get_rpi_current = rospy.ServiceProxy('HALGetRpiCurrent', HALGetOneValue)
        response = get_rpi_current()
        return response.value

    def GetBattVoltage(self):
        get_batt_voltage = rospy.ServiceProxy('HALGetBattVoltage', HALGetOneValue)
        response = get_batt_voltage()
        return response.value

    def GetBattCurrent(self):
        get_batt_current = rospy.ServiceProxy('HALGetBattCurrent', HALGetOneValue)
        response = get_batt_current()
        return response.value

    def GetTemp(self):
        get_temp = rospy.ServiceProxy('HALGetTemp', HALGetTwoValues)
        response = get_temp()
        return {"imu" : response.value_1, "pp" : response.value_2}

    def PowerOnBaseNode(self):
        power_on_base_node = rospy.ServiceProxy('HALPowerOnBaseNode', Trigger)
        response = power_on_base_node()
        return response.success

        # Note, some additional checking can be done on the PowerPi.  Maybe we should stay here until the PowerPi
        # sends a good status wrt to power and then return
            
    def PowerOffBaseNode(self):
        power_off_base_node = rospy.ServiceProxy('HALPowerOffBaseNode', Trigger)
        response = power_off_base_node()
        return response.success

        # Note, some additional checking can be done on the PowerPi.  Maybe we should stay here until the PowerPi
        # sends a good status wrt to power and then return
