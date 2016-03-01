#!/usr/bin/env python

import rospy

from arlobot_exception import ArlobotError

from hal.sensors.ultrasonic_range_sensor import UltrasonicRangeArraySensor, UltrasonicRangeArraySensorError
from hal.sensors.infrared_range_sensor import InfraredRangeArraySensor, InfraredRangeArraySensorError
from hal.sensors.laser_sensor import LaserScanSensor, LaserScanSensorError
from hal.sensors.imu_sensor import ImuSensor, ImuSensorError
from hal.sensors.power_sensor import PowerSensor, PowerSensorError


class ArlobotSensorsNodeError(Exception):
    pass


class ArlobotSensorsNode:
    '''
    This node is responsible for reading all sensor information coming from the Arlobot base.  The primary source of
    sensor information will be the Psoc4 processor via I2C to the Raspberry Pi, but there could be sensors attached
    directly to the Raspberry Pi or some other processor or bus.

    The purpose of this node is to have central place where all sensors in the system can be managed.  I'm not sure if
    that is practical or even a good approach
    '''
    def __init__(self):
        # Initialize the node
        enable_debug = rospy.get_param('debug', False)
        if enable_debug:
            rospy.init_node('arlobot_sensors_node', log_level=rospy.DEBUG)
        else:
            rospy.init_node('arlobot_sensors_node')
        rospy.on_shutdown(self.Shutdown)

        loop_rate = rospy.get_param("Sensors Node Loop Rate")
        self._loop_rate = rospy.Rate(loop_rate)

        # Initialization
        try:
            self._urs = UltrasonicRangeArraySensor()
        except UltrasonicRangeArraySensorError:
            raise ArlobotSensorsNodeError("Unable to create Ultrasonic Range Sensor")

        try:
            self._irs = InfraredRangeArraySensor()
        except InfraredRangeArraySensorError:
            raise ArlobotSensorsNodeError("Unable to create Infrared Range Sensor")

        try:
            self._lss = LaserScanSensor()
        except LaserScanSensorError:
            raise ArlobotSensorsNodeError("Unable to create Laser Scan Sensor")

        try:
            self._imu = ImuSensor()
        except ImuSensorError:
            raise ArlobotSensorsNodeError("Unable to create IMU Sensor")

        try:
            self._pwr = PowerSensor()
        except PowerSensorError:
            raise ArlobotSensorsNodeError("Unable to create Power Sensor")

    def Start(self):
        rospy.loginfo("Arlobot Sensors Node has started")

    def Loop(self):
        while not rospy.is_shutdown():
            # ROS provides the sensor_msgs/Range.msg which supports ultrasonic and infrared sensors

            # Publish the ultrasonic distance sensor data
            self._urs.Publish()

            # Publish the infrared distance sensor data
            self._irs.Publish()

            # Publish the laser scan data
            self._lss.Publish()

            # Publish the IMU data
            self._imu.Publish()

            # Publish the Power data
            self._pwr.Publish()

            self._loop_rate.sleep()


        else:
            rospy.logwarn("Arlobot Sensors Node: shutdown invoked, exiting")

    def Shutdown(self):
        rospy.loginfo("Arlobot Sensors Node has shutdown")
        pass


if __name__ == "__main__":
    arlobotsensorsnode = ArlobotSensorsNode()
    arlobotsensorsnode.Start()
    arlobotsensorsnode.Loop()
