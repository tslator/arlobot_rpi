#!/usr/bin/env python

import rospy
from ..hal_proxy import BaseHALProxy, BaseHALProxyError
from ..hal import HardwareAbstractionLayer, HardwareAbstractionLayerError
from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from arlobot_exception import ArlobotError


class ImuSensorError(ArlobotError):
    pass


class ImuSensor:
    def __init__(self):

        self._frame_id = "imu"

        self._publisher = rospy.Publisher("imu", Imu, queue_size = 10)

        try:
            self._hal_proxy = BaseHALProxy()
        except BaseHALProxyError:
            raise ImuSensorError("Unable to create HAL")

    def Publish(self):
        imu_data = self._hal_proxy.GetImu()

        imu_msg = Imu()
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = self._frame_id

        imu_msg.header = h
        imu_msg.orientation_covariance = (-1., )*9
        imu_msg.angular_velocity_covariance = (-1., )*9
        imu_msg.linear_acceleration_covariance = (-1., )*9

        imu_msg.orientation.x = imu_data["accel"]["x"]
        imu_msg.orientation.y = imu_data["accel"]["y"]
        imu_msg.orientation.z = imu_data["accel"]["z"]

        # Read the x, y, z and calculate the header
        x = imu_data["mag"]["x"]
        y = imu_data["mag"]["y"]
        z = imu_data["mag"]["z"]
        heading = 0.0

        imu_msg.orientation.w = heading

        self._publisher.publish(imu_msg)

if __name__ == "__main__":
    pass