#!/usr/bin/env python

from rospy import Publisher, Time, get_time
from ..hal_proxy import HALProxy, HALProxyError
from ..hal import HardwareAbstractionLayer, HardwareAbstractionLayerError
from sensor_msgs.msg import LaserScan
from math import pi


class LaserScanSensorError(HardwareAbstractionLayerError):
    pass


class LaserScanSensor:
    __MIN_ANGLE = 0.0
    __MAX_ANGLE = 2.0*pi
    __ANGLE_INCREMENT = 2.0*pi/360
    __MIN_RANGE = 0.06
    __MAX_RANGE = 5.0
    __FRAME_ID = "neato_laser"

    def __init__(self):
        self._publisher = Publisher("scan", LaserScan, queue_size=10)

        self._last_time = Time.now()

        try:
            self._hal_proxy = HALProxy()
        except HALProxyError:
            raise LaserScanSensorError("Unable to create HAL")

    def Publish(self):
        ranges, intensities, scan_time  = self._hal_proxy.GetLaserScan()
        self._publisher.publish(self._msg(ranges, intensities, scan_time, Time.now()))

    def _msg(self, ranges, intensities, scan_time):
        new_time = Time.now()
        delta_time = new_time - self._last_time
        self._last_time = new_time

        msg = LaserScan()
        msg.header.frame_id = self.__FRAME_ID
        msg.header.stamp = Time.now()
        msg.angle_max = self.__MAX_ANGLE
        msg.angle_min = self.__MIN_ANGLE
        msg.angle_increment = self.__ANGLE_INCREMENT
        # Note: time_increment is the time between measurements, i.e. how often we read the scan and publish it (in
        # seconds)
        msg.time_increment = delta_time
        # Note: scan_time is the time between scans of the laser, i.e., the time it takes to read 360 degrees.  This
        # comes from the XV11 itself (in seconds)
        msg.scan_time = scan_time
        msg.range_min = self.__MIN_RANGE
        msg.range_max = self.__MAX_RANGE
        msg.ranges = [min(max(range, msg.range_min), msg.range_max) for range in ranges]
        msg.intensities = intensities

        return msg

if __name__ == "__main__":
    pass