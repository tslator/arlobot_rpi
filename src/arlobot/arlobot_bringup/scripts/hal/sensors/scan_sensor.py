#!/usr/bin/env python

from rospy import Publisher, Time, get_time
from ..hal_proxy import BaseHALProxy, BaseHALProxyError
from sensor_msgs.msg import LaserScan
from math import pi


class ScanSensorError(Exception):
    pass


class ScanSensor:
    __MIN_ANGLE = 0.0
    __MAX_ANGLE = 2.0*pi
    __ANGLE_INCREMENT = 2.0*pi/360

    def __init__(self, pub_name):

        self._publisher = Publisher(pub_name, LaserScan, queue_size=10)

        self._last_time = Time.now()

        try:
            self._hal_proxy = BaseHALProxy()
        except BaseHALProxyError:
            raise ScanSensorError("Unable to create HAL")

    def _get_data(self):
        """
        This method is overridden in the subclasses and returns a tuple of ranges (distances), intensities, and delta time
        """
        return [], [], 0

    def _msg(self, ranges, intensities, scan_time):
        new_time = Time.now()
        delta_time = new_time - self._last_time
        self._last_time = new_time

        msg = LaserScan()
        msg.header.frame_id = self._frame_id
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

    def Publish(self):
        ranges, intensities, scan_time  = self._get_data()
        self._publisher.publish(self._msg(ranges, intensities, scan_time))


class LaserScanSensor(ScanSensor):
    __MIN_RANGE = 0.06
    __MAX_RANGE = 5.0
    __FRAME_ID = "neato_laser"

    def __init__(self, name):
        ScanSensor.__init__(self, "laser", name+"_laser")

    def _get_data(self):
        return self._hal_proxy.GetLaserScan()


class UltrasonicScanSensor(ScanSensor):
    __MIN_RANGE = 0.02 # meters
    __MAX_RANGE = 5.0  # meters
    __FRAME_ID = "ultrasonic_array"
    _OFFSETS = [60, 30, 0, 330, 300, 45, 0, 315, 240, 210, 180, 150, 120, 225, 180, 135]

    def __init__(self):
        ScanSensor.__init__(self, "ultrasonic_scan")

        self._last_scan_time = Time.now()

    def _get_data(self):
        new_time = Time.now()
        delta_time = new_time - self._last_scan_time
        self._last_scan_time = new_time

        values = self._hal_proxy.GetUltrasonic()

        ranges = [self.__MAX_RANGE]*360
        for ii in range(len(values)):
            offset = UltrasonicScanSensor._OFFSETS[ii]
            # Note: There are duplicates in the offsets, e.g., sensors at the same angle but different height.  I think
            # we want to take the smallest value.
            ranges[offset] = min(values[ii], ranges[offset])

        return ranges, [0]*360, delta_time


class InfraredScanSensor(ScanSensor):
    __MIN_RANGE = 0.10
    __MAX_RANGE = 0.80
    __FRAME_ID = "infrared_array"
    __OFFSETS = [330, 30, 45, 15, 345, 315, 330, 30, 150, 210, 225, 195, 165, 135, 150, 210]

    def __init__(self):
        ScanSensor.__init__(self, "infrared_scan")

        self._last_scan_time = Time.now()

    def _get_data(self):
        new_time = Time.now()
        delta_time = new_time - self._last_scan_time
        self._last_scan_time = new_time

        values = self._hal_proxy.GetInfrared()

        ranges = [self.__MAX_RANGE]*360
        for ii in range(len(values)):
            offset = InfraredScanSensor.__OFFSETS[ii]
            # Note: There are duplicates in the offsets, e.g., sensors at the same angle but different height.  I think
            # we want to take the smallest value.
            ranges[offset] = min(values[ii], ranges[offset])

        return ranges, [0]*360, delta_time


if __name__ == "__main__":
    import sys
    lss = LaserScanSensor()
    iterations = sys.argv[1]
    for i in range(iterations):
        lss.Publish()
