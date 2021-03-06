#!/usr/bin/env python

from rospy import Publisher, Time, get_time, get_param
from ..base_hal_proxy import BaseHALProxy, BaseHALProxyError
from sensor_msgs.msg import LaserScan
from math import pi


class ScanSensorError(Exception):
    pass


class ScanSensor:
    __MIN_ANGLE = 0.0
    __MAX_ANGLE = 2.0*pi
    __ANGLE_INCREMENT = 2.0*pi/360.0

    def __init__(self, pub_name, frame_id, min_range, max_range):

        self._publisher = Publisher(pub_name, LaserScan, queue_size=10)

        self._frame_id = frame_id
        self._min_range = min_range
        self._max_range = max_range
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
        msg.time_increment = 0.1 #delta_time.secs
        # Note: scan_time is the time between scans of the laser, i.e., the time it takes to read 360 degrees.
        msg.scan_time = 0.1 # scan_time
        msg.range_min = float(self._min_range)
        msg.range_max = float(self._max_range)
        msg.ranges = [min(max(range, msg.range_min), msg.range_max) for range in ranges]
        msg.intensities = intensities

        return msg

    def Publish(self):
        ranges, intensities, scan_time  = self._get_data()
        self._publisher.publish(self._msg(ranges, intensities, scan_time))


class LaserScanSensor(ScanSensor):

    def __init__(self, name):
        min_range = get_param("Laser Min Range")
        max_range = get_param("Laser Max Range")
        ScanSensor.__init__(self, "laser", name+"_laser", min_range, max_range)

    def _get_data(self):
        return self._hal_proxy.GetLaserScan()


class UltrasonicScanSensor(ScanSensor):
    # Note: The values in offsets are degrees and represent the physical position of the sensor
    _OFFSETS = [60, 30, 0, 330, 300, 45, 0, 315, 240, 210, 180, 150, 120, 225, 180, 135]

    def __init__(self):
        min_range = get_param("Ultrasonic Min Range")
        max_range = get_param("Ultrasonic Max Range")
        ScanSensor.__init__(self,
                            "ultrasonic_scan",
                            "ultrasonic_array",
                            min_range,
                            max_range)

        self._last_scan_time = Time.now()
        self._max_range = max_range

    def _get_data(self):
        new_time = Time.now()
        scan_time = new_time - self._last_scan_time
        self._last_scan_time = new_time

        values = self._hal_proxy.GetUltrasonic()

        ranges = [self._max_range]*360
        for ii in range(len(values)):
            offset = UltrasonicScanSensor._OFFSETS[ii]
            # Note: There are duplicates in the offsets, e.g., sensors at the same angle but different height.  I think
            # we want to take the smallest value.
            ranges[offset] = min(values[ii], ranges[offset])

        # Note: scan_time is the time between scans of the laser, i.e., the time it takes to read 360 degrees.  For the
        # xv11 laser, scan_time comes from the driver.  For the purposes of simulating a laser scan, we'll just use the
        # time between calls to publish
        return ranges, [0]*360, scan_time.secs


class InfraredScanSensor(ScanSensor):
    # Note: The values in offsets are degrees and represent the physical position of the sensor
    __OFFSETS = [330, 30, 45, 15, 345, 315, 330, 30, 150, 210, 225, 195, 165, 135, 150, 210]

    def __init__(self):
        min_range = get_param("Infrared Min Range")
        max_range = get_param("Infrared Max Range")
        ScanSensor.__init__(self,
                            "infrared_scan",
                            "infrared_array",
                            min_range,
                            max_range)

        self._last_scan_time = Time.now()
        self._max_range = max_range

    def _get_data(self):
        new_time = Time.now()
        scan_time = new_time - self._last_scan_time
        self._last_scan_time = new_time

        values = self._hal_proxy.GetInfrared()

        ranges = [self._max_range]*360
        for ii in range(len(values)):
            offset = InfraredScanSensor.__OFFSETS[ii]
            # Note: There are duplicates in the offsets, e.g., sensors at the same angle but different height.  I think
            # we want to take the smallest value.
            ranges[offset] = min(values[ii], ranges[offset])

        # Note: scan_time is the time between scans of the laser, i.e., the time it takes to read 360 degrees.  For the
        # xv11 laser, scan_time comes from the driver.  For the purposes of simulating a laser scan, we'll just use the
        # time between calls to publish
        return ranges, [0]*360, scan_time


if __name__ == "__main__":
    import sys
    lss = LaserScanSensor()
    iterations = sys.argv[1]
    for i in range(iterations):
        lss.Publish()
