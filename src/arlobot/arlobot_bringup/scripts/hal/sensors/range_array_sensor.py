from __future__ import print_function

import rospy
from sensor_msgs.msg import Range


class RangeArraySensorError(Exception):
    pass


class RangeArraySensor:
    def __init__(self, fov, min, max, type):
        """

        :rtype: object
        """
        self._fov = fov
        self._min = min
        self._max = max
        self._type = type

    def Publish(self):
        '''
        Note: This function is implemented in the derived classes
        '''
        pass

    def _msg(self, frame_id, distance):
        msg = Range()
        msg.header.frame_id = frame_id
        msg.header.stamp = rospy.Time.now()
        msg.radiation_type = self._type
        msg.field_of_view = self._fov
        msg.min_range = self._min
        msg.max_range = self._max
        msg.range = min(max(distance, msg.min_range), msg.max_range)

        return msg

if __name__ == "__main__":
    try:
        rs = RangeArraySensor()
    except RangeArraySensorError as err:
        print("Failed to instantiate RangeSensor")

    print(rs._msg)

    rs.Publish()