from __future__ import print_function

import rospy
from sensor_msgs.msg import Range
from range_array_sensor import RangeArraySensor, RangeArraySensorError
from ..hal_protocol.base_hal_proxy import BaseHALProxy, BaseHALProxyError


class InfraredRangeArraySensorError(Exception):
    pass


class InfraredRangeArraySensor(RangeArraySensor):
    __FIELD_OF_VIEW = 0.174533
    __MIN_RANGE = 0.10
    __MAX_RANGE = 0.80
    __FRONT_FRAME_ID = "/infrared_array_front"
    __BACK_FRAME_ID = "/infrared_array_back"

    def __init__(self):
        fov = rospy.get_param("Infrared Field Of View")
        min_range = rospy.get_param("Infrared Min Range")
        max_range = rospy.get_param("Infrared Max Range")
        RangeArraySensor.__init__(self, fov, min_range, max_range, Range.INFRARED)

        self._num_front_sensors = rospy.get_param("Num Infrared Front Sensors")
        self._num_back_sensors = rospy.get_param("Num Infrared Back Sensors")

        self._front_publisher = rospy.Publisher("infrared_array_front", Range, queue_size=self._num_front_sensors)
        self._back_publisher = rospy.Publisher("infrared_array_back", Range, queue_size=self._num_back_sensors)

        try:
            self._hal_proxy = BaseHALProxy()
        except BaseHALProxyError:
            raise InfraredRangeArraySensorError("Unable to create HAL")

    def Publish(self):
        distances = self._hal_proxy.GetInfrared()

        for index in range(self._num_front_sensors):
            self._front_publisher.publish(self._msg("{}_{}".format(self.__FRONT_FRAME_ID, index), distances[index]))
        for index in range(self._num_back_sensors):
            self._back_publisher.publish(self._msg("{}_{}".format(self.__BACK_FRAME_ID, index), distances[index + self._num_front_sensors]))


if __name__ == "__main__":
    try:
        irs = InfraredRangeArraySensor()
    except InfraredRangeArraySensorError as err:
        print("Failed to instantiate UltrasonicRangeSensor")

    irs.Publish()