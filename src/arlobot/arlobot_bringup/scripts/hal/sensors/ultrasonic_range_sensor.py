from __future__ import print_function

import rospy
from ..hal_proxy import HALProxy, HALProxyError
from sensor_msgs.msg import Range
from range_array_sensor import RangeArraySensor, RangeArraySensorError
from ..hal import HardwareAbstractionLayer, HardwareAbstractionLayerError


class UltrasonicRangeArraySensorError(Exception):
    pass


class UltrasonicRangeArraySensor(RangeArraySensor):
    __FIELD_OF_VIEW = 0.523599
    __MIN_RANGE = 0.02 # meters
    __MAX_RANGE = 5.0  # meters
    __FRONT_FRAME_ID = "ultrasonic_array_front"
    __BACK_FRAME_ID = "ultrasonic_array_back"

    def __init__(self):
        RangeArraySensor.__init__(self, self.__FIELD_OF_VIEW, self.__MIN_RANGE, self.__MAX_RANGE, Range.ULTRASOUND)

        self._num_front_sensors = rospy.get_param("Num Ultrasonic Front Sensors")
        self._num_back_sensors = rospy.get_param("Num Ultrasonic Back Sensors")

        self._front_publisher = rospy.Publisher("ultrasonic_array_front", Range, queue_size=16)
        self._back_publisher = rospy.Publisher("ultrasonic_array_back", Range, queue_size=16)

        try:
            self._hal_proxy = HALProxy()
        except HALProxyError:
            raise UltrasonicRangeArraySensorError("Unable to create HAL")

    def Publish(self):
        distances = self._hal_proxy.GetUltrasonic()

        for index in range(self._num_front_sensors):
            self._front_publisher.publish(self._msg("{}_{}".format(self.__FRONT_FRAME_ID, index), distances[index]))
        for index in range(0, self._num_back_sensors):
            self._back_publisher.publish(self._msg("{}_{}".format(self.__BACK_FRAME_ID, index), distances[index + self._num_front_sensors]))


if __name__ == "__main__":
    try:
        urs = UltrasonicRangeArraySensor()
    except UltrasonicRangeArraySensorError as err:
        print("Failed to instantiate UltrasonicRangeSensor")

    urs.Publish()