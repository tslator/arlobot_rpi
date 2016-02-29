from __future__ import print_function

import rospy
from ..hal_proxy import HALProxy, HALProxyError
from msgs.msg import RangeArray
from range_array_sensor import RangeArraySensor, RangeArraySensorError
from ..hal import HardwareAbstractionLayer, HardwareAbstractionLayerError


class UltrasonicRangeArraySensorError(Exception):
    pass


class UltrasonicRangeArraySensor(RangeArraySensor):
    __FIELD_OF_VIEW = 0.523599
    __MIN_RANGE = 0.02 # meters
    __MAX_RANGE = 5.0  # meters
    __FRAME_ID = "/ultrasound_array"

    def __init__(self):
        RangeArraySensor.__init__(self, self.__FIELD_OF_VIEW, self.__MIN_RANGE, self.__MAX_RANGE, RangeArray.ULTRASOUND, self.__FRAME_ID)

        self._publisher = rospy.Publisher("ultrasonic_distance_array", RangeArray, queue_size = 10)

        try:
            self._hal_proxy = HALProxy()
        except HALProxyError:
            raise UltrasonicRangeArraySensorError("Unable to create HAL")

    def Publish(self):
        distances = self._hal_proxy.GetUltrasonic()
        self._publisher.publish(self._msg(distances))


if __name__ == "__main__":
    try:
        urs = UltrasonicRangeArraySensor()
    except UltrasonicRangeArraySensorError as err:
        print("Failed to instantiate UltrasonicRangeSensor")

    urs.Publish()