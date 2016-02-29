from __future__ import print_function

import rospy
from ..hal_proxy import HALProxy, HALProxyError
from msgs.msg import RangeArray
from range_array_sensor import RangeArraySensor, RangeArraySensorError
from ..hal import HardwareAbstractionLayer, HardwareAbstractionLayerError


class InfraredRangeArraySensorError(Exception):
    pass


class InfraredRangeArraySensor(RangeArraySensor):
    __FIELD_OF_VIEW = 0.174533
    __MIN_RANGE = 0.10
    __MAX_RANGE = 0.80
    __FRAME_ID = "/infrared_array"

    def __init__(self):
        RangeArraySensor.__init__(self, self.__FIELD_OF_VIEW, self.__MIN_RANGE, self.__MAX_RANGE, RangeArray.INFRARED, self.__FRAME_ID)

        self._publisher = rospy.Publisher("infrared_distance_array", RangeArray, queue_size = 10)

        try:
            self._hal_proxy = HALProxy()
        except HALProxyError:
            raise InfraredRangeArraySensorError("Unable to create HAL")

    def Publish(self):
        # Read the infrared range values from i2c bus
        distances = self._hal_proxy.GetInfrared()
        self._publisher.publish(self._msg(distances))


if __name__ == "__main__":
    try:
        irs = InfraredRangeArraySensor()
    except InfraredRangeArraySensorError as err:
        print("Failed to instantiate UltrasonicRangeSensor")

    irs.Publish()