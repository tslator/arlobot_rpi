#!/usr/bin/env python
from __future__ import print_function

import rospy
from hal import HardwareAbstractionLayer, HardwareAbstractionLayerError
from hw.m4atxhw import M4AtxHw, M4AtxHwError
from hw.kinectauxhw import KinectAuxHw, KinectAuxHwError


class PCHardwareAbstractionLayerError(HardwareAbstractionLayerError):
    pass


class PCHardwareAbstractionLayer(HardwareAbstractionLayer):

    def __init__(self):
        HardwareAbstractionLayer.__init__(self, "PC HAL")

        #try:
        #    self._m4atx = M4AtxHw()
        #except M4AtxHwError:
        #    raise PCHardwareAbstractionLayerError("Unable to instantiate M4AtxHw")

        try:
            self._kinectaux = KinectAuxHw()
        except KinectAuxHwError:
            raise PCHardwareAbstractionLayerError("Unable to instantiate KinectAuxHw")

    def _init(self):
        pass

    def _startup(self):
        #self._m4atx.Start()
        pass

    def _stop(self):
        #self._m4atx.Stop()
        pass

    def _shutdown(self):
        pass

    def GetVoltages(self):
        #return self._m4atx.GetVoltages()
        return []

    def GetTemp(self):
        #return self._m4atx.GetTemp()
        return 0

    def GetSpeed(self):
        return self._kinectaux.GetSpeed()

    def GetAccel(self):
        return self._kinectaux.GetAccel()

    def GetTilt(self):
        return self._kinectaux.GetTilt()

    def SetTilt(self, angle):
        return self._kinectaux.SetTilt(angle)

    def SetLed(self, value):
        return self._kinectaux.SetLed(value)

if __name__ == "__main__":
    pc_hal = PCHardwareAbstractionLayer()

    pc_hal.Startup()
    while not pc_hal.Ready(True): pass
    pc_hal.GetVoltages()
    pc_hal.Shutdown()
    while not pc_hal.Ready(False): pass
