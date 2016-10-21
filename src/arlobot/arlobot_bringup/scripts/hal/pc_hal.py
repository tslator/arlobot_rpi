#!/usr/bin/env python
from __future__ import print_function

import rospy
from hal_protocol import HALProtocol, HALProtocolError
from hal.hw.m4atxhw import M4AtxHw, M4AtxHwError
from hal.hw.kinectauxhw import KinectAuxHw, KinectAuxHwError


class PCHardwareAbstractionLayerError(HALProtocolError):
    pass


class PCHardwareAbstractionLayer(HALProtocol):

    def __init__(self):
        HALProtocol.__init__(self, "PC HAL")

        #try:
        #    self._m4atx = M4AtxHw()
        #except M4AtxHwError:
        #    raise PCHardwareAbstractionLayerError("Unable to instantiate M4AtxHw")

        #try:
        #    self._kinectaux = KinectAuxHw()
        #except KinectAuxHwError:
        #    raise PCHardwareAbstractionLayerError("Unable to instantiate KinectAuxHw")

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
        #return self._kinectaux.GetSpeed()
        return None

    def GetAccel(self):
        #return self._kinectaux.GetAccel()
        return None

    def GetTilt(self):
        #return self._kinectaux.GetTilt()
        return None

    def SetTilt(self, angle):
        #return self._kinectaux.SetTilt(angle)
        return None

    def SetLed(self, value):
        #return self._kinectaux.SetLed(value)
        return None

if __name__ == "__main__":
    pc_hal = PCHardwareAbstractionLayer()

    pc_hal.Startup()
    while not pc_hal.Ready(True): pass
    pc_hal.GetVoltages()
    pc_hal.Shutdown()
    while not pc_hal.Ready(False): pass
