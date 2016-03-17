#!/usr/bin/env python
from __future__ import print_function

import rospy
from hal import HardwareAbstractionLayer, HardwareAbstractionLayerError
from hw.m4atxhw import M4AtxHw, M4AtxHwError


class PCHardwareAbstractionLayerError(HardwareAbstractionLayerError):
    pass


class PCHardwareAbstractionLayer(HardwareAbstractionLayer):
    def __init__(self):
        HardwareAbstractionLayer.__init__(self, "PC HAL")
        try:
            self._m4atx = M4AtxHw()
        except M4AtxHwError:
            raise PCHardwareAbstractionLayerError("Unable to instantiate M4AtxHw")

    def _init(self):
        pass

    def _startup(self):
            self._m4atx.Start()

    def _stop(self):
        self._m4atx.Stop()

    def _shutdown(self):
        pass

    def GetVoltages(self):
        return self._m4atx.GetVoltages()

    def GetTemp(self):
        return self._m4atx.GetTemp()

if __name__ == "__main__":
    pc_hal = PCHardwareAbstractionLayer()

    pc_hal.Startup()
    while not pc_hal.Ready(): pass
    pc_hal.SetSpeed({"left" : 1.0, "right" : 1.0})
    pc_hal.Shutdown()
    while not pc_hal.Ready(): pass
