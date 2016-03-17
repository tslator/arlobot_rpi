#! /usr/bin/env python
from __future__ import print_function


class PowerPiHwError(Exception):
    pass


class PowerPiHw:

    __CONFIG_REG = 0
    __SHUNT_REG = 1
    __BUS_REG = 2
    __POWER_REG = 3
    __CURRENT_REG = 4
    __CALIBRATION_REG = 5

    AVR_ADDR = 0x21
    INC219_ADDR = 0x40

    def __init__(self, i2cbus, avr_addr=AVR_ADDR, ina219_addr=INC219_ADDR):
        self._i2c_bus = i2cbus
        self._avr_address = avr_addr
        self._ina219_address = ina219_addr

    def GetVoltage(self):
        reg_value = self._i2c_bus.ReadUint16(self._ina219_address, self.__BUS_REG)
        print("voltage value: ", reg_value)
        return float((reg_value & 0xFFF8) >> 1)

    def GetCurrent(self):
        reg_value = self._i2c_bus.ReadUint16(self._ina219_address, self.__SHUNT_REG)
        print("current value: ", reg_value)
        return float(reg_value/10)


if __name__ == "__main__":
    import i2c
    i2c1 = i2c.I2CBus(1)

    pph = PowerPiHw(i2c1, 0x21, 0x40)
    voltage = pph.GetVoltage()    
    current = pph.GetCurrent()

    print(voltage, current)
