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

    def __init__(self, i2cbus, avr_addr, ina219_addr):
        self._avr_address = avr_addr
        self._ina219_address = ina219_addr

        self._i2c_bus = i2cbus

    def GetVoltage(self):
        reg_value = self._i2c_bus.ReadUint16(self._ina219_address, self.__BUS_REG)
        return float((reg_value & 0xFFF8) >> 1)

    def GetCurrent(self):
        reg_value = self._i2c_bus.ReadUint16(self._ina219_address, self.__SHUNT_REG)
        return float(reg_value/10)

    def GetTemp(self):
        return {'f' : 1.0, 'c' : 1.0}
