from __future__ import print_function

from smbus import SMBus

class I2CBusError(Exception):
    pass


class I2CBus:

    DEV_I2C_0 = 0
    DEV_I2C_1 = 1

    def __init__(self, device):
        self._device = device
        try:
            self._smbus = SMBus(self._device)
        except RuntimeError:
            raise I2CBusError("Unable to open SMBus")

    def WriteUint8(self, address, offset, value):
        self._smbus.write_byte_data(address, offset, value)

    def ReadUint8(self, address, offset):
        value = self._smbus.read_byte_data(address, offset)
        return value

    def WriteUint16(self, address, offset, value):
        self._smbus.write_word_data(address, offset, value)

    def ReadUint16(self, address, offset):
        value = self._smbus.read_word_data(address, offset)
        return value

    def ReadUint32(self, address, offset):
        value = self._smbus.read_i2c_block_data(address, offset, 4)
        return (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | (value[3])

    def ReadArray(self, address, offset, size = 8):
        values = self._smbus.read_i2c_block_data(address, offset, size)
        return values


if __name__ == "__main__":
    bus = SMBus(1)

    bus.write_block_data(0x08, 0, [0x01,0x01,0x17])
    
