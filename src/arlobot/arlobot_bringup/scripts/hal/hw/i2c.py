from __future__ import print_function

# Note: There is not SMBus device on Ubuntu VirtualBox so we need to create a fake SMBus object
#from smbus import SMBus

class SMBus:
    def __init__(self, device):
        pass

    def read_byte_data(self, address, offset):
        return 0

    def write_byte_data(self, address, offset, value):
        pass

    def write_word_data(self, address, offset, value):
        pass

    def read_word_data(self, address, offset):
        return 0

    def read_i2c_block_data(self, address, offset, len):
        return 0


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
        #self._smbus.write_byte_data(address, offset, value)
        value = value

    def ReadUint8(self, address, offset):
        #value = self._smbus.read_byte_data(address, offset)
        value = 0
        return value

    def WriteUint16(self, address, offset, value):
        #self._smbus.write_word_data(address, offset, value)
        value = value

    def ReadUint16(self, address, offset):
        value = self._smbus.read_word_data(address, offset)
        value = 0
        return value

    def ReadUint32(self, address, offset):
        #value = self._smbus.read_i2c_block_data(address, offset, 4)
        value = 0
        #return (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | (value[3])
        return value

    def ReadArray(self, address, offset, size = 8):
        #values = self._smbus.read_i2c_block_data(address, offset, size)
        values = []
        return values


if __name__ == "__main__":

    try:
        i2c0 = I2CBus(I2CBus.DEV_I2C_0)
        assert(i2c0 != None)
    except I2CBusError as err:
        print("Failed to open I2C device 0")

    try:
        i2c1 = I2CBus(I2CBus.DEV_I2C_1)
        assert(i2c1 != None)
    except I2CBusError as err:
        print("Failed to open I2C device 0")

    i2c1.WriteUint16(0xff, 0)
    value = i2c1.ReadUint16(0)
    assert(value == 0xff)

    value = i2c1.ReadUint32(6)

    values = i2c1.ReadArray(12)
    assert(len(values) == 8)
