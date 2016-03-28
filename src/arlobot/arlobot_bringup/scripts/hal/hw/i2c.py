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

    def _try(self, call, tries=10):
        assert tries > 0
        error = None
        result = None

        while tries:
            try:
                result = call()
            except IOError as e:
                error = e
                tries -= 1
            else:
                break

        if not tries:
            print("failure after {} tries".format(20 - tries))
            raise error

        return result    

    def WriteUint8(self, address, offset, value):
        self._try(lambda : self._smbus.write_byte_data(address, offset, value))

    def ReadUint8(self, address, offset):
        value = self._try(lambda : self._smbus.read_byte_data(address, offset))
        return value

    def WriteUint16(self, address, offset, value):
        self._try(lambda : self._smbus.write_word_data(address, offset, value))

    def ReadUint16(self, address, offset):
        value = self._try(lambda : self._smbus.read_word_data(address, offset))
        return value

    def WriteInt16(self, address, offset, value):
        self._try(lambda : self._smbus.write_word_data(address, offset, value))

    def ReadInt16(self, address, offset):
        value = self._try(lambda : self._smbus.read_word_data(address, offset))
        return value if value < (2**15 - 1) else value - 2**16

    def ReadUint32(self, address, offset):
        value = self._try(lambda : self._smbus.read_i2c_block_data(address, offset, 4))
        return (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | (value[3])

    def ReadInt32(self, address, offset):
        value = self._try(lambda : self._smbus.read_i2c_block_data(address, offset, 4))

        int_value = (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | (value[3])
        return int_value if int_value < (2**31 - 1) else int_value - 2**32

    def ReadArray(self, address, offset, size = 8):
        #values = self._try(lambda : self._smbus.read_i2c_block_data(address, offset, size))
        #return values
        return [0]*8


if __name__ == "__main__":
    import sys
    '''
    Note: I2C device 0 is not supported on Raspberry Pi rev 2
    try:
        i2c0 = I2CBus(I2CBus.DEV_I2C_0)
        assert(i2c0 != None)
    except I2CBusError as err:
        print("Failed to open I2C device 0")
    '''
    try:
        i2c1 = I2CBus(I2CBus.DEV_I2C_1)
        assert(i2c1 != None)
    except I2CBusError as err:
        print("Failed to open I2C device 0")

    i2c1.WriteUint16(int(sys.argv[1]), 0x00, 0xff)
    value = i2c1.ReadUint16(int(sys.argv[1]), 0)
    print("ReadUint16 returned: ", value)
    assert(value == 0xff)

    i2c1.WriteUint16(int(sys.argv[1]), 0x00, 0x7f)
    value = i2c1.ReadUint16(int(sys.argv[1]), 0)
    print("ReadUint16 returned: ", value)
    assert(value == 0x7f)

    value = i2c1.ReadUint32(int(sys.argv[1]), 6)
    print("ReadUint32 returned:", value)
    values = i2c1.ReadArray(int(sys.argv[1]), 12)
    assert(len(values) == 8)
    print("ReadArray returned:", values)
