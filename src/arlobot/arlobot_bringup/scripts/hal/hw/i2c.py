from __future__ import print_function

from smbus import SMBus
import struct


class I2CBusError(Exception):
    pass


class I2CBus:

    DEV_I2C_0 = 0
    DEV_I2C_1 = 1

    # The following formatting is taken from struct
    __TYPE_SIZES = {'d': 8, # double - 8 bytes
                    'f': 4, # float - 4 bytes
                    'L': 4, # uint32 - 4 bytes
                    'l': 4, # int32 - 4 bytes
                    'H': 2, # uint16 - 2 bytes
                    'h': 2, # int16 - 2 bytes
                    'B': 1, # uint8 - 1 byte
                    'b': 1  # int8 - 1 byte
                   }

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

    def _read_multiple_bytes(self, address, offset, num_bytes, use_i2c):
        values = []
        if use_i2c:
            values = self._try(lambda: self._smbus.read_i2c_block_data(address, offset, num_bytes))
        else:
            for i in range(num_bytes):
                value = self._try(lambda: self._smbus.read_byte_data(address, offset + i))
                values.append(value)
        return values

    def _write_multiple_bytes(self, address, offset, bytes, use_i2c):
        if use_i2c:
            self._try(lambda: self._smbus.write_i2c_block_data(address, offset, bytes))
        else:
            for i in range(len(bytes)):
                self._try(lambda: self._smbus.write_byte_data(address, offset + i, bytes[i]))

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

    def ReadUint32(self, address, offset, use_i2c=False):
        values = self._read_multiple_bytes(address, offset, I2CBus.__TYPE_SIZES['L'], use_i2c)
        #return (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | (value[3])
        return struct.unpack('L', str(bytearray(values)))[0]

    def ReadInt32(self, address, offset, use_i2c=False):
        values = self._read_multiple_bytes(address, offset, I2CBus.__TYPE_SIZES['l'], use_i2c)
        #int_value = (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | (value[3])
        #return int_value if int_value < (2**31 - 1) else int_value - 2**32
        return struct.unpack('l', str(bytearray(values)))[0]

    def ReadFloat(self, address, offset, use_i2c=False):
        values = self._read_multiple_bytes(address, offset, I2CBus.__TYPE_SIZES['f'], use_i2c)
        return struct.unpack('f', str(bytearray(values)))[0]

    def WriteFloat(self, address, offset, value, use_i2c=False):
        bytes = bytearray(struct.pack('f',value))
        self._write_multiple_bytes(address, offset, bytes, use_i2c)

    def ReadArray(self, address, offset, num_values, type, use_i2c=False):
        # Create a format specifier based on the number of values requested.  
        # All of the values will be read as the same type, e.g., all floats, all long, etc
        # The format specifies the number of float values to convert
        format = '%s%s' % (num_values, type)
        # Calculate number of bytes to read
        #   - num_values is the number of values to read
        #   - num_bytes is num_values * size of each value
        num_bytes = num_values * I2CBus.__TYPE_SIZES[type]
        # It turns out that reading i2c block data is not supported on all Raspberry Pi's (probably a OS/driver difference)
        # The Pi 2 running Jessie doesn't support i2c (i2cget with no arguments shows no 'i' option)
        # The Pi 3 running Jessie does support i2c (i2cget with no argument shows 'i' option)
        # So, we need to support both options
        values = self._read_multiple_bytes(address, offset, num_bytes, use_i2c)
        return list(struct.unpack(format, str(bytearray(values))))

if __name__ == "__main__":
    # Note: This test requires a compatible Psoc application that exposes an I2C device that can support the following:
    #
    #   Write/Read Uint8    (1 byte at offset 0)
    #   Write/Read Uint16   (2 bytes at offset 2)
    #   Write/Read Int16    (2 bytes at offset 4)
    #   Write/Read Uint32   (4 bytes at offset 6)
    #   Write/Read Int32    (4 bytes at offset 10)
    #   Write/Read Float    (4 bytes at offset 14)
    #   Write/Read Array of
    #       Uint8 - 4 bytes at offset 0
    #       Uint16 - 4 words at offset 0
    #       Int16 - 4 words at offset 0
    #       Uint32 - 4 longs at offset 0
    #       Int32 - 4 longs at offset 0
    #       Float - 4 floats at offset 0
    #
    #  Add a test project to the arlobot_freesoc workspace that will implement this i2c device
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
        print("Successfully opened I2C /dev/i2c1")
    except I2CBusError as err:
        print("Failed to open I2C device 0")

    address = int(sys.argv[1])
    assert(address == 0x08)

    print("Validate WriteUint8")
    i2c1.WriteUint8(address, 0, 0xff)
    value = i2c1.ReadUint8(address, 0)
    assert(value == 0xff)
    print("Passed")

    print("Validate WriteUint6")
    i2c1.WriteUint16(address, 2, 2000)
    value = i2c1.ReadUint16(address, 2)
    assert(value == 2000)
    print("Passed")
 
    print("Validate WriteInt16")
    i2c1.WriteInt16(address, 4, -2000)
    value = i2c1.ReadInt16(address, 4)
    assert(value == -2000)
    print("Passed")

    print("Validate ReadUint32")
    value = i2c1.ReadUint32(address, 6)
    assert(value == 100000)
    print("Passed")

    print("Validate ReadInt32")
    value = i2c1.ReadInt32(address, 10)
    assert(value == -100000)
    print("Passed")

    print("Validate ReadFloat")
    value = i2c1.ReadFloat(address, 14)
    assert(round(value, 2) == 3.14)
    print("Passed")

    print("Validate ReadArray: 4 uint8")
    values = i2c1.ReadArray(address, 0, 4, 'B')
    assert(values == [255, 45, 208, 7])
    print("Passed")

    print("Validate ReadArray: 4 uint16")
    values = i2c1.ReadArray(address, 0, 4, 'H')
    assert(values == [11775, 2000, 63536, 34464])
    print("Passed")
    
    print("Validate ReadArray: 4 int16")
    values = i2c1.ReadArray(address, 0, 4, 'h')
    assert(values == [11775, 2000, -2000, -31072])
    print("Passed")

    print("Validate ReadArray: 4 uint32")
    values = i2c1.ReadArray(address, 0, 4, 'L')
    assert(values == [131083775, 2258696240L, 2036334593, 4123262974L])
    print("Passed")

    print("Validate ReadArray: 4 int32")
    values = i2c1.ReadArray(address, 0, 4, 'l')
    assert(values == [131083775, -2036271056, 2036334593, -171704322])
    print("Passed")

    print("Validate ReadArray: 4 float")
    values = i2c1.ReadArray(address, 0, 4, 'f')
    assert(values == [3.132339567047409e-34, -6.05499895173445e-35, 7.269216097124774e+34, -4.969189579182135e+32])
    print("Passed")

    print("Validation Complete")

