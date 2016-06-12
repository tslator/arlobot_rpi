from __future__ import print_function

from smbus import SMBus
import struct


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
        values = self._try(lambda : self._smbus.read_i2c_block_data(address, offset, 4))
        #return (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | (value[3])
        return struct.unpack('L', str(bytearray(values)))

    def ReadInt32(self, address, offset):
        values = self._try(lambda : self._smbus.read_i2c_block_data(address, offset, 4))
        #int_value = (value[0] << 24) | (value[1] << 16) | (value[2] << 8) | (value[3])
        #return int_value if int_value < (2**31 - 1) else int_value - 2**32
        return struct.unpack('l', str(bytearray(values)))

    def ReadFloat(self, address, offset):
        values = self._try(lambda : self._smbus.read_i2c_block_data(address, offset, 4))
        float_value = struct.unpack('f', str(bytearray(values)))
        return float_value

    def ReadArray(self, address, offset, num_values, type):
        type_sizes = {'d' : 8, 'f' : 4, 'L' : 4, 'l' : 4, 'H' : 2, 'h' : 2, 'B' : 1, 'b' : 1 }

        # Create a format specifier based on the number of values requested.  
        # All of the values will be read as the same type, e.g., all floats, all long, etc
        # The format specifies the number of float values to convert
        format = '%s%s' % (num_values, type)
        # Calculate number of bytes to read
        #   - num_values is the number of values to read
        #   - num_bytes is num_values * size of each value
        num_bytes = num_values * type_sizes[type]
        values = self._try(lambda: self._smbus.read_i2c_block_data(address, offset, num_bytes))
        return struct.unpack(format, str(bytearray(values)))

if __name__ == "__main__":
    import sys
    import time
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



    i2c1.WriteInt16(int(sys.argv[1]), 2, 100)
    i2c1.WriteInt16(int(sys.argv[1]), 4, 110)
    time.sleep(2)
    left_speed = i2c1.ReadInt16(int(sys.argv[1]), 10)
    right_speed = i2c1.ReadInt16(int(sys.argv[1]), 12)
    print("ReadInt16 (left) returned: ", left_speed)
    print("ReadInt16 (right) returned: ", right_speed)

    x_dist = i2c1.ReadFloat(int(sys.argv[1]), 14)
    y_dist = i2c1.ReadFloat(int(sys.argv[1]), 18)
    heading = i2c1.ReadFloat(int(sys.argv[1]), 22)
    linear = i2c1.ReadFloat(int(sys.argv[1]), 26)
    angular = i2c1.ReadFloat(int(sys.argv[1]), 30)
    print("x dist: ", x_dist)
    print("y dist: ", y_dist)
    print("heading: ", heading)
    print("linear: ", linear)
    print("angular: ", angular)

    print("Reading 5 float values (20 bytes)")
    float_values = i2c1.ReadArray(int(sys.argv[1]), 14, 20, 'f')
    print(float_values)


