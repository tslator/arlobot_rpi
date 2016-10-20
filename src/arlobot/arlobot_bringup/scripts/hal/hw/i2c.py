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

    def _read_multiple_bytes(self, address, offset, num_bytes):
        return self._smbus.read_i2c_block_data(address, offset, num_bytes)

    def _write_multiple_bytes(self, address, offset, byte_values):
        self._smbus.write_i2c_block_data(address, offset, list(byte_values))

    def WriteUint8(self, address, offset, value):
        self._smbus.write_byte_data(address, offset, value)

    def ReadUint8(self, address, offset):
        return self._smbus.read_byte_data(address, offset)

    def WriteUint16(self, address, offset, value):
        self._smbus.write_word_data(address, offset, value)
  
    def WriteInt16(self, address, offset, value):
        self._smbus.write_word_data(address, offset, value)

    def WriteUint32(self, address, offset, value):
        bytes = bytearray(struct.pack('L', value))
        self._write_multiple_bytes(address, offset, bytes)

    def WriteFloat(self, address, offset, value):
        bytes = bytearray(struct.pack('f',value))
        self._write_multiple_bytes(address, offset, bytes)

    def WriteArray(self, address, offset, values, type):
        # Convert each value to its byte representation and place into a bytearray before writing to the bus
        # Note: struct.pack returns a string representation of the value.  For a 1-byte value, it is a
        # string representation of 1 byte, for a 2 or 4 byte value, it is a 2-byte of 4-byte representation
        # Therefore, it is necessary to concatentate the strings before converting to a bytearray
        byte_values = ''
        for value in values:
            byte_values += struct.pack(type, value)
        self._write_multiple_bytes(address, offset, bytearray(byte_values))

    def ReadUint16(self, address, offset):
        return self._smbus.read_word_data(address, offset)

    def ReadInt16(self, address, offset):
        return self._smbus.read_word_data(address, offset)

    def ReadUint32(self, address, offset):
        bytes = self._read_multiple_bytes(address, offset, I2CBus.__TYPE_SIZES['L'])
        return struct.unpack('L', str(bytearray(bytes)))[0]

    def ReadInt32(self, address, offset):
        bytes = self._read_multiple_bytes(address, offset, I2CBus.__TYPE_SIZES['l'])
        return struct.unpack('l', str(bytearray(bytes)))[0]

    def ReadFloat(self, address, offset):
        values = self._read_multiple_bytes(address, offset, I2CBus.__TYPE_SIZES['f'])
        return struct.unpack('f', str(bytearray(values)))[0]

    def ReadArray(self, address, offset, num_values, type):
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
        bytes = self._read_multiple_bytes(address, offset, num_bytes)
        
        return list(struct.unpack(format, str(bytearray(bytes))))



def test(i2c, address):
    def dump_i2c_counters():
        print("rd1_busy: {}".format(i2c.ReadUint32(address, 8) ))
        print("busy    : {}".format(i2c.ReadUint32(address, 12) ))
        print("error   : {}".format(i2c.ReadUint32(address, 16) ))
        print("read1   : {}".format(i2c.ReadUint32(address, 20) ))
        print("wr1_busy: {}".format(i2c.ReadUint32(address, 24) ))
        print("write1  : {}".format(i2c.ReadUint32(address, 28) ))

    def test_read_write_bytes(indent=4):
        print("{}Write 8 bytes one at a time ...".format(' '*indent))
        for i in range(8):
            writes = i+128
            i2c.WriteUint8(address, i, writes)
            reads = i2c.ReadUint8(address, i)
            assert(reads == writes)

        print("{}Write 8 bytes as an array ...".format(' '*indent))
        writes = [i for i in range(128, 128+8)]
        i2c.WriteArray(address, 0, writes, 'B')
        reads = i2c.ReadArray(address, 0, 8, 'B')
        assert(reads == writes)

    def test_read_write_words(indent=4):
        print("{}Write 4 words one at a time ...".format(' '*indent))
        for i in range(4):
            writes = i+32768
            offset = i * 2
            i2c.WriteUint16(address, offset, writes)
            reads = i2c.ReadUint16(address, offset)
            assert(reads == writes)

        print("{}Write 4 words as an array ...".format(' '*indent))
        writes = [i for i in range(32768, 32768+4)]
        i2c.WriteArray(address, 0, writes, 'H')
        reads = i2c.ReadArray(address, 0, 4, 'H')
        assert(reads == writes)

    def test_read_write_longs(indent=4):
        print("{}Write 2 longs one at a time ...".format(' '*indent))
        for i in range(2):
            writes = i+2147483648
            offset = i * 4
            i2c.WriteUint32(address, offset, writes)
            reads = i2c.ReadUint32(address, offset)
            assert(reads == writes)

        print("{}Write 2 longs as an array ...".format(' '*indent))
        writes = [i for i in range(2147483648, 2147483648+2)]
        i2c.WriteArray(address, 0, writes, 'L')
        reads = i2c.ReadArray(address, 0, 2, 'L')
        assert(reads == writes)

    def test_read_write_floats(indent=4):
        print("{}Write 2 floats one at a time ...".format(' '*indent))
        for i in range(2):
            writes = i+3.14
            offset = i * 4
            i2c.WriteFloat(address, offset, writes)
            reads = i2c.ReadFloat(address, offset)
            assert( "{:.2f}".format(reads) == "{:.2f}".format(writes) )

        print("{}Write 2 floats as an array ...".format(' '*indent))
        writes = [i*3.14 for i in range(1,3)]
        i2c.WriteArray(address, 0, writes, 'f')
        reads = i2c.ReadArray(address, 0, 2, 'f')
        for i in range(0,2):
            assert( "{:.2f}".format(reads[i]) == "{:.2f}".format(writes[i])  ) 

    print("Start Test")
    print("Dump i2c counters ...")
    dump_i2c_counters()

    print("Test Case 1: read write bytes ...")
    test_read_write_bytes()

    print("Test Case 2: read write words ...")
    test_read_write_words()

    print("Test Case 3: read write longs ...")
    test_read_write_longs()
    
    print("Test Case 4: read write floats ...")
    test_read_write_floats()
   
    print("Dumb i2c counters ...")
    dump_i2c_counters() 
  
    print("End Test")

if __name__ == "__main__":
    # Note: This test requires a compatible Psoc application that exposes an I2C device supporting the following mapping:
    #
    #                       Offset   Byte    Word    Long    Float
    # Read/Write Section      0       1       1       1        1 
    #                         1       2       1       1        1
    #                         2       3       2       1        1
    #                         3       4       2       1        1
    #                         4       5       3       2        2
    #                         5       6       3       2        2
    #                         6       7       4       2        2
    #                         7       8       4       2        2
    #  Read/Only Section      
    #      rd1_busy counter   8                       1                 
    #                         9                       1
    #                        10                       1
    #                        11                       1
    #          busy counter  12                       2
    #                        13                       2
    #                        14                       2
    #                        15                       2
    #         error counter  16                       3
    #                        17                       3
    #                        18                       3
    #                        19                       3
    #         read1 counter  20                       4
    #                        21                       4
    #                        22                       4
    #                        23                       4
    #      wr1_busy counter  24                       5
    #                        25                       5
    #                        26                       5
    #                        27                       5
    #        write1 counter  28                       6
    #                        29                       6
    #                        30                       6
    #                        31                       6
    #
    # The read-only section of the i2c interface exports counters of i2c conditions as seen by the Psoc
    #    rd1_busy/wr1_busy - are incremented when there is an attempt to read/write to address 1 and the I2C component is busy
    #    read1/write1 - are incremented when there is a read/write to address 1
    #    busy - incremented if the master is in communication with the slave
    #    error - incremented when an i2c hardware error is detecte
    #
    # Note: the above counters are incremented by a routine called from the main loop of the Psoc and will not contain accurate
    # count values.  the intent here is to provide values to read from the i2c slave that are independent of the i2c master
    # also, it is expected that there will be no errors, so getting a non-zero count for error could indicate a hardware/connection
    # problem.
    #
    # The freesoc project has a define TEST_I2C (enabled in config.h) which supports this mapping
    #------------------------------------------------------------------------------------------------------------------------------

    import sys
    
    try:
        i2c1 = I2CBus(I2CBus.DEV_I2C_1)
        assert(i2c1 != None)
        print("Successfully opened I2C /dev/i2c1")
    except I2CBusError as err:
        print("Failed to open I2C device 0")

    address = int(sys.argv[1])
    assert(address == 0x08)
 
    print("Execute single pass ...")
    test(i2c1, address)

    print("Execute stress ...")
    for i in range(10):
        test(i2c1, address)

    print("Validation Complete")

