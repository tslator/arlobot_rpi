from __future__ import print_function

from i2c import I2CBus, I2CBusError
import sys
import time

if __name__ == "__main__":
    try:
        i2c1 = I2CBus(I2CBus.DEV_I2C_1)
    except I2CBusError:
        print("Error instantiating I2CBus")

    print("speed: ", sys.argv[1])
    start = i2c1.ReadUint32(0x08, 8)
    i2c1.WriteUint16(0x08, 2, int(sys.argv[1]))
    start_time = time.time()
    result = int(i2c1.ReadUint16(0x08, 2))
    print("result: ", result)

    time.sleep(float(sys.argv[2]))

    i2c1.WriteUint16(0x08, 2, 0)
    end_time = time.time()
    end = i2c1.ReadUint32(0x08, 8)
    print("delta count: ", end - start, "delta time: ", end_time - start_time)
    

