from __future__ import print_function

from i2c import I2CBus, I2CBusError
import sys
import time

if __name__ == "__main__":
    try:
        i2c1 = I2CBus(I2CBus.DEV_I2C_1)
    except I2CBusError:
        print("Error instantiating I2CBus")

    address = int(sys.argv[1])
    speed = int(sys.argv[2])
    run_time = int(sys.argv[3])

    print("speed: ", speed)
    start = i2c1.ReadUint32(address, 8)
    i2c1.WriteUint16(address, 2, speed)
    start_time = time.time()

    time.sleep(float(sys.argv[3]))

    velocity = i2c1.ReadUint16(address, 12)
    end = i2c1.ReadUint32(address, 8)
    i2c1.WriteUint16(address, 2, 0)
    end_time = time.time()
    print("delta count: ", end - start, " distance(m): ", ((end - start) / 144) * 0.4787, " velocity(m/s): ", velocity)
    

