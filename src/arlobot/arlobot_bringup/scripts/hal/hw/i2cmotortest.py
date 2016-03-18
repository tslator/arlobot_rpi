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
    start_count = i2c1.ReadInt32(address, 8)
    i2c1.WriteInt16(address, 2, speed)
    start_time = time.time()

    time.sleep(float(sys.argv[3]))

    velocity = i2c1.ReadInt16(address, 12)
    i2c1.WriteInt16(address, 2, 0)
    end_count = i2c1.ReadInt32(address, 8)
    end_time = time.time()
    delta_count = abs(start_count - end_count)
    print("start: ", start_count, " end: ", end_count, " delta count: ", delta_count, " distance(m): ", (float(delta_count) / 144) * 0.4787, " velocity(m/s): ", velocity)
    

