from __future__ import print_function

import sys
import time
import logging
import threading
import random

sys.path.append("../")
from i2c import I2CBus, I2CBusError
from psochw import PsocHw, PsocHwError
from imuhw import ImuHw, ImuHwError

# The purpose of this module is to test the I2C bus through the clients that use the bus
# There are 2 I2C buses on the BBB, but due to the current device tree configuration
# only 1 bus is exposed on the external connector.  But, after some throughput analysis
# it appears that 100khz is more than sufficient to handle both devices on the same bus.
# It is the job of this module to test and validate that that is the case.

# The approach here will be to run three simple scenarios:
#     - simple  - here we will just exercise the velocity, odometry, and imu interfaces
#                 at a leisurely pace making sure the data is correct.  The access rates
#                 will be much less than the typical usage.
#     - typical - here we will take the same approach as simple but using the typical access rates.
#     - stress  - again, we'll exercise the same interfaces, but push the access rates
#                 to beyond reasonable and try and break it if possible.


logging.basicConfig(level=logging.INFO,
                    format='[%(levelname)s] (%(threadName)-10s) %(message)s',
                    )

def imu_worker(imu, delay, runtime):
    start = time.time()
    time.sleep(random.random())
    delta_time = time.time() - start
    while delta_time < runtime:
        start_cmd = time.time()
        data = imu.GetImuData()        
        logging.info("GetImuData: {}".format(time.time() - start_cmd))
        time.sleep(delay)
        delta_time = time.time() - start   
 
def psoc_speed_worker(psoc, delay, runtime):
    start = time.time()
    time.sleep(random.random())
    delta_time = time.time() - start
    while delta_time < runtime:
        start_cmd = time.time()
        psoc.SetSpeed(0.2, 0.2)
        hb = psoc.GetHeartbeat()
        logging.info("SetSpeed ({}): {}".format(hb, time.time() - start_cmd))
        time.sleep(delay)
        delta_time = time.time() - start

def psoc_odom_worker(psoc, delay, runtime):
    start = time.time()
    time.sleep(random.random())
    delta_time = time.time() - start
    while delta_time < runtime:
        start_cmd = time.time()
        data = psoc.GetOdometry()
        hb = psoc.GetHeartbeat()
        logging.info("GetOdometry ({}): {}".format(hb, time.time() - start_cmd))
        time.sleep(delay)
        delta_time = time.time() - start

def start_threads(imu, psoc, config):
    imu_t = threading.Thread(name='imu_worker', target=imu_worker, args=(imu, config['imu']['delay'], config['imu']['runtime']))
    psoc_t1 = threading.Thread(name='psoc_speed_worker', target=psoc_speed_worker, args=(psoc, config['psoc1']['delay'], config['psoc1']['runtime']))
    psoc_t2 = threading.Thread(name='psoc_odom_worker', target=psoc_odom_worker, args=(psoc, config['psoc2']['delay'], config['psoc1']['runtime']))

    imu_t.start()
    psoc_t1.start()
    psoc_t2.start()

    return imu_t, psoc_t1, psoc_t2

def join_threads(thread_list):
    for t in thread_list:
        t.join()

if __name__ == "__main__":
    print("Instantiating i2c bus for psoc ...")
    try:
        i2c_bus = I2CBus(I2CBus.DEV_I2C_1)
    except RuntimeError:
        print("Unable to create I2CBus device {}".format(I2CBus.DEV_I2C_1))
        sys.exit(1)

    print("Instantiating psoc ...")
    try:
        psoc = PsocHw(i2c_bus, PsocHw.PSOC_ADDR)
    except PsocHwError as e:
        print("Failed to create PsocHw instance {} - {}".format(PsocHw.PSOC_ADDR, e.args))
        sys.exit(1)

    print("Instantiating imu (this takes few seconds) ...")
    try:
        imu = ImuHw()
    except ImuHwError as e:
        print("Failed to create ImuHw instance {}".format(e.args))
        sys.exit(1)

    simple_config = {'imu': {'delay': 1, 'runtime': 60}, 'psoc1': {'delay': 1, 'runtime': 60}, 'psoc2': {'delay': 1, 'runtime': 60}}
    typical_config = {'imu': {'delay': 0.1, 'runtime': 60}, 'psoc1': {'delay': 0.1, 'runtime': 60}, 'psoc2': {'delay': 0.05, 'runtime': 60}}
    stress_config = {'imu': {'delay': 0, 'runtime': 60}, 'psoc1': {'delay': 0, 'runtime': 60}, 'psoc2': {'delay': 0, 'runtime': 60}}


    # Perform Simple Test
    print("--------------------------")
    print("Performing simple test ...")
    print("--------------------------")
    t_list = start_threads(imu, psoc, simple_config)
    join_threads(t_list)

    # Perform Typical Test
    print("---------------------------")
    print("Performing typical test ...")
    print("---------------------------")
    t_list = start_threads(imu, psoc, typical_config)
    join_threads(t_list)

    # Perform Stress Test
    print("--------------------------")
    print("Performing stress test ...")
    print("--------------------------")
    t_list = start_threads(imu, psoc, stress_config)

#--------------------------------------------------------------------------------------------------    


