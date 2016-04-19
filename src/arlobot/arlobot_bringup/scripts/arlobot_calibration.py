#! /usr/bin/env python

# Purpose:
#   1. Retrieve current (possibly default) calibration values from each Psoc (just in case we have to restore them and
#      also for possible analysis/comparison.
#   2. Store retrieved values in 'old' calibration file (maybe we store multiple 'old' files.  How many?)
#   3. Initiate calibration sequence on each Psoc
#   4. Retrieve calibration values from Psoc and store into file

CONTROL_DOWNLOAD_CALIBRATION = 0x10
STATUS_DOWNLOADING = 0x10
START_TRANSMIT = 0
END_TRANSMIT = 0xffff

cal_values = []

def download_calibration():
    # Send command to Psoc to start transfer of calibration data
    psoc.SetControl(DOWNLOAD_CALIBRATION)

    # Wait for Psoc to acknowledge downloading request in the status register
    while not psoc.GetStatus() & DOWNLOADING: pass

    # Wait for the Psoc to issue a start transmission via the calibrate register
    while not psoc.GetCalibrate() is START_TRANSMIT: pass

    # Handshake with Psoc for each value until end of transfer complete
    # Read values from the calibrate register until end transmission received
    value = psoc.GetCalibrate()
    while not value != END_TRANSMIT:
        cal_values.append(value)
        value = psoc.GetCalibrate()

    # Wait for Psoc to indicate downloading is complete
    while psoc.GetStatus() & DOWNLOADING: pass

    print cal_values


def initiate_calibration():
    # Send command to Psoc to start calibration
    psoc.SetControl(START_CALIBRATION)

    # Wait for Psoc to acknowledge calibration request in the status register
    while not psoc.GetStatus() & CALIBRATING: pass

    # Wait for Psoc to indicate calibration is complete in the status register
    while psoc.GetStatus() & CALIBRATING: pass

    # Wait for Psoc to indicate it is calibrated
    while not psoc.GetStatus() & CALIBRATED: pass


if __name__ == "__main__":
    # Request Psoc to download calibration data
    download_calibration()

    initiate_calibration()

    download_calibration()
