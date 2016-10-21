#! /usr/bin/env python
"""
This module supports the following high-level motions:
    - Move forward 1 meter
    - Move backward 1 meter
    - Rotate counter-clockwise
        - 90 degrees
        - 180 degrees
        - 270 degrees
        - 360 degrees
    - Rotate clockwise
        - 90 degrees
        - 180 degrees
        - 270 degrees
        - 360 degrees
"""
from __future__ import print_function
from src.arlobot.arlobot_bringup.scripts.hal.hw.i2c import I2CBus, I2CBusError
from src.arlobot.arlobot_bringup.scripts.hal.hw.psochw import PsocHw, PsocHwError
import time
import sys
import argparse

def parse():
    parser = argparse.ArgumentParser()

    subparser = parser.add_subparsers()

    forward_parser = subparser.add_parser("forward", help="forward help")
    forward_parser.add_argument("-d", "--distance")
    forward_parser.set_defaults(which='forward')

    backward_parser = subparser.add_parser("backward", help="backward help")
    backward_parser.add_argument("-d", "--distance")
    backward_parser.set_defaults(which='backward')

    rotate_parser = subparser.add_parser("rotate", help="rotate help")
    rotate_parser.add_argument("-d", "--direction", choices=['cw', 'ccw'])
    rotate_parser.add_argument("-g", "--degrees")
    rotate_parser.set_defaults(which='rotate')

    return parser.parse_args()

def DoForwardMove(psoc, distance):
    # Use fixed velocity: 0.200

    _, _, left_dist, right_dist, _ = psoc.GetOdometry()
    left_end_dist = left_dist + distance
    right_end_dist = right_dist + distance

    while left_dist < left_end_dist and right_dist < right_end_dist:
        psoc.SetSpeed(0.2, 0.2)
        _, _, left_dist, right_dist, _ = psoc.GetOdometry()
        time.sleep(0.1)

    psoc.SetSpeed(0.0, 0.0)

def DoBackwardMove(psoc, distance):
    # Use fixed velocity: 0.200

    _, _, left_dist, right_dist, _ = psoc.GetOdometry()
    left_end_dist = left_dist - distance
    right_end_dist = right_dist - distance

    while left_dist > left_end_dist and right_dist > right_end_dist:
        psoc.SetSpeed(-0.2, -0.2)
        _, _, left_dist, right_dist, _ = psoc.GetOdometry()
        time.sleep(0.1)

    psoc.SetSpeed(0.0, 0.0)

def Uni2Diff(linear, angular):
    TRACK_WIDTH = 0.403
    WHEEL_DIAMETER = 0.1524
    left = (2*linear - angular*TRACK_WIDTH)/WHEEL_DIAMETER
    right = (2*linear + angular*TRACK_WIDTH)/WHEEL_DIAMETER

    return left, right

def DoCwRotate(psoc, degrees):
    # Use fixed angular velocity

    left, right = Uni2Diff(0.0, 0.05)

    _, _, _, _, heading = psoc.GetOdometry()

    end_heading = heading - (degrees/360.0)*3.14

    while heading > end_heading:
        psoc.SetSpeed(left, right)
        _, _, _, _, heading = psoc.GetOdometry()
        time.sleep(0.1)

    psoc.SetSpeed(0.0, 0.0)


def DoCcwRotate(psoc, degrees):
    left, right = Uni2Diff(0.0, 0.05)
    _, _, _, _, heading = psoc.GetOdometry()

    end_heading = heading + (degrees/360.0)*3.14

    while heading < end_heading:
        psoc.SetSpeed(left, right)
        _, _, _, _, heading = psoc.GetOdometry()
        time.sleep(0.1)

    psoc.SetSpeed(0.0, 0.0)


def main(psoc, args):
    if args.forward:
        DoForwardMove(psoc, args.distance)

    elif args.backward:
        DoBackwardMove(psoc, args.distance)

    elif args.rotate:
        if args.direction == 'cw':
            DoCwRotate(psoc, args.degrees)
        elif args.direction == 'ccw':
            DoCcwRotate(psoc, args.degrees)


if __name__ == "__main__":
    print("{}:".format(sys.argv[0]))

    args = parse()
    print(args)

    try:
        i2c_bus = I2CBus(I2CBus.DEV_I2C_1)
    except RuntimeError:
        print("Unable to create I2CBus device {}".format(I2CBus.DEV_I2C_1))

    try:
        psoc = PsocHw(i2c_bus, PsocHw.PSOC_ADDR)
    except PsocHwError:
        print("Failed to create PsocHw instance {}".format(PsocHw.PSOC_ADDR))


    main(psoc, args)



