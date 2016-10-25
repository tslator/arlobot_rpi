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
import sys
import time
import argparse
import math
sys.path.append("../")
from i2c import I2CBus, I2CBusError
from psochw import PsocHw, PsocHwError


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

    left_dist = 0.0
    right_dist = 0.0
    left_end_dist = distance
    right_end_dist = distance

    while left_dist < left_end_dist and right_dist < right_end_dist:
        psoc.SetSpeed(0.2, 0.2)
        _, _, left_delta_dist, right_delta_dist, _ = psoc.GetOdometry()
        left_dist += left_delta_dist
        right_dist += right_delta_dist
        time.sleep(0.1)

    psoc.SetSpeed(0.0, 0.0)

def DoBackwardMove(psoc, distance):
    # Use fixed velocity: 0.200

    left_dist = 0.0
    right_dist = 0.0
    left_end_dist = -distance
    right_end_dist = -distance

    while left_dist > left_end_dist and right_dist > right_end_dist:
        psoc.SetSpeed(-0.2, -0.2)
        _, _, left_delta_dist, right_delta_dist, _ = psoc.GetOdometry()
        left_dist += left_delta_dist
        right_dist += right_delta_dist
        time.sleep(0.1)

    psoc.SetSpeed(0.0, 0.0)

def Uni2Diff(linear, angular):
    TRACK_WIDTH = 0.403
    WHEEL_DIAMETER = 0.1524
    left = (2*linear - angular*TRACK_WIDTH)/WHEEL_DIAMETER
    right = (2*linear + angular*TRACK_WIDTH)/WHEEL_DIAMETER

    return left, right

def ToDegrees(radians):
    return (radians/math.pi)*360.0

def ToRadians(degrees):
    return (degrees/360.0)*math.pi

def DoCwRotate(psoc, degrees):
    # Use fixed angular velocity

    left, right = Uni2Diff(0.0, -0.035)

    _,_,_,_,heading = psoc.GetOdometry()    
    desired_heading = heading - ToRadians(degrees)

    end_heading = math.atan2(math.sin(desired_heading), math.cos(desired_heading))
    print("Start Heading (deg): ", ToDegrees(heading))
    print("Target Heading (deg): ", ToDegrees(desired_heading))
    print("Expected Final Heading (deg): ", ToDegrees(end_heading))

    while abs(end_heading - heading) > 0.1:
        psoc.SetSpeed(left, right)
        _,_,_,_,heading = psoc.GetOdometry()
        #print(heading, end_heading, abs(end_heading - heading))
        time.sleep(0.001)

    psoc.SetSpeed(0.0, 0.0)
    print("Actual Final Heading (deg): ", ToDegrees(heading))


def DoCcwRotate(psoc, degrees):
    left, right = Uni2Diff(0.0, 0.035)

    _, _, _, _, heading = psoc.GetOdometry()
    desired_heading = heading + ToRadians(degrees)

    end_heading = math.atan2(math.sin(desired_heading), math.cos(desired_heading))
    print("Start Heading (deg): ", ToDegrees(heading))
    print("Target Heading (deg): ", ToDegrees(desired_heading))
    print("Expected Final Heading (deg): ", ToDegrees(end_heading))
    
    while abs(end_heading - heading) > 0.1:
        psoc.SetSpeed(left, right)
        _,_,_,_,heading = psoc.GetOdometry()
        #print(heading, end_heading, abs(end_heading - heading))
        time.sleep(0.001)

    psoc.SetSpeed(0.0, 0.0)
    print("Actual Final Heading (deg): ", ToDegrees(heading))


def main(psoc, args):
    if args.which == 'forward':
        distance = max(min(float(args.distance), 2.0), 0.001)
            
        DoForwardMove(psoc, distance)

    elif args.which == 'backward':
        distance = max(min(float(args.distance), 2.0), 0.001)

        DoBackwardMove(psoc, float(args.distance))

    elif args.which == 'rotate':
        degrees = min(max(float(args.degrees), 0.1), 360.0)
        
        if args.direction == 'cw':
            DoCwRotate(psoc, float(degrees))
        elif args.direction == 'ccw':
            DoCcwRotate(psoc, float(degrees))
    else:
        print("which which?")

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



