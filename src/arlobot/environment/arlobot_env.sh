#!/usr/bin/env bash

export ROS_IP=`ifconfig wlan0 | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}'`
export ROS_HOSTNAME="arlobot"

source /home/pi/catkin_ws/devel/setup.bash

exec "@"