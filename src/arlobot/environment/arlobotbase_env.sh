#!/usr/bin/env bash

#source arlobotbase.conf
#source env.conf
#source $ARLOBOT_CATKIN_ROOT/devel/setup.bash

export ROS_MASTER_URI=http://arlobot:11311
export ROS_IP=`ifconfig eth0 | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}'`
export ROS_HOSTNAME=arlobotbase
source /home/arlo/catkin_ws/devel/setup.bash

exec "@"


