#!/usr/bin/env bash

source ~/arlobot.conf
source arlobot.conf
source $ARLOBOT_CATKIN_ROOT/devel/setup.bash

exec "$@"
