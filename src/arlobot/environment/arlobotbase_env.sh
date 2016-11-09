#!/usr/bin/env bash

source ~/arlobot.conf
source arlobotbase.conf
source $ARLOBOT_CATKINWS_ROOT/devel/setup.bash

exec "$@"


