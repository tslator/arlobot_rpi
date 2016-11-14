#!/usr/bin/env bash

source ~/arlobotbase.conf
source $ARLOBOT_ENV/arlobotbase.conf
source $ARLOBOT_CATKINWS_ROOT/devel/setup.bash

exec "$@"


