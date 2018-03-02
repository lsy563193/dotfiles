#!/bin/sh
export HOME=/tmp
roslaunch pp mybot.launch 1>/tmp/pp.log 2>&1 &
#roslaunch pp mybot.launch 1>/dev/null 2>&1 &
exit 0
