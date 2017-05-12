#!/bin/sh -e
#
# rc.local
#
# This script is executed at the end of each multiuser runlevel.
# Make sure that the script will "exit 0" on success or any other
# value on error.
#
# In order to enable or disable this script just change the execution
# bits.
#
# By default this script does nothing.

export HOME=/tmp

#roslaunch pp mybot.launch 2>/dev/null &
roslaunch pp mybot.launch 2>/dev/stdout &
# . /opt/ros/indigo/startup_pp.sh

exit 0
