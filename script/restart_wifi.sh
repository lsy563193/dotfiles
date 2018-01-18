#!/bin/sh

# Shut down the wlan0
ifdown wlan0
if [ ! $? = '0' ]; then
	echo -e "\033[1m" "ifdown wlan0 failed!!" "\033[0m"
	exit
fi

# Kill the process of hostapd, then network.sh will restart it up.
kill -9 `pidof hostapd`
