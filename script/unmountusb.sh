#!/bin/sh
start_pp=/opt/ros/indigo/bin/start_pp.sh
ls /de/sd*
if [ $? -eq 0 ];then
	umount /mnt/usb
	sync
	/opt/ros/indigo/lib/wav_play/wav_play /opt/ros/indigo/share/pp/audio/32.wav
fi

#pid=`pgrep -lf roslaunch | awk '{print $1}'`
#if [ -z $pid ];then
#	$start_pp
#fi	
exit 0
