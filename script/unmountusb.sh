#!/bin/sh
if [ -e /dev/sda ];then
	umount /mnt/usb
	sync
	/opt/ros/indigo/lib/wav_play/wav_play /opt/ros/indigo/share/pp/audio/32.wav
fi
exit 0

