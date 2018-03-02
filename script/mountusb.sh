#!/bin/sh

mount_state=0
cp_state1=0
cp_state2=0
kill_pp_state=0
pp_log_size=0
pp_log1="/tmp/.ros/log/latest"
pp_log2="/tmp/pp.log"
dir_usb="/mnt/usb"
wav_play="/opt/ros/indigo/lib/wav_play/wav_play"
wav_dir="/opt/ros/indigo/share/pp/audio"

#### mount device #####
if [ ! -d $dir_usb ];then
	mkdir $dir_usb && chmod 755 $dir_usb
fi

if [ $# -ne 0 ];then
	mount -t vfat /dev/$1 /mnt/usb
	if [ "$?" -eq 0 ];then
		mount_state=1
		echo "mount /dev/$1 on /mnt/usb success!!"
	else
		echo "mount fail!"
		#$wav_play "$wav_dir/53.wav"
	fi
	sync
fi

#### kill pp ####

#pgrep -lf roslaunch | awk '{print $1}' | xargs kill
proc_id=0
proc_id=`pgrep -lf roslaunch | awk '{print $1}'`
echo "pp process id = $proc_id"

while [ -n "$proc_id" ]
do
	echo "kill pp..."
	kill $proc_id
	sleep 1
	proc_id=`pgrep -lf roslaunch | awk '{print $1}'`
	if [ -z $proc_id ];then
		kill_pp_state=1
		echo "kill pp state=$kill_pp_state"
		break
	fi
done

#### copy log files ####
if [ $mount_state -eq 1 ];then
	$wav_play "$wav_dir/51.wav"
	cp -rLP $pp_log1 /mnt/usb
	if [ "$?" -eq 0 ];then
		echo "copy $pp_log2 success!"
		cp_state1=1
	else
		echo "copy $pp_log2 fail!"
	fi

	cp $pp_log2 /mnt/usb
	if [ "$?" -eq 0 ];then                   
		echo "copy $pp_log1 success!"
		cp_state2=1
	else
		echo "copy $pp_log1 fail!"
	fi

	sync													 
	if [ "$cp_state1" -eq 1 -a "$cp_state2" -eq 1 ];then                                                      
		$wav_play "$wav_dir/52.wav"
	else                                                                               
		$wav_play "$wav_dir/53.wav"      
	fi
fi

#### unmount device ####
if [ $mount_state -eq 1 ];then
	umount /mnt/usb  
	echo "umount usb device"
fi

#### clear log file ####
pp_log_size=`ls -l $pp_log2 | awk ' {print $5}'`
echo "$pp_log2 size = $pp_log_size"
if [ $pp_log_size -gt 1024000 ];then
	echo "log size great then 10240, clear it!!"
	echo -n "" > $pp_log2
fi 
sync

exit 0

