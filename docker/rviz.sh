#!/bin/bash
xhost +
# if [ "$1" == "rviz" ];then
  echo "rviz"
  docker run -it \
    --add-host=ilife:192.168.0.1 \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	-v /home/syue/CLionProjects/cartographer_ros/cartographer_ros/launch/demo_revo_lds.launch:/opt/cartographer_ros/share/cartographer_ros/launch/demo_revo_lds.launch \
	-v /home/syue/catkin_ws/install_isolated/share/cartographer_ros/configuration_files/demo_2d.rviz:/opt/cartographer_ros/share/cartographer_ros/configuration_files/demo_2d.rviz \
  cartographer_ros:latest  \
bash -c "roslaunch /opt/cartographer_ros/share/cartographer_ros/launch/demo_revo_lds.launch"

#bash -c "rosrun rviz rviz /root/.rviz/default.rviz"

# -v /home/syue/.rviz/default.rviz:/opt/ros/indigo/share/rviz/default.rviz \
  
# fi 
#http://wiki.ros.org/docker/Tutorials/GUI 
#http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration#Nvidia 
#https://github.com/yeasy/docker_practice/blob/master/image/dockerfile/copy.md
