#!/bin/bash
  if [ "$1" == "clion" ];then
    echo $arg
    xhost +
    docker run -it \
      --env="DISPLAY" \
      --env="QT_X11_NO_MITSHM=1" \
      --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v ~/config.rviz:/root/.rviz/default.rviz -v /opt/clion-2017.2.1:/opt/clion -v ~/.CLion2017.2:/root/.CLion2017.2 -v ~/catkin_ws:/root/catkin_ws \
      docker:latest \
     bash -c "/opt/clion/bin/clion.sh"
  fi
  if [ "$1" == "rviz" ];then
      docker run -it \
         --add-host=ilife:192.168.0.1 \
         --env="DISPLAY" \
         --env="QT_X11_NO_MITSHM=1" \
         --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v ~/.config.rviz:/root/.rviz/default.rviz \
      docker:latest \
      bash -c "rosrun rviz rviz /root/.rviz/default.rviz"
  fi 
#http://wiki.ros.org/docker/Tutorials/GUI 
#http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration#Nvidia 
#https://github.com/yeasy/docker_practice/blob/master/image/dockerfile/copy.md
