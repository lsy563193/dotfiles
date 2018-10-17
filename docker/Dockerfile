FROM osrf/ros:indigo-desktop-full

# # ARG ros_ip=172.117.0.1
ARG ros_ip=192.168.0.1
ENV ROS_IP=${ros_ip}  
ENV ROS_MASTER_URI=http://${ros_ip}:11311
# COPY ./sources.list /etc/apt/sources.list
COPY ./default.rviz /root/.rviz/

# RUN  apt-get update 