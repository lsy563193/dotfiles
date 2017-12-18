//
// Created by root on 12/6/17.
//

#ifndef PP_ARCH_HPP
#define PP_ARCH_HPP

#include <stdint.h>
#include <ros/ros.h>


#include "action.hpp"
#include "movement.hpp"
#include "move_type.hpp"
#include "state.hpp"
#include "mode.hpp"


#define ROS_INFO_FL() ROS_INFO("%s,%d",__FUNCTION__, __LINE__)
#define PP_INFO() ROS_INFO("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__)
#define PP_WARN() ROS_WARN("%s,%s,%d",__FILE__,__FUNCTION__, __LINE__)
//#define PP_INFO(...) ROS_LOG( __VA_ARGS__)
//#define ROS_INFO(...) ROS_LOG(::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)

#endif //PP_ARCH_HPP
