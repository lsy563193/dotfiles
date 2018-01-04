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

#define INFO_RED(X)		ROS_INFO("%s,%d,\033[1;40;31m"#X"\033[0m",__FUNCTION__,__LINE__)
#define INFO_GREEN(X)	ROS_INFO("%s,%d,\033[1;40;32m"#X"\033[0m",__FUNCTION__,__LINE__)
#define INFO_YELLOW(X)	ROS_INFO("%s,%d,\033[1;40;33m"#X"\033[0m",__FUNCTION__,__LINE__)
#define INFO_BLUE(X)	ROS_INFO("%s,%d,\033[1;40;34m"#X"\033[0m",__FUNCTION__,__LINE__)
#define INFO_PURPLE(X)	ROS_INFO("%s,%d,\033[1;40;35m"#X"\033[0m",__FUNCTION__,__LINE__)
#define INFO_CYAN(X)	ROS_INFO("%s,%d,\033[1;40;36m"#X"\033[0m",__FUNCTION__,__LINE__)
#define INFO_WHITE(X)	ROS_INFO("%s,%d,\033[1;40;37m"#X"\033[0m",__FUNCTION__,__LINE__)
#define INFO_BLACK(X)	ROS_INFO("%s,%d,\033[1;40;30m"#X"\033[0m",__FUNCTION__,__LINE__)

//#define PP_INFO(...) ROS_LOG( __VA_ARGS__)
//#define ROS_INFO(...) ROS_LOG(::ros::console::levels::Info, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)

#endif //PP_ARCH_HPP
