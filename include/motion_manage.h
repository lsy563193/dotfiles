//
// Created by lsy563193 on 4/25/17.
//

#ifndef PP_MOTION_MANAGE_H
#define PP_MOTION_MANAGE_H

#include "laser.hpp"
#include "slam.h"

class MotionManage {
public:
	MotionManage();

	~MotionManage();
//private:
	bool is_align_active(){
		return is_align_active_;
	};
	static Laser* s_laser;
	static Slam* s_slam;
private:

	void robot_obstacles_cb(const obstacle_detector::Obstacles::ConstPtr &msg);
	int16_t get_align_angle(void);

	int slam_type_;
	ros::NodeHandle nh_;
	bool is_align_active_;

	volatile enum align_state{
		stop=0,
		start=1,
	}line_align_;
//	int16_t line_angle_;

	//startup flag;
//	enum start_object {laser,total};
//	std::bitset<total> startup_flag;

};

#endif //PP_MOTION_MANAGE_H
