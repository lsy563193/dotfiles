//
// Created by lsy563193 on 4/25/17.
//

#ifndef PP_MOTION_MANAGE_H
#define PP_MOTION_MANAGE_H

#include "laser.hpp"
#include "slam.h"
#include "mathematics.h"
//#include "obstacle_detector.h"
#include <list>
#include <deque>

class MotionManage {
public:
	MotionManage();

	~MotionManage();

	static void pubCleanMapMarkers(uint8_t id, const std::deque<Cell_t>& path, Cell_t* cell_p = nullptr);
//private:
	bool is_align_active(){
		return is_align_active_;
	};
	static Laser* s_laser;
	static Slam* s_slam;

	bool initSucceeded()
	{
		return init_succeeded_;
	}
	bool initSucceeded(bool status)
	{
		init_succeeded_ = status;
	}

private:

	bool turn_to_align(int16_t angle);
	//void robot_obstacles_cb(const obstacle_detector::Obstacles::ConstPtr &msg);
	//bool get_align_angle(float & angle);

	ros::NodeHandle nh_;
	bool is_align_active_;

	volatile enum align_state{
		stop=0,
		start=1,
	}line_align_;

	bool init_succeeded_;

	bool initCleaning(uint8_t cleaning_mode);
	bool initNavigationCleaning(void);
	bool initWallFollowCleaning(void);
	bool initSpotCleaning(void);
	bool initExplorationCleaning(void);
	bool initGoHome(void);
};

#endif //PP_MOTION_MANAGE_H
