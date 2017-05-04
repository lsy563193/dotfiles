//
// Created by lsy563193 on 4/25/17.
//

#ifndef PP_MOTION_CONTROLER_H
#define PP_MOTION_CONTROLER_H

//#include "../include/obstacle_detector.h"

//typedef boost::shared_ptr<obstacle_detector::ObstacleDetector> ObstacleDetectorPtr;

class Motion_controller {

public:
	Motion_controller();

	~Motion_controller();
//private:
//	enum start_object {gyro, lidar, obs_det, align, slam,start_obs};

//	std::bitset<start_obs> start_bit;
//  ObstacleDetectorPtr od;
	ros::ServiceClient align_cli_;
};

#endif //PP_MOTION_CONTROLER_H
