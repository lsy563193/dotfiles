//
// Created by root on 11/17/17.
//

#ifndef PP_PLANER_H
#define PP_PLANER_H

#include "ros/ros.h"

//#include "pp.h"
class Timer {
public:
	Timer() {
		plan_status_ = 0;
		work_start_time_ = time(NULL);
		wall_follow_start_time_ = time(NULL);
		trap_start_time_ = time(NULL);
	}

	void initWorkTimer(void);

	uint32_t getWorkTime(void);

	void pauseWorkTimer(void);

	void resumeWorkTimer(void);

	void initWallFollowTimer(void);

	bool wallFollowTimeout(double duration);

	void initTrapTimer(void);

	bool trapTimeout(double duration);

	void resetPlanStatus(void)
	{
		plan_status_ = 0;
	}

	void setPlanStatus(uint8_t Status) {
		plan_status_ = Status;
		if (plan_status_ != 0)
			ROS_DEBUG("Plan status return %d.", plan_status_);
	}

	uint8_t getPlanStatus(void) {
		return plan_status_;
	}

private:
	// Variable for plan status
	uint8_t plan_status_;

	time_t work_start_time_;
	uint32_t saved_work_time_;
	time_t wall_follow_start_time_;
	time_t trap_start_time_;
};

extern Timer robot_timer;

#endif //PP_PLANER_H
