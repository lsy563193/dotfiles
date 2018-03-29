//
// Created by root on 11/17/17.
//

#ifndef PP_PLANER_H
#define PP_PLANER_H

#include "ros/ros.h"

class Timer {
public:
	Timer() {
		plan_status_ = 0;
		work_start_time_ = time(NULL);
		wall_follow_start_time_ = time(NULL);
		trap_start_time_ = time(NULL);
		weeks_= std::vector<uint8_t>(10,0);
		hours_= std::vector<uint8_t>(10,0);
		mints_= std::vector<uint8_t>(10,0);
		planEnable_ = std::vector<uint8_t>(10,0);
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
		plan_status_ |= Status;
		if (plan_status_ != 0)
			ROS_DEBUG("Plan status return %d.", plan_status_);
	}

	uint8_t getPlanStatus(void) {
		return plan_status_;
	}

	uint8_t getWeeks(uint8_t pos)
	{
		if(pos < 10)
			return weeks_[pos];
		else
			return 0;
	}

	uint8_t getHours(uint8_t pos)
	{
		if(pos < 10)
			return hours_[pos];
		else
			return 0;
	}

	uint8_t getMints(uint8_t pos)
	{
		if(pos < 10)
			return mints_[pos];
		else
			return 0;
	}

	uint8_t getPlanEnable(uint8_t pos)
	{
		if(pos < 10)
			return planEnable_[pos];
		else
			return 0;
	}

	uint8_t setAppointment(uint8_t num,uint8_t enable,uint8_t weeks,uint8_t hours,uint8_t mints);

private:
	// Variable for plan status
	uint8_t plan_status_;

	time_t work_start_time_;
	uint32_t saved_work_time_;
	time_t wall_follow_start_time_;
	time_t trap_start_time_;

	std::vector<uint8_t> planEnable_;
	std::vector<uint8_t> weeks_;
	std::vector<uint8_t> hours_;
	std::vector<uint8_t> mints_;
};

extern Timer robot_timer;

#endif //PP_PLANER_H
