//
// Created by root on 11/17/17.
//

#include "ros/ros.h"
#include "robot_timer.h"

Timer robot_timer;

void Timer::initWorkTimer(void)
{
	work_start_time_ = time(NULL);
	saved_work_time_ = 0;
}

uint32_t Timer::getWorkTime()
{
	auto work_time = static_cast<uint32_t>(difftime(time(NULL), work_start_time_) + saved_work_time_);
	return work_time;
}

void Timer::pauseWorkTimer()
{
	saved_work_time_ = getWorkTime();
}

void Timer::resumeWorkTimer()
{
	work_start_time_ = time(NULL);
}

void Timer::initWallFollowTimer()
{
	wall_follow_start_time_ = time(NULL);
}

bool Timer::wallFollowTimeout(double duration)
{
	return difftime(time(NULL), wall_follow_start_time_) > duration;

}

void Timer::initTrapTimer()
{
	trap_start_time_ = time(NULL);
}

bool Timer::trapTimeout(double duration)
{
	return difftime(time(NULL), trap_start_time_) > duration;
}


uint8_t Timer::setAppointment(uint8_t num,uint8_t enable,uint8_t week,uint8_t hour,uint8_t mint)
{
	if(planEnable_.size() < num)
		planEnable_[num] = enable;
	if(hours_.size() < num)
		hours_[num] = hour;
	if(mints_.size() < num)
		mints_[num] = mint;
	if(weeks_.size() < num)
		weeks_[num] = week;
}
