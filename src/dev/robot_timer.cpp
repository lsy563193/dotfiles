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

bool Timer::setAppointment(uint8_t num,uint8_t isEnable,uint8_t week,uint8_t hour,uint8_t mint)
{
	//ROS_INFO("%s,%d,set plan num %d, week %d,hour %d,minutes %d",__FUNCTION__,__LINE__,num,week,hour,mint);
	if(num <= planEnable_.size())
		planEnable_.at(isEnable);
	if(num <= weeks_.size())
		weeks_.at(num)  = week;
	if(num <= hours_.size())
		hours_.at(num) = hour;
	if(num <= mints_.size())
		mints_.at(num)= mint;
}
