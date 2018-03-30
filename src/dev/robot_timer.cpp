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

void Timer::setM0Plan(uint32_t mint)
{

}

void getM0Clock(uint32_t time)
{

}
