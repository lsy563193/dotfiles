//
// Created by root on 11/17/17.
//

#include "ros/ros.h"
#include "robot_timer.h"
#include "serial.h"
#include "appointment.h"

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

uint32_t Timer::getRobotUpTime()
{
	auto robot_time = static_cast<uint32_t>(difftime(time(NULL), robot_start_time_));
	return robot_time;
}

void Timer::setRealTimeOffsetByRemote(uint16_t minute)
{
	time_t origin_calendar_time;
	getRealCalendarTime(origin_calendar_time);
	struct tm *s_origin_time = localtime(&origin_calendar_time);
	ROS_INFO("%s %d Origin calendar time: %s ", __FUNCTION__, __LINE__, ctime(&origin_calendar_time));
	struct tm s_new_calendar_time;
	s_new_calendar_time.tm_mday = s_origin_time->tm_mday;
	s_new_calendar_time.tm_mon = s_origin_time->tm_mon;
	s_new_calendar_time.tm_year = s_origin_time->tm_year;
	s_new_calendar_time.tm_hour = (minute/60)%24;
	s_new_calendar_time.tm_min = minute%60;
	s_new_calendar_time.tm_sec = 0;

	time_t new_calendar_time = mktime(&s_new_calendar_time);
	setRealTimeOffset(new_calendar_time);
	ROS_INFO("%s %d New calendar time: %s ", __FUNCTION__, __LINE__, asctime(&s_new_calendar_time));
}

void Timer::getRealCalendarTime(time_t &real_calendar_time)
{
	// Robot calendar time.
	time_t robot_calendar_time = time(NULL);
	real_calendar_time = robot_calendar_time + static_cast<time_t>(realtime_offset_);
//	ROS_INFO("%s,%d Real calendar time: %s",__FUNCTION__,__LINE__,ctime(&real_calendar_time));
}

void Timer::setRealTimeOffset(time_t new_calendar_time)
{
	// Robot calendar time.
	time_t calendar_time = time(NULL);
	// Time offset.
	realtime_offset_ = difftime(new_calendar_time, calendar_time);
	ROS_INFO("%s %d:\033[1;40;35m Set time offset %f.\033[0m", __FUNCTION__, __LINE__, realtime_offset_);
}
