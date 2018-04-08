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

void Timer::setPlan2Bottom(uint32_t mint)
{
	mint=mint;
}

struct Timer::DateTime Timer::getClockFromBottom()
{

}

void Timer::setRealTime(Timer::DateTime date_time)
{
	date_time_.year = date_time.year;
	date_time_.month = date_time.month;
	date_time_.day = date_time.day;
	date_time_.hour = date_time.hour;
	date_time_.mint = date_time.mint;
	date_time_.sec = date_time.sec;
	ROS_INFO("%s,%d,\033[1m %u/%u/%u %u:%u:%u\033[0m",
				__FUNCTION__,__LINE__,
				date_time_.year,date_time_.month,date_time_.day,
				date_time_.hour,date_time.mint,date_time.sec);
}

uint32_t Timer::getRealTimeInMint()
{
	uint32_t w = getRealTimeWeekDay();	
	return (uint32_t)(w*24*60+date_time_.hour*60+date_time_.mint);
}

uint32_t Timer::getRealTimeWeekDay()
{
	uint32_t c = date_time_.year/100;
	uint32_t y = date_time_.year - c*100;
	uint32_t m = date_time_.month;
	uint32_t d = date_time_.day;
	if(m == 1 || m == 2)
	{
		if(m == 1)
			m = 13;
		else
			m = 14;		  
	}
	uint32_t w = (uint32_t)(( c/4 - 2*c + y + y/4 + 26*(m+1)/10 + d - 1) % 7);

	ROS_INFO("%s,%d,\033[1m weekday %u \033[0m",__FUNCTION__,__LINE__,w);

	return w;
}

char* Timer::asctime()
{
	static char buf[50];
	sprintf(buf, "%d/%d/%d %d:%d:%d",
				date_time_.year,date_time_.month, date_time_.day,
				date_time_.hour,date_time_.mint,date_time_.sec);
	return buf;
	
}
