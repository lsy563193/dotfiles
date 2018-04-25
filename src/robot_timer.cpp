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

void Timer::setRealTime(struct tm &real_time)
{
	time_t ltime = time(NULL);
	time_t rtime = mktime(&real_time);
	if(ltime <= rtime)
		realtime_offset_ = (uint32_t)difftime(rtime,ltime);
	else
		realtime_offset_ = (uint32_t)difftime(ltime,rtime);
	ROS_INFO("%s,%d,\033[1;40;35m time from cloud %s , time offset %u\033[0m",
				__FUNCTION__,__LINE__,asctime(&real_time),realtime_offset_);
}

void Timer::setRealTime(uint16_t min)
{
	time_t ltime = time(NULL);
	struct tm *local_time = localtime(&ltime);
	ROS_INFO("time local %s",asctime(local_time));
	struct tm new_time;
	new_time.tm_mday = local_time->tm_mday;
	new_time.tm_mon =local_time->tm_mon;
	new_time.tm_year = local_time->tm_year;
	new_time.tm_hour = (min/60)%24;
	new_time.tm_min = min%60;
	new_time.tm_sec = 0;

	time_t newtime = mktime(&new_time);
	if(newtime >= ltime)
		realtime_offset_ =(uint32_t)difftime(newtime,ltime);
	else
		realtime_offset_ =(uint32_t)difftime(ltime,newtime);
	ROS_INFO("%s,%d,\033[1;40;35mtime from IR_Remote %s time offset %u\033[0m",
				__FUNCTION__,__LINE__,asctime(&new_time),realtime_offset_);
}

void Timer::updateRealTimeFromMint(struct tm &dt, uint16_t diff_mins)
{
	uint8_t mint_sum = dt.tm_min + (uint8_t)(diff_mins%60);
	uint16_t hour_sum= dt.tm_hour + diff_mins/60;
	dt.tm_min = mint_sum % 60;
	dt.tm_hour = (mint_sum >= 60)?(hour_sum+1) %24:hour_sum %24;
	uint8_t days_count = hour_sum / 24;
	//leap year cal
	bool leap_year = false;
	int year = dt.tm_year+1900;
	if(year % 4 && !(year %100))
	{
		////century year
		if(year - (year % 100)*100 == 0)
			if( year % 400 == 0)
				leap_year = true;
	}

	//month and day cal
	uint8_t day_sum = dt.tm_mday + days_count;

	// -- tm_mon (0~11)
	// -- equal fabruray
	int month = dt.tm_mon+1;
	if(month== 2)
	{
		if(leap_year && day_sum <=29 )
			dt.tm_mday +=days_count;
		else if(leap_year && day_sum > 29)
			dt.tm_mday = day_sum %29,month += 1;
		else if(!leap_year && day_sum <= 28 )
			dt.tm_mday += days_count;
		else if(!leap_year && day_sum >28 )
			dt.tm_mday  = day_sum %28, month+= 1;
		dt.tm_mon = month-1;
	}
	// -- least than july
	else if(month <= 7)
	{
		if( month %2 == 0)
		{
			if(day_sum <=30 )
				dt.tm_mday += days_count;
			else
				dt.tm_mday = day_sum % 30,month+= 1;

		}
		else if(dt.tm_mon %2 != 0 )
		{
			if(day_sum <= 31)
				dt.tm_mday += days_count;
			else
				dt.tm_mday = day_sum % 31,month +=1;
		}
		dt.tm_mon = month - 1;
	}
	// -- begger than august
	else if(month >= 8)
	{
		if(month %2 == 0 )
		{
			if(day_sum <=31 )
				dt.tm_mday += days_count;
			else
				dt.tm_mday = day_sum % 31,month += 1;
		}
		else if(month %2 != 0 && hour_sum >= 24)
		{
			if(day_sum <= 30)
				dt.tm_mday += days_count;
			else
				dt.tm_mday = day_sum % 30,month +=1;
		}
		dt.tm_mon = month-1;
	}
	//year 
	if(	dt.tm_mon> 11)
		dt.tm_year += 1,dt.tm_mon = 0;

	// --set realtime offset
	realtime_offset_ = (uint32_t)difftime(mktime(&dt),time(NULL));
}

/*
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

uint32_t Timer::getLocalTimeInMint()
{
	time_t t_t = time(NULL);
	struct tm *tm_t = localtime(&t_t);
	uint32_t w = tm_t->tm_wday+1;
	return (uint32_t)(w*24*60+tm_t->tm_hour*60+tm_t->tm_min);
}
*/

uint32_t Timer::getRobotUpTime()
{
	auto robot_time = static_cast<uint32_t>(difftime(time(NULL), robot_start_time_));
	return robot_time;
}

void Timer::getRealtime(struct tm *date_time)
{
	time_t real_time = time(NULL)+static_cast<time_t>(realtime_offset_);
	date_time = localtime( &real_time );
	ROS_INFO("%s,%d current realtime: %s",__FUNCTION__,__LINE__,asctime(date_time));
}
