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

void Timer::setRealTime(Timer::DateTime date_time)
{
	date_time_.year = date_time.year;
	date_time_.month = date_time.month;
	date_time_.day = date_time.day;
	date_time_.hour = date_time.hour;
	date_time_.mint = date_time.mint;
	date_time_.sec = date_time.sec;
	ROS_INFO("%s,%d,\033[1;41;35m from cloud %u/%u/%u %u:%u:%u\033[0m",
				__FUNCTION__,__LINE__,
				date_time_.year,date_time_.month,date_time_.day,
				date_time_.hour,date_time.mint,date_time.sec);
	//--update appointment count down ,set appointment to bottom board
	//uint16_t mint = appmt_obj.getNewestAppointment();
	//appmt_obj.setPlan2Bottom(mint);
}

void Timer::setRealTime(uint16_t mints)
{
	time_t ltime = time(NULL);
	struct tm *local_time = localtime(&ltime);
	date_time_.day = (uint8_t)local_time->tm_mday;
	date_time_.month =(uint8_t)(local_time->tm_mon+1);
	date_time_.year = (uint16_t)(local_time->tm_year+1900);
	date_time_.hour = (uint8_t)((mints/60)%24);
	date_time_.mint = (uint8_t)(mints%60);
	date_time_.sec = 0;
	ROS_INFO("%s,%d,\033[1m %u/%u/%u %u:%u:%u\033[0m",
				__FUNCTION__,__LINE__,
				date_time_.year,date_time_.month,date_time_.day,
				date_time_.hour,date_time_.mint,date_time_.sec);
	//--update appointment count down ,set appointment to bottom board
	//uint16_t mint = appmt_obj.getNewestAppointment();
	//appmt_obj.setPlan2Bottom(mint);
}

void Timer::updateRealTimeFromMint(struct Timer::DateTime &dt, uint16_t diff_mins)
{
	uint8_t mint_sum = dt.mint + (uint8_t)(diff_mins%60);
	uint16_t hour_sum= dt.hour + diff_mins/60;
	dt.mint = mint_sum % 60;
	dt.hour = (mint_sum >= 60)?(hour_sum+1) %24:hour_sum %24;
	uint8_t days_count = hour_sum / 24;
	//leap year cal
	bool leap_year = false;
	if(dt.year % 4 && !(dt.year %100))
	{
		////century year
		if(dt.year - (dt.year % 100)*100 == 0)
			if( dt.year % 400 == 0)
				leap_year = true;
	}
	//month and day cal
	uint8_t day_sum = dt.day + days_count;
	if(dt.month == 2)
		if(leap_year && day_sum <=29 )
			dt.day +=days_count;
		else if(leap_year && day_sum > 29)
			dt.day = day_sum %29,dt.month += 1;
		else if(!leap_year && day_sum <= 28 )
			dt.day += days_count;
		else if(!leap_year && day_sum >28 )
			dt.day  = day_sum %28, dt.month += 1;

	else if(dt.month <= 7)
		if(dt.month %2 == 0)
			if(day_sum <=30 )
				dt.day += days_count;
			else
				dt.day = day_sum % 30,dt.month += 1;
		else if(dt.month %2 != 0 )
			if(day_sum <= 31)
				dt.day += days_count;
			else
				dt.day = day_sum % 31,dt.month +=1;

	else if(dt.month >= 8 )
		if(dt.month %2 == 0 )
			if(day_sum <=31 )
				dt.day += days_count;
			else
				dt.day = day_sum % 31,dt.month += 1;
		else if(dt.month %2 != 0 && hour_sum >= 24)
			if(day_sum <= 30)
				dt.day += days_count;
			else
				dt.day = day_sum % 30,dt.month +=1;
	//year 
	if(	dt.month > 12)
		dt.year += 1,dt.month = 1;
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

uint32_t Timer::getLocalTimeInMint()
{
	time_t t_t = time(NULL);
	struct tm *tm_t = localtime(&t_t);
	uint32_t w = tm_t->tm_wday+1;
	return (uint32_t)(w*24*60+tm_t->tm_hour*60+tm_t->tm_min);
}
