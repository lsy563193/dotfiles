//
// Created by root on 11/17/17.
//

#ifndef PP_PLANER_H
#define PP_PLANER_H

#include "ros/ros.h"

class Timer {
public:
	Timer() {
		work_start_time_ = time(NULL);
		wall_follow_start_time_ = time(NULL);
		trap_start_time_ = time(NULL);
	}

	struct DateTime{
		uint16_t year;
		uint8_t day;
		uint8_t month;
		uint8_t hour;
		uint8_t mint;
		uint8_t sec;
	};

	void initWorkTimer(void);

	uint32_t getWorkTime(void);

	void pauseWorkTimer(void);

	void resumeWorkTimer(void);

	void initWallFollowTimer(void);

	bool wallFollowTimeout(double duration);

	void initTrapTimer(void);

	bool trapTimeout(double duration);

	/*
	 * @brief set real time to struct DataTime
	 * @param1 date time
	 */
	void setRealTime(struct DateTime);

	/*
	 * @breif set time from bottom board
	 * @param1 realtime in mints
	 */
	void setRealTime(uint16_t realtime);

	struct DateTime getRealTime()
	{
		return date_time_;	
	}

	/*
	 * @brief get real time in mintues
	 * @return time in mintues
	 */
	uint32_t getRealTimeInMint();

	uint32_t getRealTimeWeekDay();

	char* asctime();
private:
	
	time_t work_start_time_;
	uint32_t saved_work_time_;
	time_t wall_follow_start_time_;
	time_t trap_start_time_;
	struct DateTime date_time_;

};

extern Timer robot_timer;

#endif //PP_PLANER_H
