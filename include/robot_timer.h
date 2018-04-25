//
// Created by root on 11/17/17.
//

#ifndef PP_PLANER_H
#define PP_PLANER_H

#include "ros/ros.h"

class Timer {
public:
	Timer() {
		robot_start_time_ = time(NULL);
		work_start_time_ = time(NULL);
		wall_follow_start_time_ = time(NULL);
		trap_start_time_ = time(NULL);
		realtime_offset_ = 0;
	}

	void initWorkTimer(void);

	uint32_t getWorkTime(void);

	uint32_t getRobotUpTime();

	void pauseWorkTimer(void);

	void resumeWorkTimer(void);

	void initWallFollowTimer(void);

	bool wallFollowTimeout(double duration);

	void initTrapTimer(void);

	bool trapTimeout(double duration);

	uint32_t getRealTimeOffset()
	{
		return realtime_offset_;
	}
	/*
	 * @brief set real time from SyncClickRxMsg 
	 * @param1 date time
	 */
	void setRealTime(struct tm &realtime);

	/*
	 * @breif set time from bottom board
	 * @param1 realtime in minutes
	 */
	void setRealTime(uint16_t realtime);

	void getRealtime(struct tm *date_time);

	/*
	 * @brief get real time in mintues
	 * from monday
	 * @return time in mintues
	 */
	//uint32_t getRealTimeInMint();

	/*
	 * @brief get real time Weekday 
	 * @return weekdy 1~7
	 */
	//uint32_t getRealTimeWeekDay();

	/*
	 * @brief get local time in mintues
	 * from monday
	 * @return time in mintues
	 */
	//uint32_t getLocalTimeInMint();

	/*
	 * @brief update time from minutes
	 * @param1 cur_time ,give a cur time in struct DateTime 
	 * @param2 mins ,give time in diffrence mintus (0~10080)
	 */
	void updateRealTimeFromMint(struct tm &cur_time,uint16_t mins);

private:
	
	time_t robot_start_time_;
	time_t work_start_time_;
	uint32_t saved_work_time_;
	time_t wall_follow_start_time_;
	time_t trap_start_time_;
	uint32_t realtime_offset_;

};

extern Timer robot_timer;

#endif //PP_PLANER_H
