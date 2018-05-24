//
// Created by root on 11/17/17.
//

#ifndef PP_PLANER_H
#define PP_PLANER_H

#include <cstdint>

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

	double getRealTimeOffset()
	{
		return realtime_offset_;
	}
	/*
	 * @brief: Set real time offset between robot calendar time and new calendar time.
	 * @param: New calendar time in time_t.
	 */
	void setRealTimeOffset(time_t new_calendar_time);

	/*
	 * @breif: Set calendar time by remote controller.
	 * @param: Time of today in minutes.
	 */
	void setRealTimeOffsetByRemote(uint16_t minute);

	void getRealCalendarTime(time_t &real_calendar_time);

private:
	
	time_t robot_start_time_;
	time_t work_start_time_;
	uint32_t saved_work_time_;
	time_t wall_follow_start_time_;
	time_t trap_start_time_;
	double realtime_offset_;

};

extern Timer robot_timer;

#endif //PP_PLANER_H
