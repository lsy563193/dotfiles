
#ifndef __APPOINTMENT_H_
#define  __APPOINTMENT_H_

#include <pthread.h>
#include <vector>
#include <string.h>
#include "dev.h"
#include <ros/ros.h>

namespace Appointment
{
const char afile[]="/opt/ros/indigo/share/pp/appointment.txt";

struct st_appmt
{	//appointment data
	uint8_t num;
	bool enable;
	uint8_t week;
	uint8_t hour;
	uint8_t mint;	
};

typedef struct st_appmt st_appmt;

class Appmt
{
public:
	enum SG{
		GET = 0,
		SET = 1,
	};
	Appmt();

	bool set(std::vector<Appointment::st_appmt> apmt_list);

	bool set(uint8_t appTime);

	std::vector<Appointment::st_appmt> get();

	/*
	 * @brief read write appointment message to apmt_l_ 
	 * @param1 SET or GET
	 * @return error -1,ok 1
	 */
	int8_t rw_routine(Appmt::SG action);

	/*
	 * @brief get the next appointment count down
	 * @return miniutes 
	 */
	uint16_t nextAppointment();

	void resetPlanStatus(void)
	{
		plan_status_ = 0;
		serial.setSendData(SERIAL::CTL_APPOINTMENT_H,0x00);
	}

	void setPlanStatus(uint8_t status)
	{
		plan_status_ = (status>>0x01) & 0x03;
		if (plan_status_ != 0)
			ROS_DEBUG("Plan status return 0x%x.", plan_status_);
	}

	uint8_t getPlanStatus(void) {
		return plan_status_;
	}	

	/*
	 * @brief set appointment to  bottom board
	 * @param1 time in minutes
	 * @param2 0b01 set appointment ,0b10 not set appointment
	 */
	void setPlan2Bottom(uint16_t time);

	void timesUp();

	bool isTimeUpOrWifiSettingAck()
	{
		return time_up_or_wifi_setting_ack_;
	}

	void resetTimeUpOrWifiSettingAck()
	{
		time_up_or_wifi_setting_ack_ = false;
	}

	bool shouldUpdateIdleTimer()
	{
		return update_idle_timer_;
	}

	void resetUpdateIdleTimerFlag()
	{
		update_idle_timer_ = false;
	}

private:

	std::vector<Appointment::st_appmt> apmt_l_;
	bool appointment_set_ ;
	uint16_t appointment_count_;
	uint32_t appointment_time_;
	bool appointment_change_;
	pthread_mutex_t appmt_lock_;

	// This variable indicates if it is a time up situation or wifi setting situation. If it is, it will not play
	// "appointment done" voice after update appointment time ack received.
	bool time_up_or_wifi_setting_ack_{false};

	// This variable is used for updating the timer for idle mode or pause state.
	bool update_idle_timer_{false};

	// Variable for plan status
	uint8_t plan_status_;
};

}

extern Appointment::Appmt appmt_obj;
#endif
