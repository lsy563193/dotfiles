
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

	Appmt();

	bool set(Appointment::st_appmt &apmt);

	bool set(uint8_t appTime);

	std::vector<Appointment::st_appmt> get();

	int8_t rd_routine(Appointment::st_appmt *vals);


	bool isActive()
	{
		if(appointment_set_) 
			return (uint32_t)(robot_timer.getRealTimeInMint() -  appointment_time_ ) >= appointment_count_;
		else
			return false;
	}
	/*
	 * @brief get the latest appointment from now
	 * @return miniutes 
	 */
	uint32_t getLastAppointment();

	void resetPlanStatus(void)
	{
		plan_status_ = 0;
	}

	void setPlanStatus(uint8_t Status) {
		plan_status_ = Status;
		if (plan_status_ != 0)
			ROS_DEBUG("Plan status return %d.", plan_status_);
	}

	uint8_t getPlanStatus(void) {
		return plan_status_;
	}	

	/*
	 * @brief set appointment to  bottom board
	 * @param1 time in minutes
	 * @param2 0b01 set appointment ,0b10 not set appointment
	 */
	void setPlan2Bottom(uint32_t time,bool appointment_set);

private:

	std::vector<Appointment::st_appmt> apmt_l_;
	bool appointment_set_ ;
	bool setorget_;
	uint32_t appointment_count_;
	uint32_t appointment_time_;
	bool appointment_change_;
	pthread_mutex_t appmt_lock_;


	// Variable for plan status
	uint8_t plan_status_;
};

}

extern Appointment::Appmt appmt_obj;
#endif
