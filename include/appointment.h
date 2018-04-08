
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
private:

	std::vector<Appointment::st_appmt> apmt_l_;
	bool appointment_set_ ;
	bool setorget_;
	uint32_t appointment_count_;
	uint32_t appointment_time_;
	bool appointment_change_;
	pthread_mutex_t appmt_lock_;


};

}

extern Appointment::Appmt appmt_obj;
#endif
