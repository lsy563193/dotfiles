
#ifndef __APPOINTMENT_H_
#define  __APPOINTMENT_H_

#include <pthread.h>
#include <vector>
#include <string.h>
#include "dev.h"

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

	std::vector<Appointment::st_appmt> get();

	static int8_t rd_routine(Appointment::st_appmt *vals);

	static bool appointment_set_ ;

	static std::vector<Appointment::st_appmt> apmt_l_;

	/*
	 * @brief get the latest appointment from now
	 * @return miniutes 
	 */
	uint32_t getLastAppointment();
private:

	//pthread_t th_id_;


};

}

extern Appointment::Appmt appmt_obj;
#endif
