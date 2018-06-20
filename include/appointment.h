
#ifndef __APPOINTMENT_H_
#define  __APPOINTMENT_H_

#include <pthread.h>
#include <vector>
#include <cstdint>

namespace Appointment
{
const char afile[]="/opt/ros/indigo/share/pp/appointment";

struct st_appmt
{	//appointment data
	uint8_t num;
	bool enable;
	uint8_t week;
	uint8_t hour;
	uint8_t mint;
};

#define TOTAL_MINS_A_WEEK (10080)

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
	 * @brief read/write appointment data
	 * @param1 action = (SET/ GET)
	 * @return true/false
	 */
	bool process(Appmt::SG action);

	/*
	 * @brief get the next appointment count down
	 * @return miniutes 
	 */
	uint16_t nextAppointment();

	void resetPlanStatus(void);


	void setPlanStatus(uint8_t status)
	{
		plan_status_ = (status>>0x01) & 0x03;
//		if (plan_status_ != 0)
//			ROS_INFO("Plan status return 0x%x.", plan_status_);
	}

	uint8_t getPlanStatus(void) {
		return plan_status_;
	}	

	/*
	 * @brief set appointment to  bottom board
	 * @param1 time in minutes
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

	std::vector<Appointment::st_appmt> appointment_list_;
	bool appointment_set_ ;
	uint16_t count_down_;
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
