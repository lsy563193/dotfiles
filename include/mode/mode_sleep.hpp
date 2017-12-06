//
// Created by austin on 17-12-5.
//

#ifndef PP_MODE_SLEEP_HPP
#define PP_MODE_SLEEP_HPP


#include "mode.hpp"

class ModeSleep: public Mode
{
public:
	ModeSleep();
	~ModeSleep();

	bool isExit();

	// For event handling.
	void remote_clean(bool state_now, bool state_last);
	void key_clean(bool state_now, bool state_last);
	void charge_detect(bool state_now, bool state_last);
	void rcon(bool state_now, bool state_last);
	void remote_plan(bool state_now, bool state_last);

private:
	bool plan_activated_status_;
};
#endif //PP_MODE_SLEEP_HPP
