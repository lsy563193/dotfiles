//
// Created by root on 11/20/17.
//

#ifndef PP_OMNI_H
#define PP_OMNI_H

#include "controller.h"

#include <pp/x900sensor.h>
extern pp::x900sensor sensor;

class Omni {
public:
	Omni()
	{
		is_enable = false;
		omni_detect_cnt=0;
		last_omni_wheel=0;
		stop_ = false;
	}
void reset()
{
	uint8_t reset_byte = controller.getSendData(CTL_OMNI_RESET);
	controller.setSendData(CTL_OMNI_RESET, reset_byte | 0x01);
}

void clear()
{
	uint8_t reset_byte = controller.getSendData(CTL_OMNI_RESET);
	controller.setSendData(CTL_OMNI_RESET, reset_byte & ~0x01);
}
int16_t getOmniWheel()
{
	return sensor.omni_wheel;
}
	bool isEnable()
	{return is_enable;}
	void detect();
	bool stop(){
		return stop_;
	};
	bool set_stop(bool val){
		return stop_ =val;
	};
private:
	bool is_enable;

	int16_t omni_detect_cnt;
	int16_t last_omni_wheel;
	bool stop_ = true;
};

extern Omni omni;

#endif //PP_OMNI_H
