//
// Created by root on 11/20/17.
//

#ifndef PP_CHARGER_H
#define PP_CHARGER_H

#include <serial.h>
#include <pp/x900sensor.h>
extern pp::x900sensor sensor;

class Charger {
public:
/*-------------------------------Check if at charger stub------------------------------------*/
	bool is_on_stub(void) {
		// 1: On charger stub and charging.
		// 2: On charger stub but not charging.
		if (sensor.c_s == 2 || sensor.c_s == 1)
			return true;
		else
			return false;
	}

	bool is_directed(void) {
		// 3: Direct connect to charge line but not charging.
		// 4: Direct connect to charge line and charging.
		if (sensor.c_s == 3 || sensor.c_s == 4)
			return true;
		else
			return false;
	}

	void set_start(void) {
		// This function will turn on the charging function.
		serial.setSendData(CTL_CHARGER, 0x01);
	}

	void set_stop(void) {
		// Set the flag to false so that it can quit charger mode_.
		serial.setSendData(CTL_CHARGER, 0x00);
	}

	int getChargeStatus() const
	{
		return sensor.c_s;
	}

	/*1 byte */
//	uint8_t trigger_status_;

};

extern Charger charger;

#endif //PP_CHARGER_H

