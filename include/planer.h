//
// Created by root on 11/17/17.
//

#ifndef PP_PLANER_H
#define PP_PLANER_H


//#include "pp.h"
class Planer {
public:
	Planer() {
		uint8_t g_status = 0;
	}

	void set_status(uint8_t Status) {
		g_status = Status;
		if (g_status != 0)
			ROS_DEBUG("Plan status return %d.", g_status);
	}

	uint8_t get_status() {
		return g_status;
	}

private:
// Variable for plan status
	uint8_t g_status;
};

extern Planer planer;

#endif //PP_PLANER_H
