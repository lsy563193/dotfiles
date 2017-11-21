//
// Created by root on 11/17/17.
//

#ifndef PP_BUMPER_H
#define PP_BUMPER_H

#include "movement.h"
#include <pp/x900sensor.h>
extern pp::x900sensor sensor;


class Bumper{
public:
	uint8_t get_status(void) {
		uint8_t Temp_Status = 0;

		if (getLeft()) {
			Temp_Status |= BLOCK_LEFT;
		}
		if (getRight()) {
			Temp_Status |= BLOCK_RIGHT;
		}
		if (getLidar()) {
			//Temp_Status |= LidarTrig;
			Temp_Status |= BLOCK_FRONT;
		}
		if (Temp_Status == (BLOCK_LEFT | BLOCK_RIGHT) || (Temp_Status & BLOCK_FRONT) != 0)
			Temp_Status = BLOCK_ALL;
		return Temp_Status;
	}

	bool getRight() const
	{
		return sensor.rbumper;
	}

	bool getLeft() const
	{
		return sensor.lbumper;
	}

	uint8_t getLidar() const
	{
		return (uint8_t)sensor.lidar_bumper;
	}

};

extern Bumper bumper;

int8_t bumper_lidar_init(const char *device);
int8_t bumper_lidar_deinit();
int8_t bumper_get_lidar_status(void);
#endif //PP_BUMPER_H
