//
// Created by root on 11/17/17.
//
#include "cliff.h"
#include "mathematics.h"
#include "map.h"

Cliff cliff;

uint8_t Cliff::getStatus(void)
{
	uint8_t status = 0x00;

	if (getLeft())
		status |= BLOCK_LEFT;

	if (getFront())
		status |= BLOCK_FRONT;

	if (getRight())
		status |= BLOCK_RIGHT;

	//if (status != 0x00){
	//	ROS_WARN("%s %d: Return Cliff status:%x.", __FUNCTION__, __LINE__, status);
	//	beep_for_command(true);
	//}
	return status;
}


