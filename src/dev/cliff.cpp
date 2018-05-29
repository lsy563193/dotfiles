//
// Created by root on 11/17/17.
//
#include "cliff.h"
#include "map.h"

Cliff cliff;

uint8_t Cliff::getStatus()
{
	uint8_t status = 0x00;

	if (getLeft())
		status |= BLOCK_LEFT;

	if (getFront())
		status |= BLOCK_FRONT;

	if (getRight())
		status |= BLOCK_RIGHT;

//	if (status != 0x00){
//		beep_for_command(true);
//	}
//	printf("%s %d: Return Cliff status:%x.\n", __FUNCTION__, __LINE__, status);
	return status;
}

bool Cliff::allTriggered()
{
	return getStatus() == BLOCK_ALL;
}
