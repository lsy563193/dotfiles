//
// Created by root on 11/17/17.
//

#include "rcon.h"
#include "mathematics.h"
#include "lidar.hpp"
#include "state.hpp"

Rcon c_rcon;
uint32_t Rcon::getAll(void)
{
	uint32_t rcon_value = getStatus();
	resetStatus();
	return rcon_value;
}

uint32_t Rcon::getForwardTop()
{
	uint32_t rcon_status = getStatus() & (RconFL_HomeT | RconFR_HomeT );
	resetStatus();
	return rcon_status;
}

uint32_t Rcon::getNavRcon()
{

	uint32_t rcon_status = getStatus() & (RconL_HomeT | RconR_HomeT| RconFL_HomeT | RconFR_HomeT | RconFR2_HomeT | RconFL2_HomeT);
	//uint32_t rcon_status = getStatus() & (RconFL_HomeT | RconFR_HomeT | RconFR2_HomeT | RconFL2_HomeT);
	resetStatus();
	return rcon_status;
}

uint32_t Rcon::getWFRcon()
{
	uint32_t rcon_status = getStatus() & (RconFL2_HomeT|RconFR_HomeT|RconFL_HomeT|RconFR2_HomeT);
	resetStatus();
	return rcon_status;
}

