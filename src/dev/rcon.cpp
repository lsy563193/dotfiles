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

int Rcon::convertToEnum(uint32_t rcon_status)
{
	if (rcon_status & RconL_HomeT)
		return left;
	if (rcon_status & RconFL_HomeT)
		return fl;
	if (rcon_status & RconFL2_HomeT)
		return fl2;
	if (rcon_status & RconFR2_HomeT)
		return fr2;
	if (rcon_status & RconFR_HomeT)
		return fr;
	if (rcon_status & RconR_HomeT)
		return right;
	if (rcon_status & RconBR_HomeT)
		return br;
	if (rcon_status & RconBL_HomeT)
		return bl;
	return 0;
}

uint32_t Rcon::convertFromEnum(int _enum)
{
	switch (_enum)
	{
		case bl:
			return RconBL_HomeT;
		case left:
			return RconL_HomeT;
		case fl2:
			return RconFL_HomeT;
		case fl:
			return RconFL_HomeT;
		case fr:
			return RconFR_HomeT;
		case fr2:
			return RconFR2_HomeT;
		case right:
			return RconR_HomeT;
		case br:
			return RconBR_HomeT;
		default:
			return 0;
	}
}

