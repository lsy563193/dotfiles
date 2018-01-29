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
/*	if (mt.is_follow_wall()) {
		if ((rcon_value &
					(RconL_HomeT | RconR_HomeT | RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT))) {
			rcon_status_ = 0;
			return get_trig_(rcon_value);
		}
		else{
			rcon_status_ = 0;
			return 0;
		}
	}
	else if (mt.is_linear()) {
		if (cm_is_exploration()) {
			auto status = rcon_value & RconAll_Home_T;
			rcon_status_ = 0;
			return status;
		}
		if(!found_charger_ && !cs.is_going_home() && estimateChargerPos(rcon_value)){
			rcon_status_ = 0;
			return get_trig_(rcon_value);
		}
		else if (!(found_temp_charger_ || found_charger_) && rcon_value & (RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT)) {
			rcon_status_ = 0;
			return get_trig_(rcon_value);
		}
		else{
			rcon_status_ = 0;
			return 0;
		}
	}
	else if (mt.is_go_to_charger()) {
		auto status = rcon_value;
		rcon_status_ = 0;
		return status;
	}

	return 0;*/
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
	resetStatus();
	return rcon_status;
}

uint32_t Rcon::getWFRcon()
{
	uint32_t rcon_status = getStatus() & (RconFL2_HomeT|RconFR_HomeT|RconFL_HomeT|RconFR2_HomeT);
	resetStatus();
	return rcon_status;
}

