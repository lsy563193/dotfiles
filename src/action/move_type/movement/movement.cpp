//
// Created by lsy563193 on 12/5/17.
//

#include <action.hpp>
#include <movement.hpp>
#include <move_type.hpp>

#include "wheel.hpp"
IMoveType* IMovement::sp_mt_  = nullptr;

//Point32_t IMovement::s_target_p = {0,0};
//Point32_t IMovement::s_start_p = {0,0};

float IMovement::s_pos_x = 0;
float IMovement::s_pos_y = 0;
//Cells IMovement::path_ = {};

void IMovement::run() {
//	PP_INFO();
	int32_t l_speed{},r_speed{};
	adjustSpeed(l_speed,r_speed);
	wheel.setPidTargetSpeed(l_speed, r_speed);
}


