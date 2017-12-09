//
// Created by lsy563193 on 12/5/17.
//

//#include "pp.h"
#include <arch.hpp>
#include "wheel.hpp"
ACleanMode *IMovement::sp_cm_ = nullptr;
Point32_t IMovement::s_target_p = {0,0};
Point32_t IMovement::s_origin_p = {0,0};
float IMovement::s_pos_x = 0;
float IMovement::s_pos_y = 0;
Path_t IMovement::path_ = {};

void IMovement::run() {
//	PP_INFO();
	int32_t l_speed,r_speed;
	adjustSpeed(l_speed,r_speed);
	wheel.setPidTargetSpeed(l_speed, r_speed);
}