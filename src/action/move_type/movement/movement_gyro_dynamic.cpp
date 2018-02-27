//
// Created by pierre on 18-2-8.
//

#include <movement.hpp>
#include <move_type.hpp>
#include <wheel.hpp>
#include <gyro.h>
#include <mode.hpp>
#include <robot.hpp>


MovementGyroDynamic::MovementGyroDynamic()
{
	ROS_INFO("%s,%d,generate movementGyroDynamic",__FUNCTION__,__LINE__);
}

MovementGyroDynamic::~MovementGyroDynamic() {
	ROS_INFO("%s,%d,delete movementGyroDynamic",__FUNCTION__,__LINE__);
}

void MovementGyroDynamic::adjustSpeed(int32_t &l_speed, int32_t &r_speed) {
#if GYRO_DYNAMIC_ADJUSTMENT
	if(!is_open_dynamic_succeed_ && wheel.getLeftWheelActualSpeed() == 0 && wheel.getRightWheelActualSpeed() == 0)
	{
		gyro.setDynamicOn();
		start_dynamic_time_ = ros::Time::now().toSec();
		is_open_dynamic_succeed_ = true;
		ROS_INFO("%s,%d,start gyro dynamic",__FUNCTION__,__LINE__);
	}
#endif
	l_speed = r_speed = 0;
}

bool MovementGyroDynamic::isFinish() {
#if GYRO_DYNAMIC_ADJUSTMENT
	auto p_mode = dynamic_cast<ACleanMode*> (sp_mt_->sp_mode_);
	if(is_open_dynamic_succeed_ && ros::Time::now().toSec() - start_dynamic_time_ > robot::instance()->getGyroDynamicRunTime()){
		p_mode->time_gyro_dynamic_ = ros::Time::now().toSec();
		gyro.setDynamicOff();
		ROS_INFO("%s,%d,shutdown gyro dynamic",__FUNCTION__,__LINE__);
		return true;
	}
#else
	return true;
#endif
	return false;
}
