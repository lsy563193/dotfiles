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
	ROS_WARN("%s %d: Generate movementGyroDynamic",__FUNCTION__,__LINE__);
}

MovementGyroDynamic::~MovementGyroDynamic() {
	auto p_mode = dynamic_cast<ACleanMode*> (sp_mt_->sp_mode_);
	p_mode->time_gyro_dynamic_ = ros::Time::now().toSec();
	gyro.setDynamicOff();
	ROS_WARN("%s %d: Delete movementGyroDynamic",__FUNCTION__,__LINE__);
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
//#if GYRO_DYNAMIC_ADJUSTMENT
	return is_open_dynamic_succeed_ && ros::Time::now().toSec() - start_dynamic_time_ > robot::instance()->getGyroDynamicRunTime() || sp_mt_->isFinishForward();
//#else
//	return true;
//#endif
}
