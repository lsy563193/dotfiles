//
// Created by root on 11/20/17.
//

#include "ros/ros.h"
#include "boost/thread.hpp"
#include "config.h"
#include "serial.h"
#include "wheel.hpp"

Wheel wheel;

void Wheel::stop(void)
{
	setPidTargetSpeed(0, 0);
}

uint32_t Wheel::getRightStep(void)
{
	auto delta_t = (ros::Time::now() - right_wheel_step_reset_time_).toSec();
	right_wheel_step_ = static_cast<uint32_t>(getRightWheelActualSpeed() * delta_t / 0.12);
	return right_wheel_step_;
}

uint32_t Wheel::getLeftStep(void)
{
	auto delta_t = (ros::Time::now() - left_wheel_step_reset_time_).toSec();
	left_wheel_step_ = static_cast<uint32_t>(getLeftWheelActualSpeed() * delta_t / 0.12);
	return left_wheel_step_;
}

void Wheel::resetStep(void)
{
	left_wheel_step_reset_time_ = ros::Time::now();
	right_wheel_step_reset_time_ = ros::Time::now();
	right_wheel_step_ = 0;
	left_wheel_step_ = 0;
}

void Wheel::setDirectionBackward(void)
{
	left_direction_ = DIRECTION_BACKWARD;
	right_direction_ = DIRECTION_BACKWARD;
	//ROS_INFO("%s %d: dir left(%d), dir right(%d).", __FUNCTION__, __LINE__, left_direction_, right_direction_);
}

void Wheel::setDirectionForward(void)
{
	//ROS_INFO("%s %d", __FUNCTION__, __LINE__);
	left_direction_ = DIRECTION_FORWARD;
	right_direction_ = DIRECTION_FORWARD;
}

void Wheel::setDirectionLeft(void)
{
	//ROS_INFO("%s %d", __FUNCTION__, __LINE__);
	left_direction_ = DIRECTION_BACKWARD;
	right_direction_ = DIRECTION_FORWARD;
}

void Wheel::setDirectionRight(void)
{
	//ROS_INFO("%s %d", __FUNCTION__, __LINE__);
	left_direction_ = DIRECTION_FORWARD;
	right_direction_ = DIRECTION_BACKWARD;
}

DirectionType Wheel::getDirection(void)
{
	if (left_direction_ == DIRECTION_FORWARD)
	{
		if (right_direction_ == DIRECTION_FORWARD)
			return DIRECTION_FORWARD;
		else
			return DIRECTION_RIGHT;
	}
	else
	{
		if (right_direction_ == DIRECTION_FORWARD)
			return DIRECTION_LEFT;
		else
			return DIRECTION_BACKWARD;
	}
}

void Wheel::setPidParam(uint8_t reg_type, float Kp, float Ki, float Kd)
{
	boost::mutex::scoped_lock lock(pid_lock);
	argu_for_pid.reg_type = reg_type;
	argu_for_pid.Kp = Kp;
	argu_for_pid.Ki = Ki;
	argu_for_pid.Kd = Kd;
}

void Wheel::pidAdjustSpeed(void)
{
	boost::mutex::scoped_lock lock(pid_lock);
#if 0
	left_pid.delta = left_pid.target_speed - left_pid.actual_speed;
	/*---target speed changed, reset err_sum---*/
	if(left_pid.last_target_speed != left_pid.target_speed)
		left_pid.delta_sum = 0;
	left_pid.delta_sum += left_pid.delta;

	/*---pid---*/
	left_pid.variation = argu_for_pid.Kp*left_pid.delta + argu_for_pid.Ki*left_pid.delta_sum + argu_for_pid.Kd*(left_pid.delta - left_pid.delta_last);
	left_pid.actual_speed += left_pid.variation;

	/*---update status---*/
	left_pid.last_target_speed = left_pid.target_speed;
	left_pid.delta_last = left_pid.delta;

	right_pid.delta = right_pid.target_speed - right_pid.actual_speed;
	/*---target speed changed, reset err_sum---*/
	if(right_pid.last_target_speed != right_pid.target_speed)
		right_pid.delta_sum = 0;
	right_pid.delta_sum += right_pid.delta;

	/*---pid---*/
	right_pid.variation = argu_for_pid.Kp*right_pid.delta + argu_for_pid.Ki*right_pid.delta_sum + argu_for_pid.Kd*(right_pid.delta - right_pid.delta_last);
	right_pid.actual_speed += right_pid.variation;

	/*---init status---*/
	right_pid.last_target_speed = right_pid.target_speed;
	right_pid.delta_last = right_pid.delta;
#else
	if (argu_for_pid.reg_type != REG_TYPE_NONE && (left_pid.last_reg_type != argu_for_pid.reg_type || right_pid.last_reg_type != argu_for_pid.reg_type))
	{
		if(argu_for_pid.reg_type == REG_TYPE_BACK)
		{
			/*---brake when turn to back regulator---*/
			left_pid.actual_speed = 0;
			right_pid.actual_speed = 0;
		}
		else
		{
			//ROS_INFO("%s %d: Slowly reset the speed to zero.", __FUNCTION__, __LINE__);
			if (left_pid.actual_speed < 0)
				left_pid.actual_speed -= static_cast<int8_t>(left_pid.actual_speed) >= -6 ? left_pid.actual_speed : floor(left_pid.actual_speed / 10.0);
			else if (left_pid.actual_speed > 0)
				left_pid.actual_speed -= static_cast<int8_t>(left_pid.actual_speed) <= 6 ? left_pid.actual_speed : ceil(left_pid.actual_speed / 10.0);
			if (right_pid.actual_speed < 0)
				right_pid.actual_speed -= static_cast<int8_t>(right_pid.actual_speed) >= -6 ? right_pid.actual_speed : floor(right_pid.actual_speed / 10.0);
			else if (right_pid.actual_speed > 0)
				right_pid.actual_speed -= static_cast<int8_t>(right_pid.actual_speed) <= 6 ? right_pid.actual_speed : ceil(right_pid.actual_speed / 10.0);
		}

		if (left_pid.actual_speed == 0 || right_pid.actual_speed == 0)
		{
			left_pid.actual_speed = 0;
			right_pid.actual_speed = 0;
			left_pid.last_reg_type = right_pid.last_reg_type = argu_for_pid.reg_type;
			//ROS_INFO("%s %d: Switch PID type to %d.", __FUNCTION__, __LINE__, argu_for_pid.reg_type);
		}
	}
	else if(argu_for_pid.reg_type == REG_TYPE_NONE || argu_for_pid.reg_type == REG_TYPE_WALLFOLLOW)
	{
		left_pid.actual_speed = left_pid.target_speed;
		right_pid.actual_speed = right_pid.target_speed;
		left_pid.last_reg_type = right_pid.last_reg_type = argu_for_pid.reg_type;
	}
	else
	{
	#if 0
		/*---if one of the wheels should change direction, set both target_speed to 0 first---*/
		if((left_pid.actual_speed * left_pid.target_speed < 0) || (right_pid.actual_speed * right_pid.target_speed < 0))
		{
			left_pid.target_speed = 0;
			right_pid.target_speed = 0;
		}
		left_pid.variation = left_pid.target_speed - left_pid.actual_speed;
		right_pid.variation = right_pid.target_speed - right_pid.actual_speed;
		/*---set variation limit---*/
		float variation_limit = 0;
		if(argu_for_pid.reg_type == REG_TYPE_LINEAR)
			variation_limit = 1;
		else if(argu_for_pid.reg_type == REG_TYPE_CURVE)
			variation_limit = 8;
		else if(argu_for_pid.reg_type == REG_TYPE_TURN)
			variation_limit = 4;
		else if(argu_for_pid.reg_type == REG_TYPE_BACK)
			variation_limit = 20;
		/*---adjust speed---*/
		if(left_pid.variation > variation_limit)left_pid.variation = variation_limit;
		else if(left_pid.variation < -variation_limit)left_pid.variation = -variation_limit;
		if(right_pid.variation > variation_limit)right_pid.variation = variation_limit;
		else if(right_pid.variation < -variation_limit)right_pid.variation = -variation_limit;

		left_pid.actual_speed += left_pid.variation;
		right_pid.actual_speed += right_pid.variation;
	#endif
		if(std::abs(left_pid.actual_speed) <= 6)
		{
			if(left_pid.target_speed > 0)
				left_pid.actual_speed = (left_pid.target_speed > 7) ? 7 : left_pid.target_speed;
			else
				left_pid.actual_speed = (left_pid.target_speed < -7) ? -7 : left_pid.target_speed;
		}
		else if(std::abs(left_pid.actual_speed) <= 15)
		{
			// For low actual speed cases.
			left_pid.delta = left_pid.target_speed - left_pid.actual_speed;
			if(left_pid.delta > 0)
				left_pid.actual_speed += left_pid.actual_speed > 0 ? 1 : 0.5;
			else if(left_pid.delta < 0)
				left_pid.actual_speed -= left_pid.actual_speed < 0 ? 1 : 0.5;
		}
		else if (std::abs(left_pid.target_speed) <= 10)
		{
			// For high actual speed cases.
			left_pid.delta = left_pid.target_speed - left_pid.actual_speed;
			if(left_pid.delta > 0)
				left_pid.actual_speed += 4;
			else if(left_pid.delta < 0)
				left_pid.actual_speed -= 4;
		}
		else
			left_pid.actual_speed = left_pid.target_speed;

		if(std::abs(right_pid.actual_speed) <= 6)
		{
			if(right_pid.target_speed > 0)
				right_pid.actual_speed = (right_pid.target_speed > 7) ? 7 : right_pid.target_speed;
			else
				right_pid.actual_speed = (right_pid.target_speed < -7) ? -7 : right_pid.target_speed;
		}
		else if(std::abs(right_pid.actual_speed) <= 15)
		{
			// For low actual speed cases.
			right_pid.delta = right_pid.target_speed - right_pid.actual_speed;
			if(right_pid.delta > 0)
				right_pid.actual_speed += right_pid.actual_speed > 0 ? 1 : 0.5;
			else if(right_pid.delta < 0)
				right_pid.actual_speed -= right_pid.actual_speed < 0 ? 1 : 0.5;
		}
		else if (std::abs(right_pid.target_speed) <= 10)
		{
			// For high actual speed cases.
			right_pid.delta = right_pid.target_speed - right_pid.actual_speed;
			if(right_pid.delta > 0)
				right_pid.actual_speed += 4;
			else if(right_pid.delta < 0)
				right_pid.actual_speed -= 4;
		}
		else
			right_pid.actual_speed = right_pid.target_speed;

		if(left_pid.actual_speed > RUN_TOP_SPEED)left_pid.actual_speed = (int8_t)RUN_TOP_SPEED;
		else if(left_pid.actual_speed < -RUN_TOP_SPEED)left_pid.actual_speed = -(int8_t)RUN_TOP_SPEED;
		if(right_pid.actual_speed > RUN_TOP_SPEED)right_pid.actual_speed = (int8_t)RUN_TOP_SPEED;
		else if(right_pid.actual_speed < -RUN_TOP_SPEED)right_pid.actual_speed = -(int8_t)RUN_TOP_SPEED;
	}
//	ROS_INFO("%s %d: real speed: %f, %f, target speed: %f, %f, reg_type: %d.", __FUNCTION__, __LINE__, left_pid.actual_speed, right_pid.actual_speed, left_pid.target_speed, right_pid.target_speed, argu_for_pid.reg_type);

	/*---init status---*/
	left_pid.last_target_speed = left_pid.target_speed;
	right_pid.last_target_speed = right_pid.target_speed;
	pidSetLeftSpeed(left_pid.actual_speed);
	pidSetRightSpeed(right_pid.actual_speed);
#endif
}

void Wheel::setPidTargetSpeed(uint8_t Left, uint8_t Right, uint8_t reg_type, float PID_p, float PID_i, float PID_d)
{
	int8_t signed_left_speed_after_pid_ = (int8_t)Left;
	int8_t signed_right_speed_after_pid_ = (int8_t)Right;
	setPidParam(reg_type, PID_p, PID_i, PID_d);
	//ROS_INFO("%s %d: Signed speed: left(%d), right(%d), dir left(%d), dir right(%d).",
	//		 __FUNCTION__, __LINE__, signed_left_speed_after_pid_, signed_right_speed_after_pid_, left_direction_, right_direction_);

	if(left_direction_ == DIRECTION_BACKWARD)
		signed_left_speed_after_pid_ *= -1;
	if(right_direction_ == DIRECTION_BACKWARD)
		signed_right_speed_after_pid_ *= -1;
	left_pid.target_speed = (float)signed_left_speed_after_pid_;
	right_pid.target_speed = (float)signed_right_speed_after_pid_;
	//ROS_INFO("%s %d: PID Target speed: left(%f), right(%f).", __FUNCTION__, __LINE__, left_pid.target_speed, right_pid.target_speed);
}

void Wheel::pidSetLeftSpeed(float speed)
{
	left_speed_after_pid_ = static_cast<int8_t>(speed);
	int16_t stream_speed;
	if (speed >= 0)
		speed = speed > RUN_TOP_SPEED ? RUN_TOP_SPEED : speed;
	else
		speed = std::abs(speed) > RUN_TOP_SPEED ? -1 * RUN_TOP_SPEED : speed;
	stream_speed = static_cast<int16_t>(speed * SPEED_ALF);

	serial.setSendData(CTL_WHEEL_LEFT_HIGH, (stream_speed >> 8) & 0xff);
	serial.setSendData(CTL_WHEEL_LEFT_LOW, stream_speed & 0xff);
}

void Wheel::pidSetRightSpeed(float speed)
{
	right_speed_after_pid_ = static_cast<int8_t>(speed);
	int16_t stream_speed;
	if (speed >= 0)
		speed = speed > RUN_TOP_SPEED ? RUN_TOP_SPEED : speed;
	else
		speed = std::abs(speed) > RUN_TOP_SPEED ? -1 * RUN_TOP_SPEED : speed;
	stream_speed = static_cast<int16_t>(speed * SPEED_ALF);

	serial.setSendData(CTL_WHEEL_RIGHT_HIGH, (stream_speed >> 8) & 0xff);
	serial.setSendData(CTL_WHEEL_RIGHT_LOW, stream_speed & 0xff);
}

int8_t Wheel::getLeftSpeedAfterPid(void)
{
	return left_speed_after_pid_;
}

int8_t Wheel::getRightSpeedAfterPid(void)
{
	return right_speed_after_pid_;
}

void Wheel::moveForward(uint8_t Left_Speed, uint8_t Right_Speed)
{
	setDirectionForward();
	setPidTargetSpeed(Left_Speed, Right_Speed);
}
