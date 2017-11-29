//
// Created by root on 11/20/17.
//

#include "pp.h"
#include "wheel.h"

struct pid_argu_struct argu_for_pid = {REG_TYPE_NONE,0,0,0};
struct pid_struct left_pid = {0,0,0,0,0,0,0,0}, right_pid = {0,0,0,0,0,0,0,0};
ros::Time g_lw_t, g_rw_t; // these variable is used for calculate wheel step

Wheel wheel;

int32_t Wheel::get_right_step(void)
{
	double t, step;
	double rwsp;
	if (right_speed < 0)
		rwsp = (double) right_speed * -1;
	else
		rwsp = (double) right_speed;
	t = (ros::Time::now() - g_rw_t).toSec();
	step = rwsp * t / 0.12;//origin 0.181
	right_step = (uint32_t) step;
	return right_step;
}

int32_t Wheel::get_left_step(void)
{
	double t, step;
	double lwsp;
	if (left_speed < 0)
		lwsp = (double) left_speed * -1;
	else
		lwsp = (double) left_speed;
	t = (double) (ros::Time::now() - g_lw_t).toSec();
	step = lwsp * t / 0.12;//origin 0.181
	left_step = (uint32_t) step;
	return left_step;
}

void Wheel::reset_step(void)
{
	g_lw_t = ros::Time::now();
	g_rw_t = ros::Time::now();
	right_step = 0;
	left_step = 0;
}

void Wheel::set_dir_backward(void)
{
	left_direction = BACKWARD;
	right_direction = BACKWARD;
	//ROS_INFO("%s %d: dir left(%d), dir right(%d).", __FUNCTION__, __LINE__, left_direction, right_direction);
}

void Wheel::set_dir_forward(void)
{
	//ROS_INFO("%s %d", __FUNCTION__, __LINE__);
	left_direction = FORWARD;
	right_direction = FORWARD;
}

void Wheel::set_pid_param(uint8_t reg_type, float Kp, float Ki, float Kd)
{
	boost::mutex::scoped_lock(pid_lock);
	argu_for_pid.reg_type = reg_type;
	argu_for_pid.Kp = Kp;
	argu_for_pid.Ki = Ki;
	argu_for_pid.Kd = Kd;
}

void Wheel::set_pid(void)
{
	boost::mutex::scoped_lock(pid_lock);
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

	/*---update status---*/
	right_pid.last_target_speed = right_pid.target_speed;
	right_pid.delta_last = right_pid.delta;
#else
	if (argu_for_pid.reg_type != REG_TYPE_NONE && (left_pid.last_reg_type != argu_for_pid.reg_type || right_pid.last_reg_type != argu_for_pid.reg_type))
	{
#if 1
		if(argu_for_pid.reg_type == REG_TYPE_BACK)
		{
			/*---brake when turn to back regulator---*/
			left_pid.actual_speed = 0;
			right_pid.actual_speed = 0;
		}
		else
#endif
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
		if(fabsf(left_pid.actual_speed) <= 6)
		{
			if(left_pid.target_speed > 0)
				left_pid.actual_speed = (left_pid.target_speed > 7) ? 7 : left_pid.target_speed;
			else
				left_pid.actual_speed = (left_pid.target_speed < -7) ? -7 : left_pid.target_speed;
		}
		else if(fabsf(left_pid.actual_speed) <= 15)
		{
			// For low actual speed cases.
			left_pid.delta = left_pid.target_speed - left_pid.actual_speed;
			if(left_pid.delta > 0)
				left_pid.actual_speed += left_pid.actual_speed > 0 ? 1 : 0.5;
			else if(left_pid.delta < 0)
				left_pid.actual_speed -= left_pid.actual_speed < 0 ? 1 : 0.5;
		}
		else if (fabsf(left_pid.target_speed) <= 10)
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

		if(fabsf(right_pid.actual_speed) <= 6)
		{
			if(right_pid.target_speed > 0)
				right_pid.actual_speed = (right_pid.target_speed > 7) ? 7 : right_pid.target_speed;
			else
				right_pid.actual_speed = (right_pid.target_speed < -7) ? -7 : right_pid.target_speed;
		}
		else if(fabsf(right_pid.actual_speed) <= 15)
		{
			// For low actual speed cases.
			right_pid.delta = right_pid.target_speed - right_pid.actual_speed;
			if(right_pid.delta > 0)
				right_pid.actual_speed += right_pid.actual_speed > 0 ? 1 : 0.5;
			else if(right_pid.delta < 0)
				right_pid.actual_speed -= right_pid.actual_speed < 0 ? 1 : 0.5;
		}
		else if (fabsf(right_pid.target_speed) <= 10)
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
	//ROS_INFO("%s %d: real speed: %f, %f, target speed: %f, %f, reg_type: %d.", __FUNCTION__, __LINE__, left_pid.actual_speed, right_pid.actual_speed, left_pid.target_speed, right_pid.target_speed, argu_for_pid.reg_type);

	/*---update status---*/
	left_pid.last_target_speed = left_pid.target_speed;
	right_pid.last_target_speed = right_pid.target_speed;
#endif
}
void Wheel::set_speed(uint8_t Left, uint8_t Right, uint8_t reg_type, float PID_p, float PID_i, float PID_d)
{
	int8_t signed_left_speed = (int8_t)Left;
	int8_t signed_right_speed = (int8_t)Right;
	set_pid_param(reg_type, PID_p, PID_i, PID_d);
	//ROS_INFO("%s %d: Signed speed: left(%d), right(%d), dir left(%d), dir right(%d).",
	//		 __FUNCTION__, __LINE__, signed_left_speed, signed_right_speed, left_direction, right_direction);

	if(left_direction == BACKWARD)
		signed_left_speed *= -1;
	if(right_direction == BACKWARD)
		signed_right_speed *= -1;
	left_pid.target_speed = (float)signed_left_speed;
	right_pid.target_speed = (float)signed_right_speed;
	//ROS_INFO("%s %d: PID Target speed: left(%f), right(%f).", __FUNCTION__, __LINE__, left_pid.target_speed, right_pid.target_speed);
}

void Wheel::set_left_speed(float speed)
{
	left_speed = speed;
	uint16_t stream_l_speed;
	speed = speed > RUN_TOP_SPEED ? RUN_TOP_SPEED : speed;
	stream_l_speed = (uint16_t)(fabs(speed * SPEED_ALF));
	if (speed < 0)
		stream_l_speed |= 0x8000;
	serial.setSendData(CTL_WHEEL_LEFT_HIGH, (stream_l_speed >> 8) & 0xff);
	serial.setSendData(CTL_WHEEL_LEFT_LOW, stream_l_speed & 0xff);
}

void Wheel::set_right_speed(float speed)
{
	right_speed = speed;
	uint16_t stream_r_speed;
	speed = speed > RUN_TOP_SPEED ? RUN_TOP_SPEED : speed;
	stream_r_speed = (uint16_t)(fabs(speed * SPEED_ALF));
	if (right_direction == BACKWARD)
		stream_r_speed |= 0x8000;
	serial.setSendData(CTL_WHEEL_RIGHT_HIGH, (stream_r_speed >> 8) & 0xff);
	serial.setSendData(CTL_WHEEL_RIGHT_LOW, stream_r_speed & 0xff);
}

int16_t Wheel::get_left_speed(void)
{
	return left_speed;
}

int16_t Wheel::get_right_speed(void)
{
	return right_speed;
}

void Wheel::move_forward(uint8_t Left_Speed, uint8_t Right_Speed)
{
	set_dir_forward();
	set_speed(Left_Speed, Right_Speed);
}

void Wheel::set_dir_left(void)
{
	//ROS_INFO("%s %d", __FUNCTION__, __LINE__);
	set_direction_flag(Direction_Flag_Left);
	left_direction = BACKWARD;
	right_direction = FORWARD;
}

void Wheel::set_dir_right(void)
{
	//ROS_INFO("%s %d", __FUNCTION__, __LINE__);
	set_direction_flag(Direction_Flag_Right);
	left_direction = FORWARD;
	right_direction = BACKWARD;
}
