//
// Created by root on 11/20/17.
//

#ifndef PP_WHEEL_H
#define PP_WHEEL_H

#include <pp/x900sensor.h>
extern pp::x900sensor sensor;

class Wheel {
public:
	Wheel() {
		left_direction = FORWARD;
		right_direction = FORWARD;
		left_speed = 0;
		right_speed = 0;
		left_step = 0;
		right_step = 0;
	};

	void stop(void)
	{
		set_speed(0, 0);
	}
	int32_t get_right_step(void);

	int32_t get_left_step(void);

	void reset_step(void);

	void set_dir_backward(void);

	void set_dir_forward(void);

	void set_pid_param(uint8_t reg_type, float Kp, float Ki, float Kd);

	void set_pid(void);

	void set_speed(uint8_t Left, uint8_t Right, uint8_t reg_type = REG_TYPE_NONE, float PID_p = 1, float PID_i = 0, float PID_d = 0);

	void set_left_speed(uint8_t speed);

	void set_right_speed(uint8_t speed);

	int16_t get_left_speed(void);

	int16_t get_right_speed(void);

	void move_forward(uint8_t Left_Speed, uint8_t Right_Speed);

	void set_dir_left(void);

	void set_dir_right(void);

	void set_left_dir(int dir)
	{
		left_direction = dir;
	}

	void set_right_dir(int dir)
	{
		right_direction = dir;
	}

	uint8_t get_dir_left(void)
	{
		return left_direction;
	}

	uint8_t get_dir_right(void)
	{
		return right_direction;
	}

uint8_t get_direction_flag(void)
{
	return g_direction_flag;
}

void set_direction_flag(uint8_t flag)
{
	g_direction_flag = flag;
}

	float getLwheelCurrent() const
	{
		return sensor.lw_crt;
	}

	float getRwheelCurrent() const
	{
		return sensor.rw_crt;
	}

	float getLeftWheelSpeed() const
	{
		return sensor.lw_vel;
	}
	float getRightWheelSpeed() const
	{
		return sensor.rw_vel;
	}
private:
	uint8_t left_direction;
	uint8_t right_direction;
	int16_t left_speed;
	int16_t right_speed;
	int32_t left_step;
	int32_t right_step;
	uint8_t g_direction_flag = 0;
};

extern Wheel wheel;

#endif //PP_WHEEL_H
