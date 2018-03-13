//
// Created by root on 11/20/17.
//

#ifndef PP_WHEEL_HPP
#define PP_WHEEL_HPP

//regulator type
#define REG_TYPE_NONE			0
#define REG_TYPE_WALLFOLLOW		1
#define REG_TYPE_LINEAR			2
#define REG_TYPE_TURN			3
#define REG_TYPE_BACK			4
#define REG_TYPE_CURVE			5

#include "ros/ros.h"
#define Wheel_Stall_Limit			((uint16_t) 750)//origin 580

struct pid_struct
{
	float delta;
	float delta_sum;
	float delta_last;
	float target_speed;
	float actual_speed;
	float last_target_speed;
	uint8_t last_reg_type;
	float variation;
};

struct pid_argu_struct
{
	uint8_t reg_type; // ISpeedGovernor type
	float Kp;
	float Ki;
	float Kd;
};

// for wheel direction
typedef enum{
	DIRECTION_FORWARD,
	DIRECTION_BACKWARD,
	DIRECTION_LEFT,
	DIRECTION_RIGHT
} DirectionType;

class Wheel {
public:
	Wheel() {
		left_direction_ = DIRECTION_FORWARD;
		right_direction_ = DIRECTION_FORWARD;
		left_speed_after_pid_ = 0;
		right_speed_after_pid_ = 0;
		left_wheel_step_ = 0;
		right_wheel_step_ = 0;
		is_left_wheel_over_current_ = false;
		is_right_wheel_over_current_= false;
		left_wheel_actual_speed_ = 0;
		right_wheel_actual_speed_ = 0;
		left_wheel_cliff_ = false;
		right_wheel_cliff_ = false;
	};

	void stop(void);

	uint32_t getRightStep(void);

	uint32_t getLeftStep(void);

	void resetStep(void);

	void setDirectionBackward(void);

	void setDirectionForward(void);

	void setDirectionLeft(void);

	void setDirectionRight(void);

	DirectionType getDirection(void);

	void setPidParam(uint8_t reg_type, float Kp, float Ki, float Kd);

	void pidAdjustSpeed(void);

	void setPidTargetSpeed(uint8_t Left, uint8_t Right, uint8_t reg_type = REG_TYPE_NONE, float PID_p = 1,
						   float PID_i = 0,
						   float PID_d = 0);

	void pidSetLeftSpeed(float speed);

	void pidSetRightSpeed(float speed);

	int8_t getLeftSpeedAfterPid(void);

	int8_t getRightSpeedAfterPid(void);

	int16_t getLeftSpeedInStream(void);

	int16_t getRightSpeedInStream(void);

	void moveForward(uint8_t Left_Speed, uint8_t Right_Speed);

	bool getLeftWheelOc() const
	{
		return is_left_wheel_over_current_;
	}

	void setLeftWheelOc(bool lw_oc)
	{
		is_left_wheel_over_current_ = lw_oc;
	}

	bool getRightWheelOc() const
	{
		return is_right_wheel_over_current_;
	}

	void setRightWheelOc(bool rw_oc)
	{
		is_right_wheel_over_current_ = rw_oc;
	}

	void setLeftCurrent(uint16_t current)
	{
		left_current_ = current;
	}

	uint16_t getLeftCurrent()
	{
		return left_current_;
	}

	void setRightCurrent(uint16_t current)
	{
		right_current_ = current;
	}

	uint16_t getRightCurrent()
	{
		return right_current_;
	}

	float getLeftWheelActualSpeed() const
	{
		return left_wheel_actual_speed_;
	}

	void setLeftWheelActualSpeed(float speed)
	{
		left_wheel_actual_speed_ = speed;
	}

	float getRightWheelActualSpeed() const
	{
		return right_wheel_actual_speed_;
	}

	void setRightWheelActualSpeed(float speed)
	{
		right_wheel_actual_speed_ = speed;
	}

	bool getLeftWheelCliffStatus()
	{
		return left_wheel_cliff_;
	}

	void setLeftWheelCliffStatus(bool val)
	{
		left_wheel_cliff_ = val;
	}

	bool getRightWheelCliffStatus()
	{
		return right_wheel_cliff_;
	}

	void setRightWheelCliffStatus(bool val)
	{
		right_wheel_cliff_ = val;
	}

private:
	DirectionType left_direction_;
	DirectionType right_direction_;
	int8_t left_speed_after_pid_;
	int8_t right_speed_after_pid_;
	int16_t left_speed_in_stream_;
	int16_t right_speed_in_stream_;
	uint32_t left_wheel_step_;
	uint32_t right_wheel_step_;

	uint16_t left_current_;
	uint16_t right_current_;
	bool is_left_wheel_over_current_;
	bool is_right_wheel_over_current_;
	float left_wheel_actual_speed_;
	float right_wheel_actual_speed_;

	bool left_wheel_cliff_;
	bool right_wheel_cliff_;
	struct pid_argu_struct argu_for_pid = {REG_TYPE_NONE,0,0,0};
	struct pid_struct left_pid = {0,0,0,0,0,0,0,0}, right_pid = {0,0,0,0,0,0,0,0};
	// These variables are used for calculate wheel step.
	ros::Time left_wheel_step_reset_time_;
	ros::Time right_wheel_step_reset_time_;
	boost::mutex pid_lock;
};

extern Wheel wheel;

#endif //PP_WHEEL_HPP
