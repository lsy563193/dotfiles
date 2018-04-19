//
// Created by austin on 18-3-13.
//

#ifndef PP_WATER_TANK_HPP
#define PP_WATER_TANK_HPP

#include <cstdint>
#include <config.h>

class WaterTank
{
public:
	WaterTank();

	enum operate_option{
		swing_motor = 0,
		pump,
		swing_motor_and_pump,
	};

	void setStatus(int _operate_option,bool status);

	bool getStatus(int _operate_option);

	void open(int _operate_option);

	void stop(int operate_option);

	void updatePWM();

	// Swing motor
	enum swing_motor_mode{
		SWING_MOTOR_LOW = 0,
		SWING_MOTOR_HIGH,
	};
	// For checking whether robot is carrying a water tank.
	bool checkEquipment();

	void slowOperateSwingMotor();

	void fullOperateSwingMotor();

	void setSwingMotorCurrent(uint16_t current)
	{
		swing_motor_current_ = current;
	}

	uint16_t getSwingMotorCurrent()
	{
		return swing_motor_current_;
	}

	void setSwingMotorEquipmentStatus(bool val){
		is_swing_motor_equipped_ = val;
	}

	bool getSwingMotorEquipmentStatus(){
		return is_swing_motor_equipped_;
	}

	void setSwingMotorPWM();

	void setUserSwingMotorMode(int mode);

	int getUserSetSwingMotorMode()
	{
		return user_set_swing_motor_mode_;
	}

	void setCurrentSwingMotorMode(int mode);

	int getCurrentSwingMotorMode()
	{
		return current_swing_motor_mode_;
	}

	// Pump
	enum pump_mode{
		PUMP_LOW=0,
		PUMP_MIDDLE,
		PUMP_HIGH,
	};

	void slowOperatePump();

	void fullOperatePump();

	void setCurrentPumpMode(uint8_t mode);

	void setUserSetPumpMode(uint8_t mode);

	int getUserSetPumpMode()
	{
		return user_set_pump_mode_;
	}

private:

	// Real time status for water tank and pump.
	bool is_swing_motor_equipped_{false};
	// Indicates whether swing motor is on.
	bool swing_motor_switch_{false};
	// Indicates whether pump is on.
	bool pump_switch_{false};

	double check_battery_time_stamp_{0};

	// Swing motor
	uint16_t swing_motor_current_{0};
	int swing_motor_operate_voltage_{FULL_OPERATE_VOLTAGE_FOR_SWING_MOTOR};
	uint8_t swing_motor_pwm_{0};
	bool is_swing_motor_mode_change_{false};
	int user_set_swing_motor_mode_{SWING_MOTOR_LOW};
	int current_swing_motor_mode_{SWING_MOTOR_LOW};

	// Pump
	double last_pump_time_stamp_{};
	double pump_time_interval_;
	uint8_t pump_cnt_{};
	uint8_t pump_max_cnt_{3};
	uint8_t pump_pwm_{};
	int user_set_pump_mode_{PUMP_LOW};
	int current_pump_mode_{PUMP_LOW};

};

extern WaterTank water_tank;
#endif //PP_WATER_TANK_HPP
