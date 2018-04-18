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
	enum pump_mode{
		PUMP_LOW=0,
		PUMP_MID,
		PUMP_HIGH,
	};
	enum swing_motor_mode{
		SWING_MOTOR_LOW = 0,
		SWING_MOTOR_HIGH,
	};
	// For checking whether robot is carrying a water tank.
	bool checkEquipment();

	void setStatus(int equiment,bool status)
	{
		if(equiment == swing_motor)
			swing_motor_switch_ = status;
		else if(equiment == pump)
			pump_switch_ = status;
	}

	bool getStatus(int equiment)
	{
		if(equiment == swing_motor)
			return swing_motor_switch_;
		else if(equiment == pump)
			return pump_switch_;
	}

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

	void open(int equipment);

	void stop(int operate_option);

	void setWaterTankPWM();

	void updatePWM();

	void setPumpMode(uint8_t mode);
	void setSwingMotorMode(uint8_t mode);
	int getSwingMotorMode();

	uint8_t getPumpMode()
	{
		return pump_mode_;
	}

	
private:

	uint16_t swing_motor_current_{};

	// Real time status for water tank and pump.
	bool is_swing_motor_equipped_{};
	// Indicates whether swing motor is on.
	bool swing_motor_switch_{false};
	// Indicates whether pump is on.
	bool pump_switch_{false};

	bool is_swing_motor_mode_change_{};

	int swing_motor_operate_voltage_{FULL_OPERATE_VOLTAGE_FOR_SWING_MOTOR};
	uint8_t swing_motor_pwm_{};
	double check_battery_time_stamp_{};
	double last_pump_time_stamp_{};
	double pump_time_interval_;
	uint8_t pump_cnt_{};
	uint8_t pump_max_cnt_{3};
	uint8_t pump_pwm_{};

	uint8_t pump_mode_;
};

extern WaterTank water_tank;
#endif //PP_WATER_TANK_HPP
