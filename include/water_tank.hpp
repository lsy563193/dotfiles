//
// Created by austin on 18-3-13.
//

#ifndef PP_WATER_TANK_HPP
#define PP_WATER_TANK_HPP

#include <cstdint>

class WaterTank
{
public:
	WaterTank();

	// For checking whether robot is carrying a water tank.
	bool checkEquipment();

	bool isEquipped()
	{
		return is_equipped_;
	}

	void setStatus(bool status)
	{
		status_ = status;
	}

	bool getStatus()
	{
		return status_;
	}

	void setCurrent(uint16_t current)
	{
		current_ = current;
	}

	uint16_t getCurrent()
	{
		return current_;
	}

	void normalOperate();

	void stop();

	void checkBatterySetPWM();

	void updatePWM();

	void setMode(uint8_t mode);

	uint8_t getMode()
	{
		return mode_;
	}

	enum{
		PUMP_LOW,
		PUMP_MID,
		PUMP_HIGH,
	};

private:
	uint16_t current_{0};

	// This variable indicates whether robot is booted with a water tank.
	bool is_equipped_{false};

	// Real time status for water tank.
	bool status_{false};

	bool operation_{false};

	uint8_t pwm_{0};

	double check_battery_time_stamp_{0};
	double last_pump_time_stamp_{0};
	double pump_time_interval_;
	uint8_t pump_cnt_{0};
	uint8_t pump_max_cnt_{3};
	uint8_t pump_switch_{0x00};

	uint8_t mode_;
};

extern WaterTank water_tank;
#endif //PP_WATER_TANK_HPP
