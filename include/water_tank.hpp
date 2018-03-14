//
// Created by austin on 18-3-13.
//

#ifndef PP_WATER_TANK_HPP
#define PP_WATER_TANK_HPP

#include <cstdint>

class WaterTank
{
public:
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

private:
	uint16_t current_{0};

	// This variable indicates whether robot is booted with a water tank.
	bool is_equipped_{false};

	// Real time status for water tank.
	bool status_{false};
};

extern WaterTank water_tank;
#endif //PP_WATER_TANK_HPP
