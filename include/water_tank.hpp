//
// Created by austin on 18-3-13.
//

#ifndef PP_WATER_TANK_HPP
#define PP_WATER_TANK_HPP

#include <cstdint>

class WaterTank
{
public:
	void setCurrent(uint16_t current)
	{
		current_ = current;
	}

	uint16_t getCurrent()
	{
		return current_;
	}

private:
	uint16_t current_;
};

extern WaterTank water_tank;
#endif //PP_WATER_TANK_HPP
