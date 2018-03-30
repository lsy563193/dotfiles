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
	enum{
		water_tank = 0,
		pump,
		tank_pump,
	};
	// For checking whether robot is carrying a water tank.
	bool checkEquipment(bool is_stop_water_tank);

	void setStatus(int equiment,bool status)
	{
		if(equiment == water_tank)
			water_tank_status_ = status;
		else if(equiment == pump)
			pump_status_ = status;
		else if(equiment == tank_pump)
			tank_pump_status_ = status;
	}

	bool getStatus(int equiment)
	{
		if(equiment == water_tank)
			return water_tank_status_;
		else if(equiment == pump)
			return pump_status_;
		else if(equiment == tank_pump)
			return tank_pump_status_;
	}

	void setCurrent(uint16_t current)
	{
		current_ = current;
	}

	uint16_t getCurrent()
	{
		return current_;
	}

	bool getEquimentStatus(){
		return is_water_tank_equiment_;
	}
	void setEquimentStatus(bool val){
		is_water_tank_equiment_ = val;
	}
	void open(int equipment);

	void stop(int equipment);

	void setWaterTankPWM();

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

	// Real time status for water tank and pump.
	bool is_water_tank_equiment_{false};

	bool water_tank_status_{false};
	bool pump_status_{false};
	bool tank_pump_status_{false};

	uint8_t water_tank_pwm_{0};
	double check_battery_time_stamp_{0};
	double last_pump_time_stamp_{0};
	double pump_time_interval_;
	uint8_t pump_cnt_{0};
	uint8_t pump_max_cnt_{3};
	uint8_t pump_pwm_{0x00};

	uint8_t mode_;
};

extern WaterTank water_tank;
#endif //PP_WATER_TANK_HPP
