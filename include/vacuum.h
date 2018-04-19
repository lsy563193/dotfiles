//
// Created by root on 11/17/17.
//

#ifndef PP_VACUUM_H
#define PP_VACUUM_H

#include <cstdint>

class Vacuum
{
public:

	enum VacMode
	{
		vac_speed_max = 100, //15500rpm
		vac_speed_normal = 80, //12000rpm
		vac_speed_low = 50,//8000rpm
	};

	bool isMaxMode() //getter
	{
		return is_max_mode_;//getter
	}

	void setForMaxMode(bool is_max)
	{
		is_max_mode_ = is_max;
	}

	void setSpeedByMode();

	void slowOperate();

	void fullOperate();

	void stop();

	void startExceptionResume();

	void resetExceptionResume();

	void setOc(bool val)
	{
		oc_ = val;
	}

	bool getOc() const
	{
		return oc_;
	}

	void setCurrent(uint16_t current)
	{
		current_ = current;
	}

	uint16_t getCurrent() const
	{
		return current_;
	}

	bool isOn()
	{
		return is_on_;
	}

private:

	bool is_max_mode_{};
	bool oc_;

	uint16_t current_;

	bool is_on_{false};

	void setSpeed(uint32_t S);
};

extern Vacuum vacuum;

#endif //PP_VACUUM_H
