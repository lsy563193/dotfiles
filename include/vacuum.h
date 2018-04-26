//
// Created by root on 11/17/17.
//

#ifndef PP_VACUUM_H
#define PP_VACUUM_H

#include <cstdint>

class Vacuum
{
public:

	enum VacSpeed
	{
		vac_speed_max = 100, //15500rpm
		vac_speed_normal = 80, //12000rpm
		vac_speed_low = 50,//7500rpm
	};

	enum VacMode
	{
		vac_low_mode = 0,
		vac_normal_mode = 1,
		vac_max_mode = 2,
	};


	bool isCurrentMaxMode()
	{
		return current_mode_ == VacMode::vac_max_mode;
	}

	void setForCurrentMode(int mode);


	bool isUserSetMaxMode()
	{
		return is_user_set_max_mode_;
	}

	void setForUserSetMaxMode(bool is_max)
	{
		is_user_set_max_mode_ = is_max;
	}

	void setSpeedByUserSetMode();

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

	void updateFilterTime(uint32_t addition_time);

	void resetFilterTime();

	uint32_t getFilterTime()
	{
		return filter_time_;
	}

private:
	void slowOperate();

	void normalOperate();

	void fullOperate();

	void setSpeed(uint32_t S);

	int current_mode_{vac_normal_mode};
	bool is_user_set_max_mode_{false};

	bool oc_;

	uint16_t current_;

	bool is_on_{false};

	uint32_t filter_time_{0};
};

extern Vacuum vacuum;

#endif //PP_VACUUM_H
