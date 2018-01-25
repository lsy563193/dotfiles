//
// Created by root on 12/5/17.
//

#ifndef PP_SPEED_GOVERNOR_HPP
#define PP_SPEED_GOVERNOR_HPP

#include <cstdint>

class ISpeedGovernor{
public:
	virtual void adjustSpeed(int32_t&, int32_t&)=0;
};
/*
class SpeedGovernorBack: public ISpeedGovernor
{
public:
	SpeedGovernorBack();

	void adjustSpeed(int32_t &left_speed, int32_t &right_speed) override ;

private:
	int32_t speed_;
};

class SpeedGovernorTurn: public ISpeedGovernor
{
public:
	SpeedGovernorTurn(int16_t target_angle);

	void adjustSpeed(int32_t &left_speed, int32_t &right_speed) override ;

private:
	int16_t target_angle_;
	int32_t speed_;
};*/

#endif //PP_SPEED_GOVERNOR_HPP
