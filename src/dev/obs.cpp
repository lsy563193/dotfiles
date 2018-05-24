//
// Created by root on 11/17/17.
//

#include "ros/ros.h"
#include "obs.h"
#include <serial.h>
#include <map.h>

Obs obs;

Obs::Obs()
{
	left_value_ = 0;
	front_value_ = 0;
	right_value_ = 0;
	left_trig_value_ = 100;
	front_trig_value_ = 100;
	right_trig_value_ = 100;
	left_baseline_ = 100;
	front_baseline_ = 100;
	right_baseline_ = 100;
}

void Obs::setLeft(int16_t value)
{
	left_value_ = value - getLeftBaseline();
}

void Obs::setFront(int16_t value)
{
	front_value_ = value - getFrontBaseline();
}

void Obs::setRight(int16_t value)
{
	right_value_ = value - getRightBaseline();
}

uint8_t Obs::getStatus(int16_t left_offset, int16_t front_offset, int16_t right_offset)
{
	uint8_t status = 0;

	if (!switch_)
		return 0;

	if (getLeft() > getLeftTrigValue() + left_offset)
		status |= BLOCK_LEFT;

	if (getFront() > getFrontTrigValue() + front_offset)
		status |= BLOCK_FRONT;

	if (getRight() > getRightTrigValue() + right_offset)
		status |= BLOCK_RIGHT;

	return status;
}

void Obs::DynamicAdjust(uint16_t count) {
//	count = 20;
//	enum {front,left,right};
		static uint16_t cnt[] = {0, 0, 0};
		static int32_t sum[] = {0, 0, 0};
		const int16_t dynamic_limit = 2000;
		Pdata p_baseline[] = {&Obs::front_baseline_, &Obs::left_baseline_, &Obs::right_baseline_};
		Pfunc p_get_obs[] = {&Obs::getFront, &Obs::getLeft, &Obs::getRight};
//	if(count == 0)
//		return ;
		for (int i = 0; i < 3; i++) {
//		if(i == 0)
//			ROS_WARN("front-------------------------");
//		if(i == 1)
//			ROS_WARN("left-------------------------");
//		if(i == 2)
//			ROS_WARN("right-------------------------");

		auto p_baseline_ = obs.*(p_baseline[i]);
		auto get = (obs.*(p_get_obs[i]))();

//		ROS_WARN("trig_val(%d),get(%d)", *p_baseline_, get);
		sum[i] += get;
		cnt[i]++;
		int16_t avg = sum[i] / cnt[i];
//		ROS_WARN("i = %d, avg(%d), (%d / %d), ",i, avg, sum[i], cnt[i]);
		auto diff = std::abs(avg - get);
		if (diff > 50) {
//		ROS_WARN("i = %d, diff = (%d) > 50.", i, diff);
			cnt[i] = 0;
			sum[i] = 0;
		}
		if (cnt[i] > count) {
			cnt[i] = 0;
			sum[i] = 0;
			get = avg / 2 + p_baseline_;
			if (get > dynamic_limit)
				get = dynamic_limit;

			obs.*(p_baseline[i]) = get;
//		if(i == 0)
//			ROS_WARN("obs front baseline = %d.", *p_baseline_);
//		else if(i == 1)
//			ROS_WARN("obs left baseline = %d.", *p_baseline_);
//		else if(i == 2)
//			ROS_WARN("obs right baseline = %d.", *p_baseline_);
		}
	}
}

bool Obs::frontTriggered(void)
{
	return (getFront() > getFrontTrigValue());
}

void Obs::control(bool _switch)
{
	uint8_t data;

	switch_ = _switch;

	if (switch_)
		data = static_cast<uint8_t>(serial.getSendData(CTL_MIX) | 0x10);
	else
		data = static_cast<uint8_t>(serial.getSendData(CTL_MIX) & ~0x10);

	serial.setSendData(CTL_MIX, data);
	ROS_INFO("%s %d: OBS set to %s.", __FUNCTION__, __LINE__, switch_ ? "ON" : "OFF");
}
