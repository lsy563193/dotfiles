//
// Created by root on 11/17/17.
//

#include "pp.h"
#include "obs.h"

Obs obs;

typedef int16_t(Obs::*Pfunc)(void);
typedef int16_t(Obs::*Pdata);
void obs_dynamic_base(uint16_t count) {
//	count = 20;
//	enum {front,left,right};
		static uint16_t cnt[] = {0, 0, 0};
		static int32_t sum[] = {0, 0, 0};
		const int16_t dynamic_limit = 2000;
		Pdata p_baseline[] = {&Obs::g_front_baseline, &Obs::g_left_baseline, &Obs::g_right_baseline};
		Pfunc p_get_obs[] = {&Obs::get_front, &Obs::get_left, &Obs::get_right};
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
			auto diff = abs_minus(avg, get);
			if (diff > 50) {
//			ROS_WARN("i = %d, diff = (%d) > 50.", i, diff);
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
//			if(i == 0)
//				ROS_WARN("obs front baseline = %d.", *p_baseline_);
//			else if(i == 1)
//				ROS_WARN("obs left baseline = %d.", *p_baseline_);
//			else if(i == 2)
//				ROS_WARN("obs right baseline = %d.", *p_baseline_);
			}
		}
	}
