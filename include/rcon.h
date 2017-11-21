//
// Created by root on 11/17/17.
//

#ifndef PP_RCON_H
#define PP_RCON_H

//#include "move_type.h"
//#include "clean_mode.h"
bool cm_is_exploration();
bool cs_is_going_home();
extern bool g_from_station;
extern bool g_motion_init_succeeded;
extern bool g_in_charge_signal_range;
bool mt_is_follow_wall();
bool mt_is_linear();
bool mt_is_go_to_charger();
class Rcon {

public:

	int get_trig_() {
		enum {
			left, fl1, fl2, fr2, fr1, right
		};
		static int8_t cnt[6] = {0, 0, 0, 0, 0, 0};
		const int MAX_CNT = 1;
//	if(get_status() != 0)
//		ROS_WARN("get_status(%d)",get_status());
		if (get_status() & RconL_HomeT)
			cnt[left]++;
		if (get_status() & RconFL_HomeT)
			cnt[fl1]++;
		if (get_status() & RconFL2_HomeT)
			cnt[fl2]++;
		if (get_status() & RconFR2_HomeT)
			cnt[fr2]++;
		if (get_status() & RconFR_HomeT)
			cnt[fr1]++;
		if (get_status() & RconR_HomeT)
			cnt[right]++;
		auto ret = 0;
		for (int i = 0; i < 6; i++)
			if (cnt[i] > MAX_CNT) {
				cnt[left] = cnt[fl1] = cnt[fl2] = cnt[fr2] = cnt[fr1] = cnt[right] = 0;
				ret = i + 1;
				break;
			}
		reset_status();
		return ret;
	}

	int get_trig(void) {
//	if (cs_is_going_home()) {
////		ROS_WARN("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
//		reset_status();
//		return 0;
//	}
		if (mt_is_follow_wall()) {
//		ROS_WARN("%s %d: rcon(%d).", __FUNCTION__, __LINE__, (RconL_HomeT | RconR_HomeT | RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT));
//		ROS_WARN("%s %d: ~rcon(%d).", __FUNCTION__, __LINE__, ~(RconL_HomeT | RconR_HomeT | RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT));
//		ROS_WARN("%s %d: status(%d).", __FUNCTION__, __LINE__, (get_status() & (RconL_HomeT | RconR_HomeT | RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT)));
			if (!(get_status() &
						(RconL_HomeT | RconR_HomeT | RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT))) {
				reset_status();
				return 0;
			}
			else
				return get_trig_();
		}
		else if (mt_is_linear()) {
//		ROS_WARN("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
			if (cm_is_exploration()) {
				auto status = get_status() & RconAll_Home_T;
				reset_status();
				return status;
			}
				// Since we have front left 2 and front right 2 rcon receiver, seems it is not necessary to handle left or right rcon receives home signal.
			else if (!(get_status() & (RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT))) {
				reset_status();
				return 0;
			}
			else
				return get_trig_();
		}
		else if (mt_is_go_to_charger()) {
			auto status = get_status();
			reset_status();
			return status;
		}

		return 0;
	}

	void set_status(uint32_t code) {
		movement_status = code;
	}

	void reset_status(void) {
		movement_status = 0;
	}

	uint32_t get_status() {
		extern Cell_t g_stub_cell;
		if (!cs_is_going_home() && g_from_station && g_motion_init_succeeded && !mt_is_go_to_charger() &&
				!mt_is_follow_wall()) {//check if robot start from charge station
			if (two_points_distance(g_stub_cell.X, g_stub_cell.Y, map_get_x_cell(), map_get_y_cell()) <= 20) {
				g_in_charge_signal_range = true;
				reset_status();
				return 0;
			}
			else {
				g_in_charge_signal_range = false;
				return movement_status;
			}
		}
		else
			return movement_status;
	}

private:
	uint32_t movement_status;
};

extern Rcon c_rcon;
#endif //PP_RCON_H
