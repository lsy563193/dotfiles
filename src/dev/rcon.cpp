//
// Created by root on 11/17/17.
//

#include "pp.h"
#include "rcon.h"

Rcon c_rcon;

uint32_t Rcon::get_status()
{
		extern Cell_t g_stub_cell;
		if (!cs.is_going_home() && g_from_station && g_motion_init_succeeded && !mt.is_go_to_charger() &&
				!mt.is_follow_wall()) {//check if robot start from charge station
			if (two_points_distance(g_stub_cell.X, g_stub_cell.Y, cost_map.get_x_cell(), cost_map.get_y_cell()) <= 20) {
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
