//
// Created by lsy563193 on 12/14/17.
//
#include "ros/ros.h"
#include <cstdint>
#include <mathematics.h>
#include <deque>
#include <pp.h>
#include <error.h>
Cell_t g_zero_home;
bool	g_start_point_seen_charger;
bool g_have_seen_charger;
std::deque<Cell_t> g_passed_path;
bool g_move_back_finished;
static uint8_t g_cleaning_mode = 0;
bool line_is_found;
bool g_motion_init_succeeded;
bool g_resume_cleaning;
//boost::shared_ptr<State> ACleanMode::sp_state_ = nullptr;
//boost::shared_ptr<IMoveType> ACleanMode::sp_move_type_ = nullptr;
uint32_t g_wf_start_timer;
uint32_t g_wf_diff_timer;
int g_wf_reach_count;
//SpotMovement::instance()
double g_time_straight;
int g_wall_distance;
int16_t g_turn_angle;
bool g_allow_check_path_in_advance;
bool g_check_path_in_advance;
bool g_slip_backward;
time_t last_time_remote_spot = time(NULL);
std::vector<Cell_t> g_homes;
std::vector<int> g_home_way_list;
std::vector<int>::iterator g_home_way_it;
bool g_go_home_by_remote = false;
Cell_t g_home_point;

std::deque <Cell_t> path_points;
double bumper_turn_factor = 0.85;

bool fw_is_time_up()
{
	return ((uint32_t)difftime(time(NULL), g_wf_start_timer)) > g_wf_diff_timer;
}

void cm_set(uint8_t mode)
{
	g_cleaning_mode = mode;
}

bool cm_is_go_charger()
{
	return g_cleaning_mode == Clean_Mode_Go_Charger;
}

uint8_t cm_get()
{
	return g_cleaning_mode;
}

bool is_fobbit_free() {
	//NOTE: g_home_way_it should last of g_home_point,for g_homeway_list may empty.
	return (/*cs.is_going_home() &&*/ *g_home_way_it % HOMEWAY_NUM == USE_CLEANED);
}

void path_set_home(const Cell_t& curr)
{
	bool is_found = false;

	for (const auto& it : g_homes) {
		ROS_INFO("%s %d: curr\033[33m(%d, %d)\033[0m home_it\033[33m(%d,%d)\033[0m.", __FUNCTION__, __LINE__, curr.X, curr.Y,it.X,it.Y);
		if (it == curr) {
			is_found = true;
			break;
		}
	}
	if (!is_found) {
		ROS_INFO("%s %d: Push new reachable home:\033[33m (%d, %d)\033[0m to home point list.", __FUNCTION__, __LINE__, curr.X, curr.Y);
		if(cm_get() != Clean_Mode_Spot)
			g_have_seen_charger = true;
		// If curr near (0, 0)
		if (abs(curr.X) >= 5 || abs(curr.Y) >= 5)
		{
			if(g_homes.size() >= HOME_CELLS_SIZE+1)//escape_count + zero_home = 3+1 = 4
			{
				std::copy(g_homes.begin() + 2, g_homes.end(), g_homes.begin()+1);//shift 1 but save zero_home
				g_homes.pop_back();
			}
			g_homes.push_back(curr);
		}
	}
	else if(curr == g_zero_home && cm_get() != Clean_Mode_Spot)
	{
		g_start_point_seen_charger = true;
		g_have_seen_charger = true;
	}
}

void cs_disable_motors(void)
{
	wheel.stop();
	brush.stop();
	vacuum.stop();
}

