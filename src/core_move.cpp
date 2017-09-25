#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <limits.h>

#include "main.h"
#include "laser.hpp"
#include "robot.hpp"
#include "robotbase.h"
#include "core_move.h"

#include "gyro.h"
#include "map.h"
#include "mathematics.h"
#include "path_planning.h"
#include "rounding.h"
#include "shortest_path.h"
#include "spot.h"
#include "go_home.hpp"

#include "movement.h"
#include "wall_follow_trapped.h"
#include <ros/ros.h>
#include <vector>
#include <chrono>
#include <functional>
#include <future>
#include <list>
#include <charger.hpp>
#include "space_exploration.h"
#include <wav.h>
//#include "../include/obstacle_detector.h"
#include <motion_manage.h>
#include <slam.h>
#include "event_manager.h"
#include "mathematics.h"
#include "wall_follow_slam.h"
#include <move_type.h>
#include <path_planning.h>
//#include "obstacle_detector.h"
//using namespace obstacle_detector;

#include "regulator.h"
#ifdef TURN_SPEED
#undef TURN_SPEED
#endif

#define TURN_SPEED	17
/*
 * MOVE_TO_CELL_SEARCH_INCREMENT: offset from robot's current position.
 * MOVE_TO_CELL_SEARCH_MAXIMUM_LENGTH: max searching radius from robot's current position.
 */
#define MOVE_TO_CELL_SEARCH_INCREMENT 2
#define MOVE_TO_CELL_SEARCH_MAXIMUM_LENGTH 10
#define MOVE_TO_CELL_SEARCH_ARRAY_LENGTH (2 * MOVE_TO_CELL_SEARCH_MAXIMUM_LENGTH / MOVE_TO_CELL_SEARCH_INCREMENT + 1)
#define MOVE_TO_CELL_SEARCH_ARRAY_LENGTH_MID_IDX ((MOVE_TO_CELL_SEARCH_ARRAY_LENGTH * MOVE_TO_CELL_SEARCH_ARRAY_LENGTH - 1) / 2)

#define	MAX_SPEED		(30)
#define WHEEL_BASE_HALF			(((double)(WHEEL_BASE)) / 2)
#define CELL_COUNT	(((double) (CELL_COUNT_MUL)) / CELL_SIZE)
#define RADIUS_CELL (3 * CELL_COUNT_MUL)

/*---cm_check_charger_signal() return value---*/
#define SEEN_CHARGER 1
#define EVENT_TRIGGERED 2

int g_rcon_triggered = 0;//1~6
bool g_exploration_home = false;
bool g_rcon_during_go_home = false;
bool g_rcon_dirction = false;
uint16_t g_straight_distance;
uint32_t g_escape_trapped_timer;
int g_is_reach = 1;

extern int g_trapped_mode;
bool g_should_follow_wall;
//std::vector<int16_t> g_left_buffer;
//std::vector<int16_t> g_right_buffer;

Cell_t g_next_cell, g_target_cell;

//uint8_t	g_remote_go_home = 0;
bool	g_go_home = false;
bool	g_from_station = 0;
int16_t g_map_gyro_offset = 0;
// This flag is indicating robot is resuming from low battery go home.
bool g_resume_cleaning = false;

// This flag is for checking whether map boundary is created.
bool g_map_boundary_created = false;

bool g_have_seen_charge_stub = false;
bool g_start_point_seen_charger = false;

Cell_t g_relativePos[MOVE_TO_CELL_SEARCH_ARRAY_LENGTH * MOVE_TO_CELL_SEARCH_ARRAY_LENGTH] = {{0, 0}};

extern Cell_t g_cell_history[];

extern int16_t g_x_min, g_x_max, g_y_min, g_y_max;

// Saved position for move back.
float saved_pos_x, saved_pos_y;

// Flag for indicating whether move back has finished.
bool g_move_back_finished = true;

// Flag for indicating whether motion instance is initialized successfully.
bool g_motion_init_succeeded = false;

bool g_go_home_by_remote = false;

//Flag for judge if keep on wall follow
bool g_keep_on_wf = false;

bool g_finish_cleaning_go_home = false;

bool g_no_uncleaned_target = false;

time_t last_time_remote_spot = time(NULL);
int16_t ranged_angle(int16_t angle)
{
	while (angle > 1800 || angle <= -1800)
	{
		if (angle > 1800) {
			angle -= 3600;
		} else
		if (angle <= -1800) {
			angle += 3600;
		}
	}
	return angle;
}

bool is_map_front_block(int dx)
{
	int32_t x, y;
	for (auto dy = -1; dy <= 1; dy++)
	{
		cm_world_to_point(gyro_get_angle(), dy * CELL_SIZE, CELL_SIZE * dx, &x, &y);
		if (map_get_cell(MAP, count_to_cell(x), count_to_cell(y)) == BLOCKED_BOUNDARY)
			return true;
	}
	return false;
}

//--------------------------------------------------------
static double radius_of(Cell_t cell_0,Cell_t cell_1)
{
	return (abs(cell_to_count(cell_0.X - cell_1.X)) + abs(cell_to_count(cell_0.Y - cell_1.Y))) / 2;
}

void cm_world_to_point(int16_t heading, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y)
{
	*x = cell_to_count(count_to_cell(map_get_relative_x(heading, offset_lat, offset_long, true)));
	*y = cell_to_count(count_to_cell(map_get_relative_y(heading, offset_lat, offset_long, true)));
}
void cm_world_to_cell(int16_t heading, int16_t offset_lat, int16_t offset_long, int16_t &x, int16_t &y)
{
	x = count_to_cell(map_get_relative_x(heading, offset_lat, offset_long, false));
	y = count_to_cell(map_get_relative_y(heading, offset_lat, offset_long, false));
}
void mark_offset(int16_t dx, int16_t dy, CellState status)
{
	int x,y;
	cm_world_to_point(gyro_get_angle(), CELL_SIZE * dy, CELL_SIZE*dx, &x, &y);
	map_set_cell(MAP, x,y,status);
}
int cm_get_grid_index(float position_x, float position_y, uint32_t width, uint32_t height, float resolution,
											double origin_x, double origin_y)
{
	int index, grid_x, grid_y;

	/*get index*/
	grid_x = int(round((position_x - origin_x) / resolution));
	grid_y = int(round((position_y - origin_y) / resolution));
	index = grid_x + grid_y * width;
	
	return index;
}

//void cm_update_map()
//{
//	auto last = map_get_curr_cell();

//	auto curr = cm_update_position();

//	ROS_WARN("1 last(%d,%d),curr(%d,%d)",last.X, last.Y, curr.X, curr.Y);

//	ROS_ERROR("2 last(%d,%d),curr(%d,%d)",last.X, last.Y,curr.X,curr.Y);
//	if (last != curr )
//	{

//	map_set_cleaned(curr);
//		if (get_bumper_status() != 0 || get_cliff_status() != 0 || get_obs_status() != 0)
//		MotionManage::pubCleanMapMarkers(MAP, g_next_cell, g_target_cell);
//	}

//}

//-------------------------------cm_move_back-----------------------------//

uint16_t round_turn_distance()
{
	auto wall = robot::instance()->getLeftWall();
	if (mt_is_left())
		wall = robot::instance()->getRightWall();

	auto distance = wall > (Wall_Low_Limit) ? wall / 3 : 220;
	check_limit(distance, Wall_Low_Limit, Wall_High_Limit);
	return distance;
}

uint16_t round_turn_speed()
{
	if ((mt_is_left()) && (get_bumper_status()  == RightBumperTrig) ||
			(mt_is_right()) && (get_bumper_status()  == LeftBumperTrig) )
		return 15;
	return 10;
}

std::vector<int16_t> reset_rounding_wall_buffer()
{

}
uint16_t bumper_straight_distance()
{
	if(get_bumper_status() == AllBumperTrig){
		return 150;
	}else if(get_bumper_status() == LeftBumperTrig)
	{
		if (mt_is_left()) return 250;
		else return 375;
	}else if(get_bumper_status() == RightBumperTrig)
	{
		if (mt_is_left()) return 375;
		else return 259;
	}
}

/*--------------------------Head Angle--------------------------------*/
bool cm_head_to_course(uint8_t speed_max, int16_t angle)
{
	bool return_value = false;
	stop_brifly();

	cm_event_manager_turn(true);

	TurnSpeedRegulator regulator(speed_max, ROTATE_LOW_SPEED, 4);
	bool		eh_status_now, eh_status_last;
	while (ros::ok()) {
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}

		if (g_fatal_quit_event
			|| g_key_clean_pressed || (!g_go_home && (g_battery_home || g_remote_home))
			|| g_oc_wheel_left || g_oc_wheel_right
			|| g_bumper_triggered || g_cliff_triggered
			)
			break;

		if(get_rcon_status())
		{
			return_value = true;
			break;
		}
		auto diff = ranged_angle(angle - gyro_get_angle());

		if (std::abs(diff) < 10) {
			stop_brifly();
			ROS_INFO("%s %d: angle: %d\tGyro: %d\tDiff: %d", __FUNCTION__, __LINE__, angle, gyro_get_angle(), diff);
			break;
		}
		uint8_t speed_up;
		regulator.adjustSpeed(diff, speed_up);
		set_wheel_speed(speed_up, speed_up);

	}

	cm_event_manager_turn(false);
	set_wheel_speed(0, 0);
	return return_value;
}

/*
 * Robot move to target cell
 * @param target: Target cell.
 * @return	-2: Robot is trapped
 *			-1: Robot cannot move to target cell
 *			1: Robot arrive target cell
 */
enum {
	EXIT_CLEAN=-1,
	NO_REATH_TARGET=0,
	REATH_TARGET=1,
};
int cm_move_to(const PPTargetType& path)
{
	Cell_t curr = map_get_curr_cell();
//#if INTERLACED_MOVE
//	extern uint16_t g_new_dir;
//	if (mt_is_linear() && IS_X_AXIS(g_new_dir))
//		curr.Y = path.cells.front().Y;
//#endif
	RegulatorManage rm(curr, g_next_cell, path);

	bool eh_status_now=false, eh_status_last=false;
	int ret = EXIT_CLEAN;

	std::vector<Cell_t> passed_path;
	passed_path.clear();
//	passed_path.push_back(curr);

	int32_t	 speed_left = 0, speed_right = 0;
	while (ros::ok())
	{
		/*for exploration mark the map and detect the rcon signal*/
		if (get_clean_mode() == Clean_Mode_Exploration) {
			explore_update_map();
			/*if (get_rcon_status()) {
				g_exploration_home = true;
				ret = true;
				break;
			}*/
		}
		/*for navigation trapped wall follow update map and push_back vector*/
		if((!mt_is_linear()) && get_clean_mode() == Clean_Mode_Navigation && g_trapped_mode == 1)
			wf_update_map(WFMAP);

		/*for exploration trapped wall follow update map and push_back vector*/
		if((!mt_is_linear()) && get_clean_mode() == Clean_Mode_Exploration && g_trapped_mode == 1)
			wf_update_map(WFMAP);

		/*for wall follow mode update map and push_back vector*/
		if(!mt_is_linear() && get_clean_mode() == Clean_Mode_WallFollow && !g_go_home)
			wf_update_map(WFMAP);

		if (/*get_clean_mode() == Clean_Mode_WallFollow &&*/ mt_is_linear()) {
			wall_dynamic_base(30);
		}

		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1)
		{
			usleep(100);
			continue;
		}

		if(!rm.isTurn())
		{
			curr = cm_update_position();
			rm.updatePosition({map_get_x_count(),map_get_y_count()});
		}

		if (rm.isExit()){
			break;
		}
		if (g_slam_error)
		{
			set_wheel_speed(0, 0);
			continue;
		}

		if (rm.isReach()){
			ret = REATH_TARGET;
			break;
		}
		if (rm.isStop()){
			ret = NO_REATH_TARGET;
			break;
		}
		if (rm.isSwitch()){
			map_set_blocked();
			MotionManage::pubCleanMapMarkers(MAP, g_next_cell, g_target_cell, path.cells);
			rm.switchToNext();
		}

		if (get_clean_mode() != Clean_Mode_WallFollow
						&& (mt_is_linear() || mt_is_follow_wall()))
		{
			if (!rm.isTurn() &&(passed_path.empty() || passed_path.back() != curr))
			{
				extern uint16_t g_new_dir;
				if (g_trapped_mode == 0)
				{
					if (passed_path.empty() ||(!mt_is_linear() || curr.X > passed_path.back().X ^ g_new_dir != POS_X)) // This checking is for avoiding position jumping back or aside during linear movement.
						passed_path.push_back(curr);
				}
//				if(MAP_SET_REALTIME)
				{
					//map_set_realtime();
//					if( g_trapped_mode==0 )
//						map_set_cleaned(curr);
					if (mt_is_follow_wall())
						map_set_follow_wall(curr);
				}

				if (g_trapped_mode == 1 )
				{
					auto is_block_clear = map_mark_robot(MAP);
					if(is_block_clear)
					{
						Cell_t target;
						int count = 0;
						if(path_dijkstra(curr, target,count))
//						BoundingBox2 map;
//						if (get_reachable_targets(curr, map))
						{
							ROS_WARN("%s %d: Found reachable targets, exit trapped.", __FUNCTION__, __LINE__);
							g_trapped_mode = 2;
							passed_path.clear(); // No need to update the cleaned path because map_mark_robot() has finished it.
						} /*else if ((double)count / (double)map_get_cleaned_area() < 0.8)
						{
							ROS_WARN("%s %d: Found reachable home, exit trapped.", __FUNCTION__, __LINE__);
							g_trapped_mode = 2;
							passed_path.clear(); // No need to update the cleaned path because map_mark_robot() has finished it.
						}*/ else
							ROS_INFO("%s %d:Still trapped.",__FUNCTION__,__LINE__);
					}
				}
			}
		}

		rm.adjustSpeed(speed_left, speed_right);
		set_wheel_speed(speed_left, speed_right);
	}
	if(! MAP_SET_REALTIME)
	{
		map_set_cleaned(passed_path);
		if (!mt_is_linear())
			map_set_follow_wall(passed_path);

//		linear_mark_clean(curr, g_next_cell);
	}
	map_set_blocked();

	MotionManage::pubCleanMapMarkers(MAP, g_next_cell, g_target_cell, path.cells);
	return ret;
}

/*

bool cm_turn_move_to_point(Point32_t Target, uint8_t speed_left, uint8_t speed_right)
{
	auto angle_start = gyro_get_angle();
	move_forward(speed_left, speed_right);

	bool eh_status_now = false;
	bool eh_status_last = false;
	cm_set_event_manager_handler_state(true);
	while (ros::ok())
	{
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1)
		{
			usleep(100);
			continue;
		}

		if (g_fatal_quit_event || g_key_clean_pressed
			|| g_bumper_triggered || g_obs_triggered || g_cliff_triggered || g_rcon_triggered
			|| (!g_go_home && (g_battery_home || g_remote_home))
			|| g_remote_spot // It will only be set if robot is not during spot.
			|| g_remote_direction_keys) // It will only be set if robot is during spot.
			return false;

		auto angle_diff = ranged_angle(gyro_get_angle() - angle_start);
		if (abs(angle_diff) >= 900){
			ROS_WARN("%s %d: reach line between point 1 & 2: \n", __FUNCTION__, __LINE__);
			break;
		}
	}
	stop_brifly();

	cm_set_event_manager_handler_state(false);

	return true;
}

bool cm_curve_move_to_point()
{
	auto *path_cells = path_get_path_points();
	std::vector<Cell_t>	cells;
	std::copy_n((*path_cells).begin(), 3, std::back_inserter(cells));
	path_reset_path_points();
	std::string msg;
	for (auto& cell : cells) {
		msg += "(" + std::to_string(cell.X) + ", " + std::to_string(cell.Y) + ")->";
	}
	ROS_ERROR("cells:%s", msg.c_str());

	auto radius_1 = radius_of(cells[1], cells[0]);
	auto radius_2 = radius_of(cells[2], cells[1]);

	if (radius_1 < RADIUS_CELL || radius_2 < RADIUS_CELL){
		ROS_WARN("%s %d: radius_1: %f\tradius_2: %f (%d)\n", __FUNCTION__, __LINE__, radius_1, radius_2, 3 * CELL_COUNT_MUL);
		return false;
	}

	auto radius = std::min(radius_1, radius_2);

	Point32_t target{cell_to_count(cells[1].X), cell_to_count(cells[1].Y)};
#define IS_MOVE_FOLLOW_Y_AXIS_FIRST (cells[0].X == cells[1].X)
#define IS_IN_NAV_X_AXIS (cells[0].X < cells[2].X)
#define IS_IN_NAV_Y_AXIS (cells[0].Y < cells[2].Y)
	if (IS_MOVE_FOLLOW_Y_AXIS_FIRST) {
		target.Y += (IS_IN_NAV_X_AXIS ? -radius : radius);
	} else {
		target.X += (IS_IN_NAV_Y_AXIS ? -radius : radius);
	}

	//1/3 move to first target
	if(!cm_move_to(target) )
		return false;

	//2/3 calculate the curve speed.
	auto speed = (uint8_t) ceil(MAX_SPEED * (radius / CELL_COUNT - WHEEL_BASE_HALF) / (radius / CELL_COUNT + WHEEL_BASE_HALF));
	uint8_t speed_left = MAX_SPEED;
	uint8_t speed_right = MAX_SPEED;
	*/
/*
 *             -x
 *  x^y   -         +
 *        0 >-  1 -<0
 *        v         v
 *        +         +
 *   -y   1     2   1 +y
 *        +         +
 *        ^         ^
 *        0 >-  1 -<0
 * x^y    +         -
 *     x^y      +x     x^y
*//*

	auto is_speed_right = (IS_IN_NAV_X_AXIS) ^ (IS_IN_NAV_Y_AXIS) ^ (IS_MOVE_FOLLOW_Y_AXIS_FIRST);
	(is_speed_right ? speed_right : speed_left) = speed;

	ROS_ERROR("is_speed_right(%d),speed_left(%d),speed_right(%d)",is_speed_right,speed_left,speed_right);

	if ( speed_left < 0 || speed_left > MAX_SPEED || speed_right < 0 || speed_right > MAX_SPEED)
		return false;

	//2/3 move to last route
	if(!cm_turn_move_to_point(target, speed_left, speed_right))
		return false;

	target.X = cell_to_count(cells[2].X);
	target.Y = cell_to_count(cells[2].Y);

	//3/3 continue to move to target
	ROS_ERROR("is_speed_right(%d),speed_left(%d),speed_right(%d)",is_speed_right,speed_left,speed_right);
	if(!cm_move_to(target))
		return false;

	return true;
}
*/

//#if 0
void linear_mark_clean(const Cell_t &start, const Cell_t &target)
{
	if (start.Y == target.Y)
	{
		Cell_t stop = {map_get_x_cell(), map_get_y_cell()};
		if (start.X != stop.X && (abs(start.Y - stop.Y) <= 2))
		{
			float slop = (((float) start.Y) - ((float) stop.Y)) / (((float) start.X) - ((float) stop.X));
			float intercept = ((float) (stop.Y)) - slop * ((float) (stop.X));

			auto start_x = std::min(start.X, stop.X);
			auto stop_x = std::max(start.X, stop.X);
			for (auto x = start_x; x<=stop_x; x++)
			{
				auto y = (int16_t) (slop * (stop.X) + intercept);
				for(auto dy=-ROBOT_SIZE_1_2;dy<=ROBOT_SIZE_1_2;dy++)
					map_set_cell(MAP, cell_to_count(x), cell_to_count(y + dy), CLEANED);
			}
		}
	}
}
//#endif

int cm_cleaning()
{
	PPTargetType cleaning_path;
	cleaning_path.target.X = 0;
	cleaning_path.target.Y = 0;
	cleaning_path.cells.clear();
	g_is_reach = REATH_TARGET;
	MotionManage motion;
	if (!motion.initSucceeded())
		return 0;

	Cell_t curr = cm_update_position();
	g_motion_init_succeeded = true;
	g_robot_slip_enable = true;
	g_robot_stuck = false;
	g_robot_slip = false;
	ROS_INFO("\033[35menable robot stuck\033[0m");
	while (ros::ok())
	{
		if (g_key_clean_pressed || g_fatal_quit_event )
			return -1;

		if (!g_go_home)
			cm_check_should_go_home();

		cm_check_temp_spot();

		path_update_cell_history();
//		path_update_cells();
		curr = map_get_curr_cell();
		int8_t is_found = path_next(curr, cleaning_path);
		MotionManage::pubCleanMapMarkers(MAP, g_next_cell, g_target_cell, cleaning_path.cells);
		ROS_INFO("%s %d: is_found: %d, next cell(%d, %d).", __FUNCTION__, __LINE__, is_found, g_next_cell.X, g_next_cell.Y);
		if (is_found == 0 ) //No target point
		{
			if(get_clean_mode() == Clean_Mode_Spot)
				return 0;
			uint8_t check_status = cm_check_charger_signal();
			if(check_status == SEEN_CHARGER)/*---have seen charger signal---*/
			{
				if(!cm_is_continue_go_to_charger())
					return -1;
			}
			else if(check_status == EVENT_TRIGGERED)/*---event triggered---*/
			{
				return -1;
			}
			extern bool g_have_seen_charge_stub;
			ROS_INFO("g_have_seen_charge_stub = %d", g_have_seen_charge_stub);
			ROS_INFO("g_no_uncleaned_target = %d", g_no_uncleaned_target);
			if (get_clean_mode() == Clean_Mode_Exploration || g_no_uncleaned_target) {
				g_no_uncleaned_target = false;
				if (curr == g_zero_home) {
					auto angle = static_cast<int16_t>(robot::instance()->startAngle() *10);
					if(cm_head_to_course(ROTATE_TOP_SPEED, -angle))
					{
						if(!cm_is_continue_go_to_charger())
							return -1;
					}
				}
			} else {/*this case is for exploration when seen the charge stub before, but it can't climb it at last*/
				// If it is at (0, 0), it means all other home point not reachable, except (0, 0).
				if (curr == g_zero_home) {
					auto angle = static_cast<int16_t>(robot::instance()->startAngle() *10);
					if(cm_head_to_course(ROTATE_TOP_SPEED, -angle))
					{
						if(!cm_is_continue_go_to_charger())
							return -1;
					}
					if (g_have_seen_charge_stub == false)
						turn_into_exploration(false);
					else
						turn_into_exploration(true);
					continue;
				}
			}
			return 0;
		}
		else if (is_found == 1)//exist target
		{
//			if (mt_is_follow_wall() || path_get_path_points_count() < 3 || !cm_curve_move_to_point())
			if((g_is_reach = cm_move_to(cleaning_path)) == EXIT_CLEAN) {
				return -1;
			}

			if (cm_should_self_check()){
				// Can not set handler state inside cm_self_check(), because it is actually a universal function.
				cm_set_event_manager_handler_state(true);
				cm_self_check();
				if(get_clean_mode() == Clean_Mode_WallFollow) {
					g_keep_on_wf = true;
					//wf_break_wall_follow();
				}
				cm_set_event_manager_handler_state(false);
			}
			else if(g_go_home)
			{
				if(curr == g_home)
				{
					if (g_home != g_zero_home || g_start_point_seen_charger )
					{
						if(!cm_is_continue_go_to_charger())
							return -1;
					}
					else
					{
						// Reach g_zero_home(0, 0).
						g_homes.pop_back();
						g_home_way_list.clear();
					}
				}
				else if (g_rcon_during_go_home)
				{
					if(!cm_is_continue_go_to_charger())
						return -1;
				}
			}
			//for exploration to climb the charge stub
			else if (g_exploration_home)
			{
				ROS_WARN("Exploration receive rcon signal");
				g_exploration_home = false;
				if (cm_go_to_charger())
					return -1;
			}

		}
		else if (is_found == 2) {
			return -1;
		}
	}
	return 0;
}

void cm_check_should_go_home(void)
{
	if (g_remote_home || g_battery_home || g_finish_cleaning_go_home)
	{
		ROS_WARN("%s %d: Receive g_remote_home(\033[32m%d\033[0m), g_battery_home(\033[32m%d\033[0m), finish cleaning(\033[32m%d\033[0m).", __FUNCTION__, __LINE__,g_remote_home,g_battery_home,g_finish_cleaning_go_home);
		debug_map(MAP, map_get_x_cell(), map_get_y_cell());
		g_go_home = true;
		work_motor_configure();
		if (get_clean_mode() == Clean_Mode_WallFollow)
		{
			robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle); //For wall follow mode.
			cm_update_position();
			//wf_mark_home_point();
			map_reset(MAP);
			ros_map_convert(MAP, true,false, false);
			map_mark_robot(MAP);//note: To clear the obstacles befor go home, please don't remove it!
		}
		if (g_battery_home)
			wav_play(WAV_BATTERY_LOW);
		if (g_remote_home)
		{
			set_led_mode(LED_STEADY, LED_ORANGE);
			g_go_home_by_remote = true;
		}
		wav_play(WAV_BACK_TO_CHARGER);
//		if (!g_battery_home && !g_map_boundary_created)
//			cm_create_home_boundary();
		g_remote_home = false;
		g_battery_home = false;
		g_finish_cleaning_go_home = false;
	}
}

void cm_check_temp_spot(void)
{
	if (!g_go_home && g_remote_spot)
	{
		if( SpotMovement::instance() -> getSpotType() == NO_SPOT){
			ROS_INFO("%s %d: Entering temp spot during navigation.", __FUNCTION__, __LINE__);
			Cell_t curr_cell = map_get_curr_cell();
			ROS_WARN("%s %d: current cell(%d, %d).", __FUNCTION__, __LINE__, curr_cell.X, curr_cell.Y);
			SpotMovement::instance() ->setSpotType(CLEAN_SPOT);
			set_wheel_speed(0, 0);
		}
		else if(SpotMovement::instance()->getSpotType() == CLEAN_SPOT){
			ROS_INFO("%s %d: Exiting temp spot.", __FUNCTION__, __LINE__);
			SpotMovement::instance()->spotDeinit();
			set_wheel_speed(0, 0);
			wav_play(WAV_CLEANING_CONTINUE);
		}
		g_remote_spot = false;
	}
}

/* Statement for cm_go_to_charger_(void)
 * return : true -- going to charger has been stopped, either successfully or interrupted.
 *          false -- going to charger failed, move to next point.
 */
bool cm_go_to_charger()
{
	// Call GoHome() function to try to go to charger stub.
	ROS_INFO("%s,%d,Call GoHome(),\033[35m disable tilt detect\033[0m.",__FUNCTION__,__LINE__);
	g_tilt_enable = false; //disable tilt detect
#if GO_HOME_REGULATOR
	set_led_mode(LED_STEADY, LED_ORANGE);
	mt_set(CM_GO_TO_CHARGER);
	PPTargetType path_empty;
	cm_move_to(path_empty);
	set_led_mode(LED_STEADY, LED_GREEN);
#else
	cm_unregister_events();
	go_home();
	cm_register_events();
#endif
	if (g_fatal_quit_event || g_key_clean_pressed || g_charge_detect)
		return true;
	work_motor_configure();
	g_tilt_enable = true; //enable tilt detect
	ROS_INFO("\033[35m" "%s,%d,enable tilt detect" "\033[0m",__FUNCTION__,__LINE__);
	return false;
}

bool cm_is_continue_go_to_charger()
{
	auto way = *g_home_way_it % HOMEWAY_NUM;
	auto cnt = *g_home_way_it / HOMEWAY_NUM;
	ROS_INFO("\033[1;46;37m" "%s,%d:g_home(%d,%d), way(%d), cnt(%d) " "\033[0m", __FUNCTION__, __LINE__,g_home.X,g_home.Y,way, cnt);
	if(cm_go_to_charger() || cnt == 0)
	{
		ROS_INFO("\033[1;46;37m" "%s,%d:cm_go_to_charger_ stop " "\033[0m", __FUNCTION__, __LINE__);
		return false;
	}
	else if(!g_go_home_by_remote)
		set_led_mode(LED_STEADY, LED_GREEN);
	g_home_way_it += (way+1);
	g_homes.pop_back();
	g_home_way_list.clear();
//	cnt = *g_home_way_it / HOMEWAY_NUM;
//	way = *g_home_way_it % HOMEWAY_NUM;
	ROS_INFO("\033[1;46;37m" "%s,%d:cm_is_continue_go_to_charger_home(%d,%d), way(%d), cnt(%d) " "\033[0m", __FUNCTION__, __LINE__,g_home.X, g_home.Y,way, cnt);
	return true;
}

void cm_reset_go_home(void)
{
	ROS_DEBUG("%s %d: Reset go home flags here.", __FUNCTION__, __LINE__);
	g_go_home = false;
	g_map_boundary_created = false;
	g_go_home_by_remote = false;
}

bool cm_check_loop_back(Cell_t target)
{
	bool retval = false;
	if ( target == g_cell_history[1] && target == g_cell_history[3]) {
		ROS_WARN("%s %d Possible loop back (%d, %d), g_cell_history:(%d, %d) (%d, %d) (%d, %d) (%d, %d) (%d, %d).", __FUNCTION__, __LINE__,
						target.X, target.Y,
						g_cell_history[0].X, g_cell_history[0].Y,
						g_cell_history[1].X, g_cell_history[1].Y,
						g_cell_history[2].X, g_cell_history[2].Y,
						g_cell_history[3].X, g_cell_history[3].Y,
						g_cell_history[4].X, g_cell_history[4].Y);
		retval	= true;
	}

	return retval;
}

void cm_create_home_boundary(void)
{
	int16_t i, j, k;
	int16_t xMinSearch, xMaxSearch, yMinSearch, yMaxSearch;

	k = 3;
	xMinSearch = xMaxSearch = yMinSearch = yMaxSearch = SHRT_MAX;
	for (i = g_x_min; xMinSearch == SHRT_MAX; i++) {
		for (j = g_y_min; j <= g_y_max; j++) {
			if (map_get_cell(MAP, i, j) != UNCLEAN) {
				xMinSearch = i - k;
				break;
			}
		}
	}
	for (i = g_x_max; xMaxSearch == SHRT_MAX; i--) {
		for (j = g_y_min; j <= g_y_max; j++) {
			if (map_get_cell(MAP, i, j) != UNCLEAN) {
				xMaxSearch = i + k;
				break;
			}
		}
	}
	for (i = g_y_min; yMinSearch == SHRT_MAX; i++) {
		for (j = g_x_min; j <= g_x_max; j++) {
			if (map_get_cell(MAP, j, i) != UNCLEAN) {
				yMinSearch = i - k;
				break;
			}
		}
	}
	for (i = g_y_max; yMaxSearch == SHRT_MAX; i--) {
		for (j = g_x_min; j <= g_x_max; j++) {
			if (map_get_cell(MAP, j, i) != UNCLEAN) {
				yMaxSearch = i + k;
				break;
			}
		}
	}
	ROS_INFO("%s %d: x: %d - %d\ty: %d - %d", __FUNCTION__, __LINE__, xMinSearch, xMaxSearch, yMinSearch, yMaxSearch);
	for (i = xMinSearch; i <= xMaxSearch; i++) {
		if (i == xMinSearch || i == xMaxSearch) {
			for (j = yMinSearch; j <= yMaxSearch; j++) {
				map_set_cell(MAP, cell_to_count(i), cell_to_count(j), BLOCKED_BUMPER);
			}
		} else {
			map_set_cell(MAP, cell_to_count(i), cell_to_count(yMinSearch), BLOCKED_BUMPER);
			map_set_cell(MAP, cell_to_count(i), cell_to_count(yMaxSearch), BLOCKED_BUMPER);
		}
	}

	// Set the flag.
	g_map_boundary_created = true;
}

/*------------- Self check and resume-------------------*/
void cm_self_check(void)
{
	ROS_WARN("%s %d: Try to resume the oc or jam cases.", __FUNCTION__, __LINE__);
	uint8_t resume_cnt = 0;
	time_t start_time = time(NULL);
	float wheel_current_sum = 0;
	uint8_t wheel_current_sum_cnt = 0;
	uint8_t bumper_jam_state = 1;
	uint8_t vacuum_oc_state = 1;
	int16_t target_angle = 0;
	bool eh_status_now=false, eh_status_last=false;

	if (g_bumper_jam || g_cliff_jam || g_omni_notmove )
	{
		// Save current position for moving back detection.
		saved_pos_x = robot::instance()->getOdomPositionX();
		saved_pos_y = robot::instance()->getOdomPositionY();
	}

	if (g_oc_wheel_left || g_oc_wheel_right)
		disable_motors();

	if (g_oc_suction)
	{
		ROS_WARN("%s, %d: Vacuum Self checking start", __FUNCTION__, __LINE__);
		disable_motors();
		start_self_check_vacuum();
	}

	SelfCheckRegulator regulator;

	robot::instance()->obsAdjustCount(50);
	while (ros::ok) {
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}

		if (g_fatal_quit_event || g_key_clean_pressed)
			break;

		if (g_slam_error)
		{
			set_wheel_speed(0, 0);
			continue;
		}

		// Check for right wheel.
		if (g_oc_wheel_left || g_oc_wheel_right)
		{
			if (time(NULL) - start_time >= 1)
			{
				wheel_current_sum /= wheel_current_sum_cnt;
				if (wheel_current_sum > Wheel_Stall_Limit)
				{
					if (resume_cnt >= 3)
					{
						disable_motors();
						if (g_oc_wheel_left)
						{
							ROS_WARN("%s,%d Left wheel stall maybe, please check!!\n", __FUNCTION__, __LINE__);
							set_error_code(Error_Code_LeftWheel);
						}
						else
						{
							ROS_WARN("%s,%d Right wheel stall maybe, please check!!\n", __FUNCTION__, __LINE__);
							set_error_code(Error_Code_RightWheel);
						}
						g_fatal_quit_event = true;
						break;
					}
					else
					{
						start_time = time(NULL);
						resume_cnt++;
						wheel_current_sum = 0;
						wheel_current_sum_cnt = 0;
						ROS_WARN("%s %d: Failed to resume for %d times.", __FUNCTION__, __LINE__, resume_cnt);
					}
				}
				else
				{
					if (g_oc_wheel_left)
					{
						ROS_WARN("%s %d: Left wheel resume succeeded.", __FUNCTION__, __LINE__);
						g_oc_wheel_left = false;
						work_motor_configure();
					}
					else
					{
						ROS_WARN("%s %d: Left wheel resume succeeded.", __FUNCTION__, __LINE__);
						g_oc_wheel_right = false;
						work_motor_configure();
					}
				}
			}
			else
			{
				if (g_oc_wheel_left)
					wheel_current_sum += (uint32_t) robot::instance()->getLwheelCurrent();
				else
					wheel_current_sum += (uint32_t) robot::instance()->getRwheelCurrent();
				wheel_current_sum_cnt++;
			}
		}
		else if (g_cliff_jam)
		{
			if (!get_cliff_status())
			{
				ROS_WARN("%s %d: Cliff resume succeeded.", __FUNCTION__, __LINE__);
				g_cliff_triggered = 0;
				g_cliff_all_triggered = false;
				g_cliff_cnt = 0;
				g_cliff_all_cnt = 0;
				g_cliff_jam = false;
			}
			float distance;
			distance = sqrtf(powf(saved_pos_x - robot::instance()->getOdomPositionX(), 2) + powf(saved_pos_y - robot::instance()->getOdomPositionY(), 2));
			if (fabsf(distance) > 0.05f)
			{
				if (++resume_cnt >= 3)
				{
					ROS_WARN("%s %d: Cliff jamed.", __FUNCTION__, __LINE__);
					g_fatal_quit_event = true;
					set_error_code(Error_Code_Cliff);
				}
				else
				{
					stop_brifly();
					saved_pos_x = robot::instance()->getOdomPositionX();
					saved_pos_y = robot::instance()->getOdomPositionY();
				}
			}
		}
		else if (g_bumper_jam)
		{
			if (!get_bumper_status())
			{
				ROS_WARN("%s %d: Bumper resume succeeded.", __FUNCTION__, __LINE__);
				g_bumper_jam = false;
				g_bumper_triggered = false;
				g_bumper_cnt = 0;
			}

			switch (bumper_jam_state)
			{
				case 1: // Move back for the first time.
				case 2: // Move back for the second time.
				{
					float distance;
					distance = sqrtf(powf(saved_pos_x - robot::instance()->getOdomPositionX(), 2) + powf(saved_pos_y - robot::instance()->getOdomPositionY(), 2));
					if (fabsf(distance) > 0.05f)
					{
						stop_brifly();
						// If cliff jam during bumper self resume.
						if (get_cliff_status() && ++g_cliff_cnt > 2)
						{
							g_cliff_jam = true;
							resume_cnt = 0;
						}
						else
						{
							bumper_jam_state++;
							ROS_WARN("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state);
						}
						saved_pos_x = robot::instance()->getOdomPositionX();
						saved_pos_y = robot::instance()->getOdomPositionY();
					}
					break;
				}
				case 3: // Move back for the third time.
				{
					float distance;
					distance = sqrtf(powf(saved_pos_x - robot::instance()->getOdomPositionX(), 2) + powf(saved_pos_y - robot::instance()->getOdomPositionY(), 2));
					if (fabsf(distance) > 0.05f)
					{
						// If cliff jam during bumper self resume.
						if (get_cliff_status() && ++g_cliff_cnt > 2)
						{
							g_cliff_jam = true;
							resume_cnt = 0;
						}
						else
						{
							bumper_jam_state++;
							ROS_WARN("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state);
							target_angle = ranged_angle(gyro_get_angle() - 900);
							ROS_WARN("%s %d: target_angle:%d.", __FUNCTION__, __LINE__, target_angle);
						}
					}
					break;
				}
				case 4:
				{
					ROS_DEBUG("%s %d: gyro_get_angle(): %d", __FUNCTION__, __LINE__, gyro_get_angle());
					// If cliff jam during bumper self resume.
					if (get_cliff_status() && ++g_cliff_cnt > 2)
					{
						g_cliff_jam = true;
						resume_cnt = 0;
					}
					else if (abs(ranged_angle(gyro_get_angle() - target_angle)) < 50)
					{
						bumper_jam_state++;
						ROS_WARN("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state);
						target_angle = ranged_angle(gyro_get_angle() + 900);
						ROS_WARN("%s %d: target_angle:%d.", __FUNCTION__, __LINE__, target_angle);
					}
					break;
				}
				case 5:
				{
					// If cliff jam during bumper self resume.
					if (get_cliff_status() && ++g_cliff_cnt > 2)
					{
						g_cliff_jam = true;
						resume_cnt = 0;
					}
					else if (abs(ranged_angle(gyro_get_angle() - target_angle)) < 50)
					{
						ROS_WARN("%s %d: Bumper jamed.", __FUNCTION__, __LINE__);
						g_fatal_quit_event = true;
						set_error_code(Error_Code_Bumper);
					}
					break;
				}
			}
		}
		else if (g_oc_suction)
		{
			if(vacuum_oc_state == 1)
			{
				ROS_DEBUG("%s %d: Wait for suction self check begin.", __FUNCTION__, __LINE__);
				if (get_self_check_vacuum_status() == 0x10)
				{
					ROS_WARN("%s %d: Suction self check begin.", __FUNCTION__, __LINE__);
					reset_self_check_vacuum_controler();
					vacuum_oc_state = 2;
				}
				continue;
			}
			else if (vacuum_oc_state == 2)
			{
				ROS_DEBUG("%s %d: Wait for suction self check result.", __FUNCTION__, __LINE__);
				if (get_self_check_vacuum_status() == 0x20)
				{
					ROS_WARN("%s %d: Resume suction failed.", __FUNCTION__, __LINE__);
					set_error_code(Error_Code_Fan_H);
					g_fatal_quit_event = true;
					break;
				}
				else if (get_self_check_vacuum_status() == 0x00)
				{
					ROS_WARN("%s %d: Resume suction succeeded.", __FUNCTION__, __LINE__);
					g_oc_suction = false;
					g_oc_suction_cnt = 0;
					break;
				}
			}
		}
		else if (g_omni_notmove)
		{
			ROS_ERROR("\033[1m" "%s,%d,omni detect" "\033[0m",__FUNCTION__,__LINE__);
			set_error_code(Error_Code_Omni);
			g_fatal_quit_event = true;
			break;
		}
		else if (g_slip_cnt >= 2)
		{
			if(g_slip_cnt < 3 && g_robot_slip){
				g_robot_slip = false;
				target_angle = ranged_angle(gyro_get_angle() + 900);	
				ROS_INFO("%s,%d,\033[32mrobot slip again slip count %d\033[0m",__FUNCTION__,__LINE__,g_slip_cnt);
			}
			else if(g_slip_cnt <4 && g_robot_slip){
				g_robot_slip = false;
				target_angle = ranged_angle(gyro_get_angle() - 900);
				ROS_INFO("%s,%d,\033[32mrobot slip again slip count %d\033[0m",__FUNCTION__,__LINE__,g_slip_cnt);
			}
			/*
			if(g_robot_slip){
				g_robot_slip = false;
				ROS_INFO("%s,%d,robot slip again",__FUNCTION__,__LINE__);
			}
			*/
			if( abs(gyro_get_angle() - target_angle) <= 50 ){
				g_slip_cnt = 0;
				g_robot_slip = false;
				ROS_INFO("\033[32m%s,%d,reach target angle\033[0m ",__FUNCTION__,__LINE__);
				break;
			}
			if(g_slip_cnt>=4){
				ROS_INFO("%s,%d,robot stuck slip count\033[32m %d \033[0m",__FUNCTION__,__LINE__,g_slip_cnt);
				g_slip_cnt = 0;
				g_robot_stuck = true;
				set_error_code(Error_Code_Stuck);
				g_fatal_quit_event = true;
				break;
			}
		}
		else
			break;

		regulator.adjustSpeed(bumper_jam_state);
	}
}

bool cm_should_self_check(void)
{
	return (g_oc_wheel_left || g_oc_wheel_right || g_bumper_jam || g_cliff_jam || g_oc_suction || g_omni_notmove || g_slip_cnt >= 2);
}

uint8_t cm_check_charger_signal(void)
{
	time_t delay_counter;
	bool eh_status_now, eh_status_last;

	ROS_INFO("%s, %d: wait about 1s to check if charger signal is received.", __FUNCTION__, __LINE__);
	delay_counter = time(NULL);
	while(time(NULL) - delay_counter < 2)
	{
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1)
		{
			usleep(100);
			continue;
		}

		if(get_rcon_status())
		{
			ROS_INFO("%s, %d: have seen charger signal, return and go home now.", __FUNCTION__, __LINE__);
			return SEEN_CHARGER;
		}
		else if(g_key_clean_pressed || g_fatal_quit_event)
		{
			ROS_INFO("%s, %d: event triggered, return now.", __FUNCTION__, __LINE__);
			return EVENT_TRIGGERED;
		}
	}
	return 0;
}

/* Event handler functions. */
void cm_register_events()
{
	ROS_INFO("%s %d: Register events", __FUNCTION__, __LINE__);
	event_manager_set_current_mode(EVT_MODE_NAVIGATION);

	/* Bumper */
	event_manager_register_handler(EVT_BUMPER_ALL, &cm_handle_bumper_all);
	event_manager_register_handler(EVT_BUMPER_LEFT, &cm_handle_bumper_left);
	event_manager_register_handler(EVT_BUMPER_RIGHT, &cm_handle_bumper_right);

#define event_manager_register_and_enable_x(name, y, enabled) \
	event_manager_register_handler(y, &cm_handle_ ##name); \
	event_manager_enable_handler(y, enabled);

	/* OBS */
	event_manager_register_and_enable_x(obs_front, EVT_OBS_FRONT, true);
	//event_manager_register_and_enable_x(obs_left, EVT_OBS_LEFT, true);
	//event_manager_register_and_enable_x(obs_right, EVT_OBS_RIGHT, true);

	/* Cliff */
	event_manager_register_and_enable_x(cliff_all, EVT_CLIFF_ALL, true);
	event_manager_register_and_enable_x(cliff_front_left, EVT_CLIFF_FRONT_LEFT, true);
	event_manager_register_and_enable_x(cliff_front_right, EVT_CLIFF_FRONT_RIGHT, true);
	event_manager_register_and_enable_x(cliff_left_right, EVT_CLIFF_LEFT_RIGHT, true);
	event_manager_register_and_enable_x(cliff_front, EVT_CLIFF_FRONT, true);
	event_manager_register_and_enable_x(cliff_left, EVT_CLIFF_LEFT, true);
	event_manager_register_and_enable_x(cliff_right, EVT_CLIFF_RIGHT, true);

	/* RCON */
	event_manager_register_and_enable_x(rcon, EVT_RCON, false);
/*
	event_manager_register_and_enable_x(rcon_front_left, EVT_RCON_FRONT_LEFT, false);
	event_manager_register_and_enable_x(rcon_front_left2, EVT_RCON_FRONT_LEFT2, false);
	event_manager_register_and_enable_x(rcon_front_right, EVT_RCON_FRONT_RIGHT, false);
	event_manager_register_and_enable_x(rcon_front_right2, EVT_RCON_FRONT_RIGHT2, false);
	event_manager_register_and_enable_x(rcon_left, EVT_RCON_LEFT, false);
	event_manager_register_and_enable_x(rcon_right, EVT_RCON_RIGHT, false);
*/

	/* Over Current */
	event_manager_enable_handler(EVT_OVER_CURRENT_BRUSH_LEFT, true);
	event_manager_register_and_enable_x(over_current_brush_main, EVT_OVER_CURRENT_BRUSH_MAIN, true);
	event_manager_enable_handler(EVT_OVER_CURRENT_BRUSH_RIGHT, true);
	event_manager_register_and_enable_x(over_current_wheel_left, EVT_OVER_CURRENT_WHEEL_LEFT, true);
	event_manager_register_and_enable_x(over_current_wheel_right, EVT_OVER_CURRENT_WHEEL_RIGHT, true);
	event_manager_register_and_enable_x(over_current_suction, EVT_OVER_CURRENT_SUCTION, true);

	/* Key */
	event_manager_register_and_enable_x(key_clean, EVT_KEY_CLEAN, true);

	/* Remote */
	event_manager_register_and_enable_x(remote_clean, EVT_REMOTE_CLEAN, true);
	event_manager_register_and_enable_x(remote_home, EVT_REMOTE_HOME, true);
	event_manager_register_and_enable_x(remote_wallfollow,EVT_REMOTE_WALL_FOLLOW, true);
	event_manager_register_and_enable_x(remote_spot, EVT_REMOTE_SPOT, true);
	event_manager_register_and_enable_x(remote_max, EVT_REMOTE_MAX, true);
	event_manager_register_and_enable_x(remote_direction, EVT_REMOTE_DIRECTION_RIGHT, true);
	event_manager_register_and_enable_x(remote_direction, EVT_REMOTE_DIRECTION_LEFT, true);
	event_manager_register_and_enable_x(remote_direction, EVT_REMOTE_DIRECTION_FORWARD, true);

	// Just enable the default handler.
	event_manager_enable_handler(EVT_REMOTE_PLAN, true);
	event_manager_enable_handler(EVT_REMOTE_DIRECTION_FORWARD, true);
	event_manager_enable_handler(EVT_REMOTE_DIRECTION_LEFT, true);
	event_manager_enable_handler(EVT_REMOTE_DIRECTION_RIGHT, true);
	event_manager_enable_handler(EVT_REMOTE_WALL_FOLLOW, true);

	/* Battery */
	event_manager_register_and_enable_x(battery_home, EVT_BATTERY_HOME, true);
	event_manager_register_and_enable_x(battery_low, EVT_BATTERY_LOW, true);

	/* Charge Status */
	event_manager_register_and_enable_x(charge_detect, EVT_CHARGE_DETECT, true);
	/* robot stuck */
	event_manager_enable_handler(EVT_ROBOT_SLIP,true);
	/* Slam Error */
	event_manager_enable_handler(EVT_SLAM_ERROR, true);

#undef event_manager_register_and_enable_x

	event_manager_set_enable(true);
}

void cm_unregister_events()
{
	ROS_INFO("%s %d: Unregister events", __FUNCTION__, __LINE__);
#define event_manager_register_and_disable_x(x) \
	event_manager_register_handler(x, NULL); \
	event_manager_enable_handler(x, false);

	/* Bumper */
	event_manager_register_and_disable_x(EVT_BUMPER_ALL);
	event_manager_register_and_disable_x(EVT_BUMPER_LEFT);
	event_manager_register_and_disable_x(EVT_BUMPER_RIGHT);

	/* OBS */
	event_manager_register_and_disable_x(EVT_OBS_FRONT);
	//event_manager_register_and_disable_x(EVT_OBS_LEFT);
	//event_manager_register_and_disable_x(EVT_OBS_RIGHT);

	/* Cliff */
	event_manager_register_and_disable_x(EVT_CLIFF_ALL);
	event_manager_register_and_disable_x(EVT_CLIFF_FRONT_LEFT);
	event_manager_register_and_disable_x(EVT_CLIFF_FRONT_RIGHT);
	event_manager_register_and_disable_x(EVT_CLIFF_LEFT_RIGHT);
	event_manager_register_and_disable_x(EVT_CLIFF_FRONT);
	event_manager_register_and_disable_x(EVT_CLIFF_LEFT);
	event_manager_register_and_disable_x(EVT_CLIFF_RIGHT);

	/* RCON */
	event_manager_register_and_disable_x(EVT_RCON);
/*
	event_manager_register_and_disable_x(EVT_RCON_FRONT_LEFT);
	event_manager_register_and_disable_x(EVT_RCON_FRONT_LEFT2);
	event_manager_register_and_disable_x(EVT_RCON_FRONT_RIGHT);
	event_manager_register_and_disable_x(EVT_RCON_FRONT_RIGHT2);
	event_manager_register_and_disable_x(EVT_RCON_LEFT);
	event_manager_register_and_disable_x(EVT_RCON_RIGHT);
*/

	/* Over Current */
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_BRUSH_LEFT);
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_BRUSH_MAIN);
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_BRUSH_RIGHT);
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_WHEEL_LEFT);
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_WHEEL_RIGHT);
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_SUCTION);

	/* Key */
	event_manager_register_and_disable_x(EVT_KEY_CLEAN);

	/* Remote */
	event_manager_register_and_disable_x(EVT_REMOTE_PLAN);
	event_manager_register_and_disable_x(EVT_REMOTE_CLEAN);
	event_manager_register_and_disable_x(EVT_REMOTE_HOME);
	event_manager_register_and_disable_x(EVT_REMOTE_SPOT);
	event_manager_register_and_disable_x(EVT_REMOTE_MAX);
	event_manager_register_and_disable_x(EVT_REMOTE_DIRECTION_LEFT);
	event_manager_register_and_disable_x(EVT_REMOTE_DIRECTION_RIGHT);
	event_manager_register_and_disable_x(EVT_REMOTE_DIRECTION_FORWARD);

	/* Battery */
	event_manager_register_and_disable_x(EVT_BATTERY_HOME);
	event_manager_register_and_disable_x(EVT_BATTERY_LOW);

	/* Charge Status */
	event_manager_register_and_disable_x(EVT_CHARGE_DETECT);

	/* Slam Error */
	event_manager_register_and_disable_x(EVT_SLAM_ERROR);
	/* robot slip */
	//event_manager_register_and_disable_x(EVT_ROBOT_SLIP);

#undef event_manager_register_and_disable_x

	event_manager_set_enable(false);
}

void cm_set_event_manager_handler_state(bool state)
{
	event_manager_enable_handler(EVT_BUMPER_ALL, state);
	event_manager_enable_handler(EVT_BUMPER_LEFT, state);
	event_manager_enable_handler(EVT_BUMPER_RIGHT, state);

	event_manager_enable_handler(EVT_OBS_FRONT, state);
	//event_manager_enable_handler(EVT_OBS_LEFT, state);
	//event_manager_enable_handler(EVT_OBS_RIGHT, state);

	event_manager_enable_handler(EVT_RCON, state);
/*
	event_manager_enable_handler(EVT_RCON_FRONT_LEFT, state);
	event_manager_enable_handler(EVT_RCON_FRONT_LEFT2, state);
	event_manager_enable_handler(EVT_RCON_FRONT_RIGHT, state);
	event_manager_enable_handler(EVT_RCON_FRONT_RIGHT2, state);
	event_manager_enable_handler(EVT_RCON_LEFT, state);
	event_manager_enable_handler(EVT_RCON_RIGHT, state);
*/
}

void cm_event_manager_turn(bool state)
{
	event_manager_enable_handler(EVT_BUMPER_ALL, state);
	event_manager_enable_handler(EVT_BUMPER_LEFT, state);
	event_manager_enable_handler(EVT_BUMPER_RIGHT, state);
}

void cm_handle_bumper_all(bool state_now, bool state_last)
{
//	g_bumper_triggered = AllBumperTrig;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		set_wheel_speed(0, 0);
//	}

//	if (g_move_back_finished && !g_bumper_jam)
//		ROS_WARN("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, get_bumper_status(), state_now ? "true" : "false", state_last ? "true" : "false");

}

void cm_handle_bumper_left(bool state_now, bool state_last)
{
//	if(g_bumper_triggered != 0)
//		return;
//	g_bumper_triggered = LeftBumperTrig;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		set_wheel_speed(0, 0);
//	}

//	if (g_move_back_finished && !g_bumper_jam)
//		ROS_WARN("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, get_bumper_status(), state_now ? "true" : "false", state_last ? "true" : "false");
}

void cm_handle_bumper_right(bool state_now, bool state_last)
{
//	if(g_bumper_triggered != 0)
//		return;
//
//	g_bumper_triggered = RightBumperTrig;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		set_wheel_speed(0, 0);
//	}

//	if (g_move_back_finished && !g_bumper_jam)
//		ROS_WARN("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, get_bumper_status(), state_now ? "true" : "false", state_last ? "true" : "false");
}

/* OBS */
void cm_handle_obs_front(bool state_now, bool state_last)
{
//	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
//	g_obs_triggered = Status_Front_OBS;
}

void cm_handle_obs_left(bool state_now, bool state_last)
{
//	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
//	g_obs_triggered = Status_Left_OBS;
}

void cm_handle_obs_right(bool state_now, bool state_last)
{
//	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
//	g_obs_triggered = Status_Right_OBS;
}

/* Cliff */
void cm_handle_cliff_all(bool state_now, bool state_last)
{
	g_cliff_all_cnt++;
	if (g_cliff_all_cnt++ > 2)
	{
		g_cliff_all_triggered = true;
		g_fatal_quit_event = true;
	}
	g_cliff_triggered = Status_Cliff_All;
	if (g_move_back_finished && !g_cliff_jam && !state_last)
		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void cm_handle_cliff_front_left(bool state_now, bool state_last)
{
//	g_cliff_all_triggered = false;
//	g_cliff_triggered = Status_Cliff_LF;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		set_wheel_speed(0, 0);
//	}

//	if (g_move_back_finished && !g_cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");

}

void cm_handle_cliff_front_right(bool state_now, bool state_last)
{
//	g_cliff_all_triggered = false;
//	g_cliff_triggered = Status_Cliff_RF;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		set_wheel_speed(0, 0);
//	}

//	if (g_move_back_finished && !g_cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");

}

void cm_handle_cliff_left_right(bool state_now, bool state_last)
{
//	g_cliff_all_triggered = false;
//	g_cliff_triggered = Status_Cliff_LR;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		set_wheel_speed(0, 0);
//	}
//
//	if (g_move_back_finished && !g_cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void cm_handle_cliff_front(bool state_now, bool state_last)
{
//	g_cliff_all_triggered = false;
//	g_cliff_triggered = Status_Cliff_Front;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		set_wheel_speed(0, 0);
//	}
//
//	if (g_move_back_finished && !g_cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void cm_handle_cliff_left(bool state_now, bool state_last)
{
//	g_cliff_all_triggered = false;
//	g_cliff_triggered = Status_Cliff_Left;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		set_wheel_speed(0, 0);
//	}

//	if (g_move_back_finished && !g_cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void cm_handle_cliff_right(bool state_now, bool state_last)
{
//	g_cliff_all_triggered = false;
//	g_cliff_triggered = Status_Cliff_Right;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		set_wheel_speed(0, 0);
//	}

//	if (g_move_back_finished && !g_cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

/* RCON */
void cm_handle_rcon(bool state_now, bool state_last)
{
/*
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_go_home) {
		ROS_DEBUG("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
		return;
	}
	if(mt_is_follow_wall()){
		if (!(get_rcon_status() & (RconL_HomeT | RconR_HomeT | RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT)))
			return;
	}
	else if (mt_is_linear())
		// Since we have front left 2 and front right 2 rcon receiver, seems it is not necessary to handle left or right rcon receives home signal.
		if (!(get_rcon_status() & (RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT)))
		return;

	g_rcon_triggered = get_rcon_trig_();
	if(g_rcon_triggered != 0){
		map_set_rcon();
	}
	reset_rcon_status();*/
}

/* Over Current */
//void cm_handle_over_current_brush_left(bool state_now, bool state_last)
//{
//	static uint8_t stop_cnt = 0;
//
//	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
//	if(!robot::instance()->getLbrushOc()) {
//		g_oc_brush_left_cnt = 0;
//		if (stop_cnt++ > 250) {
//			set_left_brush_pwm(30);
//		}
//		return;
//	}
//
//	stop_cnt = 0;
//	if (g_oc_brush_left_cnt++ > 40) {
//		g_oc_brush_left_cnt = 0;
//		set_left_brush_pwm(0);
//		ROS_WARN("%s %d: left brush over current", __FUNCTION__, __LINE__);
//	}
//}

void cm_handle_over_current_brush_main(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!robot::instance()->getMbrushOc()){
		g_oc_brush_main_cnt = 0;
		return;
	}

	if (g_oc_brush_main_cnt++ > 40) {
		g_oc_brush_main_cnt = 0;
		ROS_WARN("%s %d: main brush over current", __FUNCTION__, __LINE__);

		if (self_check(Check_Main_Brush) == 1) {
			g_oc_brush_main = true;
			g_fatal_quit_event = true;
		}
    }
}

//void cm_handle_over_current_brush_right(bool state_now, bool state_last)
//{
//	static uint8_t stop_cnt = 0;
//
//	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
//
//	if(!robot::instance()->getRbrushOc()) {
//		g_oc_brush_right_cnt = 0;
//		if (stop_cnt++ > 250) {
//			set_right_brush_pwm(30);
//		}
//		return;
//	}
//
//	stop_cnt = 0;
//	if (g_oc_brush_right_cnt++ > 40) {
//		g_oc_brush_right_cnt = 0;
//		set_right_brush_pwm(0);
//		ROS_WARN("%s %d: reft brush over current", __FUNCTION__, __LINE__);
//	}
//}

void cm_handle_over_current_wheel_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if ((uint32_t) robot::instance()->getLwheelCurrent() < Wheel_Stall_Limit) {
        g_oc_wheel_left_cnt = 0;
		return;
	}

	if (g_oc_wheel_left_cnt++ > 40){
		g_oc_wheel_left_cnt = 0;
		ROS_WARN("%s %d: left wheel over current, \033[1m%u mA\033[0m", __FUNCTION__, __LINE__, (uint32_t) robot::instance()->getLwheelCurrent());

		g_oc_wheel_left = true;
	}
}

void cm_handle_over_current_wheel_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if ((uint32_t) robot::instance()->getRwheelCurrent() < Wheel_Stall_Limit) {
		g_oc_wheel_right_cnt = 0;
		return;
	}

	if (g_oc_wheel_right_cnt++ > 40){
		g_oc_wheel_right_cnt = 0;
		ROS_WARN("%s %d: right wheel over current, \033[1m%u mA\033[0m", __FUNCTION__, __LINE__, (uint32_t) robot::instance()->getRwheelCurrent());

		g_oc_wheel_right = true;
	}
}

void cm_handle_over_current_suction(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!robot::instance()->getVacuumOc()) {
		g_oc_suction_cnt = 0;
		return;
	}

	if (g_oc_suction_cnt++ > 40) {
		g_oc_suction_cnt = 0;
		ROS_WARN("%s %d: vacuum over current", __FUNCTION__, __LINE__);

		g_oc_suction = true;
	}
}

/* Key */
void cm_handle_key_clean(bool state_now, bool state_last)
{
	time_t start_time;
	bool reset_manual_pause = false;

	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_slam_error)
	{
		beep_for_command(INVALID);
		while (get_key_press() & KEY_CLEAN)
		{
			usleep(20000);
		}
		ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);
		reset_touch();
		return;
	}

	beep_for_command(VALID);
	set_wheel_speed(0, 0);
	g_key_clean_pressed = true;

	if(SpotMovement::instance()->getSpotType() != NORMAL_SPOT && get_clean_mode() != Clean_Mode_WallFollow && get_clean_mode() != Clean_Mode_Exploration)
		robot::instance()->setManualPause();

	start_time = time(NULL);
	while (get_key_press() & KEY_CLEAN)
	{
		if (time(NULL) - start_time > 3) {
			if (!reset_manual_pause)
			{
				beep_for_command(VALID);
				reset_manual_pause = true;
				robot::instance()->resetManualPause();
				ROS_WARN("%s %d: Manual pause has been reset.", __FUNCTION__, __LINE__);
			}
		}
		else
			ROS_DEBUG("%s %d: Key clean is not released.", __FUNCTION__, __LINE__);
		usleep(20000);
	}

	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);
	reset_touch();
}

/* Remote */

void cm_handle_remote_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_slam_error)
	{
		beep_for_command(INVALID);
		reset_rcon_remote();
		return;
	}
	beep_for_command(VALID);
	g_key_clean_pressed = true;
	if(SpotMovement::instance()->getSpotType() != NORMAL_SPOT && get_clean_mode() != Clean_Mode_WallFollow && get_clean_mode() != Clean_Mode_Exploration){
		robot::instance()->setManualPause();
	}
	reset_rcon_remote();
}

void cm_handle_remote_home(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_motion_init_succeeded && !g_go_home && !cm_should_self_check() && !g_slam_error && !robot::instance()->isManualPaused()) {

		if( SpotMovement::instance()->getSpotType()  == NORMAL_SPOT){
			beep_for_command(INVALID);
		}
		else{
			g_remote_home = true;
			beep_for_command(VALID);
			if (SpotMovement::instance()->getSpotType() == CLEAN_SPOT){
				SpotMovement::instance()->spotDeinit();
			}
		}
		ROS_INFO("g_remote_home = %d", g_remote_home);
	}
	else {
		beep_for_command(INVALID);
		ROS_INFO("g_remote_home = %d", g_remote_home);
	}
	reset_rcon_remote();
}

void cm_handle_remote_spot(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!g_motion_init_succeeded || get_clean_mode() != Clean_Mode_Navigation
		|| g_go_home || cm_should_self_check() || g_slam_error || robot::instance()->isManualPaused()
		|| time(NULL) - last_time_remote_spot < 3)
		beep_for_command(INVALID);
	else
	{
		g_remote_spot = true;
		last_time_remote_spot = time(NULL);
		beep_for_command(VALID);
	}

	reset_rcon_remote();
}

void cm_handle_remote_wallfollow(bool state_now,bool state_last)
{
	ROS_WARN("%s,%d: is called.",__FUNCTION__,__LINE__);
	beep_for_command(INVALID);
	reset_rcon_remote();
}

void cm_handle_remote_max(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_motion_init_succeeded && !g_go_home && !cm_should_self_check() && SpotMovement::instance()->getSpotType() == NO_SPOT && !g_slam_error && !robot::instance()->isManualPaused())
	{
		beep_for_command(VALID);
		switch_vac_mode(true);
	}
	else
		beep_for_command(INVALID);
	reset_rcon_remote();
}

void cm_handle_remote_direction(bool state_now,bool state_last)
{
	ROS_WARN("%s,%d: is called.",__FUNCTION__,__LINE__);
	// For Debug
	// g_battery_home = true;
	// path_set_continue_cell(map_get_curr_cell());
	// robot::instance()->setLowBatPause();
	beep_for_command(INVALID);
	reset_rcon_remote();
}

/* Battery */
void cm_handle_battery_home(bool state_now, bool state_last)
{
	if (g_motion_init_succeeded && ! g_go_home) {
		ROS_INFO("%s %d: low battery, battery =\033[33m %dmv \033[0m", __FUNCTION__, __LINE__,
						 robot::instance()->getBatteryVoltage());
		g_battery_home = true;

		if (get_vac_mode() == Vac_Max) {
			switch_vac_mode(false);
		}
#if CONTINUE_CLEANING_AFTER_CHARGE
		if (SpotMovement::instance()->getSpotType() != NORMAL_SPOT ){
			path_set_continue_cell(map_get_curr_cell());
			robot::instance()->setLowBatPause();
		}
#endif 
    }
}

void cm_handle_battery_low(bool state_now, bool state_last)
{
    uint8_t         v_pwr, s_pwr, m_pwr;
    uint16_t        t_vol;

    ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_battery_low_cnt++ > 50) {
		t_vol = get_battery_voltage();
		ROS_WARN("%s %d: low battery, battery < %umv is detected.", __FUNCTION__, __LINE__,t_vol);

		if (g_go_home) {
			v_pwr = Home_Vac_Power / t_vol;
			s_pwr = Home_SideBrush_Power / t_vol;
			m_pwr = Home_MainBrush_Power / t_vol;
		} else {
			v_pwr = Clean_Vac_Power / t_vol;
			s_pwr = Clean_SideBrush_Power / t_vol;
			m_pwr = Clean_MainBrush_Power / t_vol;
		}

		g_battery_low_cnt = 0;
		set_bldc_speed(v_pwr);
		set_side_brush_pwm(s_pwr, s_pwr);
		set_main_brush_pwm(m_pwr);

		g_fatal_quit_event = true;
		g_battery_low = true;
	}
}

void cm_handle_charge_detect(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: Detect charger: %d, g_charge_detect_cnt: %d.", __FUNCTION__, __LINE__, robot::instance()->getChargeStatus(), g_charge_detect_cnt);
	if (((get_clean_mode() == Clean_Mode_Exploration || g_go_home) && robot::instance()->getChargeStatus()) ||
		(get_clean_mode() != Clean_Mode_Exploration && robot::instance()->getChargeStatus() == 3))
	{
		if (g_charge_detect_cnt++ > 2)
		{
			g_charge_detect = robot::instance()->getChargeStatus();
			g_fatal_quit_event = true;
			ROS_WARN("%s %d: g_charge_detect has been set to %d.", __FUNCTION__, __LINE__, g_charge_detect);
			g_charge_detect_cnt = 0;
		}
	}
}
