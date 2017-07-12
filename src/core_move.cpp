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

#include <wav.h>
//#include "../include/obstacle_detector.h"
#include <motion_manage.h>
#include <slam.h>
#include <event_manager.h>
#include <mathematics.h>
#include "wall_follow_slam.h"
#include <move_type.h>
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

bool g_have_seen_charge_stub = false;
uint16_t g_turn_angle;
uint16_t g_wall_distance=20;
uint16_t g_straight_distance;
uint32_t g_escape_trapped_timer;

extern int g_trapped_mode;
int jam=0;
bool g_should_follow_wall;
//std::vector<int16_t> g_left_buffer;
//std::vector<int16_t> g_right_buffer;

Point32_t g_next_point, g_target_point;

// This list is for storing the position that robot sees the charger stub.
std::list <Point32_t> g_home_point_old_path;
std::list <Point32_t> g_home_point_new_path;

// This is for the continue point for robot to go after charge.
Point32_t g_continue_point;

//uint8_t	g_remote_go_home = 0;
bool	g_go_home = false;
bool	g_from_station = 0;
int16_t g_map_gyro_offset = 0;

// This flag is for checking whether map boundary is created.
bool g_map_boundary_created = false;

// This g_pnt16_ar_tmp is for trapped reference point.
Cell_t g_pnt16_ar_tmp[3];

Cell_t g_relativePos[MOVE_TO_CELL_SEARCH_ARRAY_LENGTH * MOVE_TO_CELL_SEARCH_ARRAY_LENGTH] = {{0, 0}};

extern Cell_t g_cell_history[];

extern int16_t g_x_min, g_x_max, g_y_min, g_y_max;

// This status is for rounding function to decide the angle it should turn.
uint8_t g_bumper_status_for_rounding;

// Saved position for move back.
float saved_pos_x, saved_pos_y;

// Flag for indicating whether move back has finished.
bool g_move_back_finished = true;

// Flag for indicating whether motion instance is initialized successfully.
bool g_motion_init_succeeded = false;

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

// Target:	robot coordinate
static bool check_map_boundary(bool& slow_down)
{
		int32_t		x, y;
		for (auto dy = -1; dy <= 1; dy++) {
			cm_world_to_point(Gyro_GetAngle(), dy * CELL_SIZE, CELL_SIZE_3, &x, &y);
			if (map_get_cell(MAP, count_to_cell(x), count_to_cell(y)) == BLOCKED_BOUNDARY)
				slow_down = true;

			cm_world_to_point(Gyro_GetAngle(), dy * CELL_SIZE, CELL_SIZE_2, &x, &y);
			if (map_get_cell(MAP, count_to_cell(x), count_to_cell(y)) == BLOCKED_BOUNDARY) {
				ROS_INFO("%s, %d: Blocked boundary.", __FUNCTION__, __LINE__);
				stop_brifly();
				return true;
			}
		}
	return false;
}

static double radius_of(Cell_t cell_0,Cell_t cell_1)
{
	return (abs(cell_to_count(cell_0.X - cell_1.X)) + abs(cell_to_count(cell_0.Y - cell_1.Y))) / 2;
}

void cm_world_to_point(uint16_t heading, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y)
{
	*x = cell_to_count(count_to_cell(map_get_relative_x(heading, offset_lat, offset_long)));
	*y = cell_to_count(count_to_cell(map_get_relative_y(heading, offset_lat, offset_long)));
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

void cm_update_map_cleaned()
{
	int32_t i,j;
	for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; ++dy) {
        for (auto dx = -ROBOT_SIZE_1_2; dx <= ROBOT_SIZE_1_2; ++dx){
					cm_world_to_point(Gyro_GetAngle(), CELL_SIZE * dy, CELL_SIZE * dx, &i, &j);
            map_set_cell(MAP, i, j, CLEANED);
        }
	}
}

void cm_update_map_obs()
{
	if (get_obs_status() & Status_Left_OBS) {
		int32_t i,j;
		cm_world_to_point(Gyro_GetAngle(), CELL_SIZE_2, CELL_SIZE, &i, &j);
		if (get_wall_adc(0) > 200) {
			if (map_get_cell(MAP, count_to_cell(i), count_to_cell(j)) != BLOCKED_BUMPER) {
				map_set_cell(MAP, i, j, BLOCKED_OBS); //BLOCKED_OBS);
			}
		}
	}

	if (get_obs_status() & Status_Right_OBS) {
		int32_t i,j;
		cm_world_to_point(Gyro_GetAngle(), -CELL_SIZE_2, CELL_SIZE, &i, &j);
		if (get_wall_adc(1) > 200) {
			if (map_get_cell(MAP, count_to_cell(i), count_to_cell(j)) != BLOCKED_BUMPER) {
				map_set_cell(MAP, i, j, BLOCKED_OBS); //BLOCKED_OBS);
			}
		}
	}

	for (auto dy = 0; dy < 3; ++dy) {
		auto i = SHRT_MAX;
		switch (dy) {
			case 0:
				i = get_obs_status() & Status_Right_OBS;
				break;
			case 1:
				i = get_obs_status() & Status_Front_OBS;
				break;
			case 2:
				i = get_obs_status() & Status_Left_OBS;
				break;
		}
		int32_t x_tmp, y_tmp;
		cm_world_to_point(Gyro_GetAngle(), (dy - 1) * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
		if (i) {
			if (map_get_cell(MAP, count_to_cell(x_tmp), count_to_cell(y_tmp)) != BLOCKED_BUMPER) {
				map_set_cell(MAP, x_tmp, y_tmp, BLOCKED_OBS);
			}
		} else {
			if (map_get_cell(MAP, count_to_cell(x_tmp), count_to_cell(y_tmp)) == BLOCKED_OBS) {
				map_set_cell(MAP, x_tmp, y_tmp, UNCLEAN);
			}
		}
	}
}

void cm_update_map_bumper()
{
	if (g_bumper_jam)
		// During self check.
		return;

	auto bumper = get_bumper_status();
	std::vector<Cell_t> d_cells;

	if ((bumper & RightBumperTrig) && (bumper & LeftBumperTrig))
		d_cells = {{2,-1}, {2,0}, {2,1}};
	else if (bumper & LeftBumperTrig) {
		d_cells = {{2, 1}, {2,2},{1,2}};
		if (g_cell_history[0] == g_cell_history[1] && g_cell_history[0] == g_cell_history[2])
			d_cells.push_back({2,0});
	} else if (bumper & RightBumperTrig) {
		d_cells = {{2,-2},{2,-1},{1,-2}};
		if (g_cell_history[0] == g_cell_history[1]  && g_cell_history[0] == g_cell_history[2])
			d_cells.push_back({2,0});
	}

	int32_t	x_tmp, y_tmp;
	for(auto& d_cell : d_cells){
		cm_world_to_point(Gyro_GetAngle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x_tmp, &y_tmp);
		ROS_DEBUG("%s %d: marking (%d, %d)", __FUNCTION__, __LINE__, count_to_cell(x_tmp), count_to_cell(y_tmp));
		map_set_cell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
	}
}

void cm_update_map_cliff()
{
	if (g_cliff_jam)
		// During self check.
		return;

	std::vector<Cell_t> d_cells;
	if (get_cliff_trig() & Status_Cliff_Front){
		d_cells.push_back({2,-1});
		d_cells.push_back({2, 0});
		d_cells.push_back({2, 1});
	}
	if (get_cliff_trig() & Status_Cliff_Left){
		d_cells.push_back({2, 1});
		d_cells.push_back({2, 2});
	}
	if (get_cliff_trig() & Status_Cliff_Right){
		d_cells.push_back({2,-1});
		d_cells.push_back({2,-2});
	}

	int32_t	x_tmp, y_tmp;
	for (auto& d_cell : d_cells) {
		cm_world_to_point(Gyro_GetAngle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x_tmp, &y_tmp);
		ROS_DEBUG("%s %d: marking (%d, %d)", __FUNCTION__, __LINE__, count_to_cell(x_tmp), count_to_cell(y_tmp));
		map_set_cell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
	}
}

Cell_t cm_update_position(bool is_turn)
{
	auto pos_x = robot::instance()->getPositionX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	auto pos_y = robot::instance()->getPositionY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	map_set_position(pos_x, pos_y);
	return map_get_curr_cell();
}

void cm_update_map()
{
	auto last = map_get_curr_cell();

	auto curr = cm_update_position();

//	ROS_WARN("1 last(%d,%d),curr(%d,%d)",last.X, last.Y, curr.X, curr.Y);

	cm_update_map_bumper();

	cm_update_map_cliff();

	cm_update_map_cleaned();

	cm_update_map_obs();
//	ROS_ERROR("2 last(%d,%d),curr(%d,%d)",last.X, last.Y,curr.X,curr.Y);
	if (last != curr || get_bumper_status()!=0 || get_cliff_trig() !=0 || get_obs_status() != 0)
		MotionManage::pubCleanMapMarkers(MAP, g_next_point, g_target_point);

	{
		Cell_t next,target;
		if(last != curr && g_trapped_mode == 1 && path_target(next, target) == 1)
			g_trapped_mode = 2;
	}

}
//-------------------------------cm_move_back-----------------------------//
/*uint16_t bumper_turn_angle()
{
	if(get_bumper_status()==AllBumperTrig || get_cliff_trig() == FrontCliffTrig || get_cliff_trig() == RightLeftCliffTrig)
			return 900;
		else if(get_bumper_status()==LeftBumperTrig || get_cliff_trig() == LeftCliffTrig)
			return (g_cm_move_type == CM_FOLLOW_LEFT_WALL) ? 300 : 1350; //left
		else if(get_bumper_status()==RightBumperTrig || get_cliff_trig() == RightCliffTrig)
			return (g_cm_move_type == CM_FOLLOW_LEFT_WALL) ? 1350 : 300; //right
		else if(get_cliff_trig() == LeftFrontCliffTrig)
			return (g_cm_move_type == CM_FOLLOW_LEFT_WALL) ? 1350 : 300; //right
		else*//* (get_cliff_trig() == RightFrontCliffTrig)*//*
			return (g_cm_move_type == CM_FOLLOW_LEFT_WALL) ? 1350 : 600;
}*/
uint16_t bumper_turn_angle()
{
	auto get_wheel_step = (mt_is_left()) ? get_right_wheel_step : get_left_wheel_step;
	auto get_obs = (mt_is_left()) ? get_left_obs : get_right_obs;
	auto get_obs_value = (mt_is_left()) ? get_left_obs_value : get_right_obs_value;
	auto status = get_bumper_status();
	auto diff_side = (mt_is_left()) ? RightBumperTrig : LeftBumperTrig;
	auto same_side = (mt_is_left()) ? LeftBumperTrig : RightBumperTrig;

	if (status == AllBumperTrig)
	{
		g_turn_angle = 850;
		g_straight_distance = 150; //150;
		g_wall_distance = std::min(g_wall_distance + 300, Wall_High_Limit);
		jam = get_wheel_step() < 2000 ? ++jam : 0;
	} else if (status == diff_side)
	{
		g_wall_distance = std::min(g_wall_distance + 300, Wall_High_Limit);
		g_turn_angle = 920;
	} else if (status == same_side)
	{
		g_wall_distance = std::max(g_wall_distance - 100, Wall_Low_Limit);

		g_turn_angle = (jam >= 3 || (g_wall_distance < 200 && get_obs() <= get_obs_value() - 200)) ? 200 : 300;

		g_wall_distance = (jam < 3 && g_wall_distance<200 && get_obs()>(get_obs_value() - 200))
											? Wall_High_Limit : g_wall_distance;

		g_straight_distance = 250; //250;
		jam = get_wheel_step() < 2000 ? ++jam : 0;
	}

	status = get_cliff_trig();
	if(status != 0)
		g_turn_angle = 750;

	g_straight_distance = 200;
//	g_left_buffer = {0, 0, 0};
	reset_wheel_step();
//	ROS_WARN("705, g_turn_angle(%d), g_straight_distance(%d),g_wall_distance(%d),jam(%d),get_right_wheel_step(%d) ", g_turn_angle,     g_straight_distance,    g_wall_distance,    jam,    get_right_wheel_step()  );
	return g_turn_angle;
}
int double_scale_10(double line_angle)
{
	int angle;
	if (line_angle > 0)
	{
		angle = int((180 - line_angle) * 10);
	} else
	{
		angle = int(fabs(line_angle) * 10);
	}
	return angle;
}
bool _laser_turn_angle(int laser_min, int laser_max, int angle_min,int angle_max,double dis_limit=0.217)
{
	ROS_WARN("bumper (%d)!", get_bumper_status());
	bool is_found;
	double line_angle;
	const auto RESET_WALL_DIS = 100;
	is_found = MotionManage::s_laser->getLaserDistance(laser_min, laser_max, -1.0, dis_limit, &line_angle);
	ROS_WARN("line_angle_raw = %lf", line_angle);
	uint16_t angle = double_scale_10(line_angle);

	if (mt_is_right())
		angle  = 1800-angle;

	ROS_WARN("line_angle = %d", angle);
	if (is_found && angle >= angle_min && angle < angle_max)
	{
		g_turn_angle = angle;
		g_wall_distance = RESET_WALL_DIS;
		ROS_WARN("laser generate turn angle!");
		return true;
	}
	return false;
}

bool laser_turn_angle()
{
 auto obs_status = get_front_obs() >= get_front_obs_value();
	stop_brifly();

	if (obs_status)
	{
		ROS_ERROR("front obs trigger");
		return _laser_turn_angle(90, 270, 450, 1800, 0.25);
	}

	uint8_t status = angle_to_bumper_status();
	int angle_min, angle_max;
	if (mt_is_left() ^ status == LeftBumperTrig)
	{
		angle_min = 450;
		angle_max = 1800;
	}else {
		angle_min = 200;
		angle_max = 900;
	}

	if (status == AllBumperTrig)
	{
		return _laser_turn_angle(90, 270, 900, 1800);
	} else if (status == RightBumperTrig)
	{

		return _laser_turn_angle(90, 180, angle_min, angle_max);
	} else if (status == LeftBumperTrig)
	{
		return _laser_turn_angle(180, 270, angle_min, angle_max);
	}
	return false;
}

uint8_t angle_to_bumper_status(void)
{
	auto diff_side = (mt_is_left()) ? RightBumperTrig : LeftBumperTrig;
	auto same_side = (mt_is_left()) ? LeftBumperTrig : RightBumperTrig;
	if (g_turn_angle == 850)
		return AllBumperTrig;
	else if (g_turn_angle == 920)
		return diff_side;
	else if (g_turn_angle == 0)
		return 0;
	return same_side;
}

uint16_t round_turn_distance()
{
	auto wall = robot::instance()->getLeftWall();
	if (mt_is_left())
		wall = robot::instance()->getRightWall();

	auto distance = wall > (Wall_Low_Limit) ? wall / 3 : g_wall_distance + 200;
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
void cm_head_to_course(uint8_t speed_max, int16_t angle)
{
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
			|| g_bumper_hitted || g_cliff_triggered
			)
			break;

		auto diff = ranged_angle(angle - Gyro_GetAngle());

		if (std::abs(diff) < 10) {
			stop_brifly();
			ROS_INFO("%s %d: angle: %d\tGyro: %d\tDiff: %d", __FUNCTION__, __LINE__, angle, Gyro_GetAngle(), diff);
			break;
		}
		uint8_t speed_up;
		regulator.adjustSpeed(diff, speed_up);
		set_wheel_speed(speed_up, speed_up);

	}

	cm_event_manager_turn(false);
	set_wheel_speed(0, 0);
}

bool cm_linear_move_to_point(Point32_t Target, int32_t speed_max)
{
	// Reset the g_bumper_status_for_rounding.
	g_should_follow_wall = false;
	g_bumper_status_for_rounding = 0;
	g_obs_triggered = g_rcon_triggered = false;
	g_move_back_finished = true;
	g_bumper_hitted =  g_cliff_triggered = false;
	bool rotate_is_needed_ = true;
	robotbase_obs_adjust_count(50);
	reset_rcon_status();
	cm_set_event_manager_handler_state(true);

	LinearRegulator regulator(speed_max);
	bool	eh_status_now=false, eh_status_last=false;
	while (ros::ok) {
		wall_dynamic_base(50);
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}

		if (g_fatal_quit_event || g_key_clean_pressed || (!g_go_home && g_remote_home) || g_remote_spot)
			break;

		if (!rotate_is_needed_ && (g_obs_triggered || g_rcon_triggered)) {
			g_should_follow_wall = true;
			ROS_WARN("%s,%d,obs or rcon trigger",__FUNCTION__,__LINE__);
			SpotType spt = SpotMovement::instance() -> getSpotType();
			if(spt == CLEAN_SPOT || spt == NORMAL_SPOT)
				SpotMovement::instance()->setDirectChange();
			break;
		}

		if (std::abs(map_get_x_count() - Target.X) < 150 && std::abs(map_get_y_count() - Target.Y) < 150) {
			ROS_DEBUG("%s, %d: Reach target.", __FUNCTION__, __LINE__);
			break;
		}

		if (g_cliff_all_cnt >= 2)
		{
			ROS_WARN("%s %d: Robot lifted up.", __FUNCTION__, __LINE__);
			g_fatal_quit_event = true;
			break;
		}

		bool slow_down=false;
		if(check_map_boundary(slow_down))
			break;

		if (g_bumper_hitted || g_cliff_triggered)
		{

			if (g_bumper_hitted){
				ROS_WARN("%s,%d ,bumper hitted",__FUNCTION__,__LINE__);
				SpotType spt = SpotMovement::instance() -> getSpotType();
				if(spt == CLEAN_SPOT || spt == NORMAL_SPOT)
					SpotMovement::instance()->setDirectChange();
			}

			g_move_back_finished = false;
			float distance;
			distance = sqrtf(powf(saved_pos_x - robot::instance()->getOdomPositionX(), 2) + powf(saved_pos_y - robot::instance()->getOdomPositionY(), 2));
			if (fabsf(distance) > 0.02f)
			{
				if (g_bumper_hitted)
				{
					// Check if still bumper triggered.
					if(!get_bumper_status())
					{
						ROS_INFO("%s %d: Move back for bumper finished.", __FUNCTION__, __LINE__);
						g_move_back_finished = true;
						g_bumper_hitted = false;
						g_bumper_cnt = 0;
						g_should_follow_wall = true;
						break;
					}
					else if (++g_bumper_cnt >= 2)
					{
						// Should switch to cm_self_check() function to resume.
						g_bumper_jam = true;
						break;
					}
					else
					{
						// Move back for one more time.
						ROS_WARN("%s %d: Move back for one more time.", __FUNCTION__, __LINE__);
						saved_pos_x = robot::instance()->getOdomPositionX();
						saved_pos_y = robot::instance()->getOdomPositionY();
						continue;
					}
				}
				else
				{
					if (!get_cliff_trig())
					{
						ROS_INFO("%s %d: Move back for cliff finished.", __FUNCTION__, __LINE__);
						g_move_back_finished = true;
						g_cliff_triggered = false;
						g_cliff_all_triggered = false;
						g_cliff_cnt = 0;
						g_cliff_all_cnt = 0;
						break;
					}
					else if (++g_cliff_cnt >= 2)
					{
						// Should switch to cm_self_check() function to resume.
						g_cliff_jam = true;
						break;
					}
					else
					{
						// Move back for one more time.
						ROS_WARN("%s %d: Move back for one more time.", __FUNCTION__, __LINE__);
						saved_pos_x = robot::instance()->getOdomPositionX();
						saved_pos_y = robot::instance()->getOdomPositionY();
						continue;
					}
				}
			}
		}
		else if (std::abs(map_get_x_count() - Target.X) < 150 && std::abs(map_get_y_count() - Target.Y) < 150) {
			ROS_INFO("%s, %d: Reach target.", __FUNCTION__, __LINE__);
			break;
		}

		if(!regulator.adjustSpeed(Target, slow_down, rotate_is_needed_))
			break;
	}

	cm_set_event_manager_handler_state(false);

	return true;
}

/*
void cm_move_to_point(Point32_t target)
{
	if (path_get_path_points_count() >= 3){
		if (!cm_curve_move_to_point())
			cm_linear_move_to_point(target, RUN_TOP_SPEED, true, true);
	}
	else
		cm_linear_move_to_point(target, RUN_TOP_SPEED, true, true);
}
*/

bool cm_turn_move_to_point(Point32_t Target, uint8_t speed_left, uint8_t speed_right)
{
	auto angle_start = Gyro_GetAngle();
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
			|| g_bumper_hitted || g_obs_triggered || g_cliff_triggered || g_rcon_triggered
			|| (!g_go_home && (g_battery_home || g_remote_home))
			|| g_remote_spot || g_remote_direction_keys)
			return false;

		auto angle_diff = ranged_angle(Gyro_GetAngle() - angle_start);
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
	if(!cm_linear_move_to_point(target, MAX_SPEED) )
		return false;

	//2/3 calculate the curve speed.
	auto speed = (uint8_t) ceil(MAX_SPEED * (radius / CELL_COUNT - WHEEL_BASE_HALF) / (radius / CELL_COUNT + WHEEL_BASE_HALF));
	uint8_t speed_left = MAX_SPEED;
	uint8_t speed_right = MAX_SPEED;
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
*/
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
	if(!cm_linear_move_to_point(target, MAX_SPEED))
		return false;

	return true;
}

int16_t uranged_angle(int16_t angle)
{
	while (angle >= 3600 || angle < 0)
	{
		if (angle >= 3600) {
			angle -= 3600;
		} else
		if (angle < 0) {
			angle += 3600;
		}
	}
	return angle;
}

int16_t calc_target(int16_t)
{
	auto angle = (mt_is_left()) ? -g_turn_angle : g_turn_angle;
	return uranged_angle(Gyro_GetAngle() + angle);
}

int16_t get_round_angle(CMMoveType type){
	int16_t		angle;
	auto status = get_bumper_status();
	if ((status & LeftBumperTrig) && !(status & RightBumperTrig)) {
		angle = (mt_is_left()) ? 450 : 1350;
	} else if (!(status & LeftBumperTrig) && (status & RightBumperTrig)) {
		angle = (mt_is_left()) ? 1350 : 450;
	} else {
		angle = 900;
	}
	return angle;
}

uint8_t cm_follow_wall(Point32_t target)
{
	g_wall_distance = 400;
	bool	eh_status_now=false, eh_status_last=false;
	cm_set_event_manager_handler_state(true);
	g_straight_distance = 300;
	RegulatorProxy regulator({map_get_x_count(),map_get_y_count()}, target);
	robotbase_obs_adjust_count(100);
	while (ros::ok())
	{
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1)
		{
			usleep(100);
			continue;
		}

		if (regulator.isExit())
			break;

		if (regulator.isSwitch())
			regulator.switchToNext();

		int32_t	 speed_left = 0, speed_right = 0;
		regulator.adjustSpeed(speed_left, speed_right);
		set_wheel_speed(speed_left, speed_right);
	}

	cm_set_event_manager_handler_state(false);
	return 0;
}

bool cm_resume_cleaning()
{
	robot::instance()->resetLowBatPause();

	cm_move_to_cell(count_to_cell(g_continue_point.X), count_to_cell(g_continue_point.Y));
	if (g_fatal_quit_event || g_key_clean_pressed)
	{
		robot::instance()->resetLowBatPause();
		return false;
	}
	return true;
}

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
			for (auto x = start_x; x<=stop_x+1; x++)
			{
				auto y = (int16_t) (slop * (stop.X) + intercept);
				for(auto dy=-ROBOT_SIZE_1_2;dy<=ROBOT_SIZE_1_2;dy++)
					map_set_cell(MAP, cell_to_count(x), cell_to_count(y + dy), CLEANED);
			}
		}
	}
}

int cm_cleaning()
{
	// Checking g_go_home is for case that manual pause when robot going home.
	if(g_go_home)
	{
		ROS_WARN("%s %d: Continue going home.", __FUNCTION__, __LINE__);
		return 0;
	}
	set_explore_new_path_flag(true);
	while (ros::ok())
	{
		if (g_key_clean_pressed || g_fatal_quit_event)
			return -1;

		if (g_remote_home || g_battery_home)
		{
			g_remote_home = false;
			g_go_home = true;
			ROS_WARN("%s %d: Receive g_remote_home or g_battery_home ,set g_go_home, reset g_remote_home.", __FUNCTION__, __LINE__);
			return 0;
		}

		if (g_remote_spot)
		{
			g_remote_spot = false;
			if(SpotMovement::instance()->getSpotType() == NO_SPOT)
				wav_play(WAV_CLEANING_CONTINUE);
		}
		
		Cell_t start{map_get_x_cell(), map_get_y_cell()};
		path_update_cell_history();
		path_update_cells();
		path_reset_path_points();
		int8_t is_found = path_next(&g_next_point, &g_target_point);
//		MotionManage::pubCleanMapMarkers(MAP, g_next_point, g_target_point);
		ROS_ERROR("State: %d", is_found);
		if (is_found == 0) //No target point
		{
			if(get_clean_mode() != Clean_Mode_Spot)
				g_go_home = true;
			return 0;
		}
		else if (is_found == 1)
		{
			if (mt_is_fallwall())
				cm_follow_wall(g_next_point);
			else
			if (path_get_path_points_count() < 3 || !cm_curve_move_to_point())
				cm_linear_move_to_point(g_next_point, RUN_TOP_SPEED);

			linear_mark_clean(start, map_point_to_cell(g_next_point));

			if (cm_should_self_check()){
				// Can not set handler state inside cm_self_check(), because it is actually a universal function.
				cm_set_event_manager_handler_state(true);
				cm_self_check();
				if(get_clean_mode() == Clean_Mode_WallFollow)
					wf_break_wall_follow();
				cm_set_event_manager_handler_state(false);
			}
		}
		if (is_found == 2)
				return -1;

	}
	return 0;
}

void cm_go_home()
{
	/* Robot will try to go to the cells in g_home_point_old_path list
	 * first, and it will only go through the CLEANED area. If the
	 * cell in g_home_point_new_path is unreachable through the
	 * CLEANED area, it will be push into g_home_point_new_path list.
	 * When all the cells in g_home_point_old_path list are unreachable
	 * or failed to go to charger, robot will start to go to cells in
	 * g_home_point_new_path through the UNCLEAN area (If there is a
	 * way like this).
	 */
	bool all_old_path_failed = false;
	Cell_t current_home_cell;
	mt_set(CM_LINEARMOVE);

	set_vacmode(Vac_Normal, false);
	set_vac_speed();
	robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle);
	if(robot::instance()->isLowBatPaused())
		wav_play(WAV_BATTERY_LOW);
	wav_play(WAV_BACK_TO_CHARGER);

	if (!robot::instance()->isLowBatPaused() && !g_map_boundary_created)
		cm_create_home_boundary();

	while (ros::ok())
	{
		if (g_home_point_old_path.empty())
		{
			if (get_clean_mode() == Clean_Mode_WallFollow || g_home_point_new_path.empty())
			{
				ROS_WARN("%s, %d: No targets left.", __FUNCTION__, __LINE__);
				// If it is the last point, it means it it now at (0, 0).
				if (!g_from_station) {
					auto angle = static_cast<int16_t>(robot::instance()->offsetAngle() *10);
					cm_head_to_course(ROTATE_TOP_SPEED, -angle);
				}
				disable_motors();
				if(g_have_seen_charge_stub)
					wav_play(WAV_BACK_TO_CHARGER_FAILED);
				robot::instance()->resetLowBatPause();
				cm_reset_go_home();
				return;
			}

			// Try all the new path home point.
			all_old_path_failed = true;
			set_explore_new_path_flag(true);
			// Get next home cell.
			current_home_cell.X = count_to_cell(g_home_point_new_path.front().X);
			current_home_cell.Y = count_to_cell(g_home_point_new_path.front().Y);
			g_home_point_new_path.pop_front();
			ROS_WARN("%s, %d: Go home Target: (%d, %d), %u new targets left.", __FUNCTION__, __LINE__, current_home_cell.X, current_home_cell.Y, (uint)g_home_point_new_path.size());

		}
		else
		{
			if (get_clean_mode() == Clean_Mode_WallFollow)
				// Always explore the new path.
				set_explore_new_path_flag(true);
			else
				// Try all the old path home point first.
				set_explore_new_path_flag(false);
			// Get next home cell.
			current_home_cell.X = count_to_cell(g_home_point_old_path.front().X);
			current_home_cell.Y = count_to_cell(g_home_point_old_path.front().Y);
			g_home_point_old_path.pop_front();
			ROS_WARN("%s, %d: Go home Target: (%d, %d), %u old path targets left, %u new targets left.", __FUNCTION__, __LINE__, current_home_cell.X, current_home_cell.Y, (uint)g_home_point_old_path.size(), (uint)g_home_point_new_path.size());
		}

		ROS_INFO("%s %d: Current Battery level: %d.", __FUNCTION__, __LINE__, get_battery_voltage());

		// Resume from go home mode.
		set_led(100, 0);
		set_vacmode(Vac_Normal, false);
		set_vac_speed();
		set_side_brush_pwm(50, 50);
		set_main_brush_pwm(30);

		if (!cm_move_to_cell(current_home_cell.X, current_home_cell.Y))
		{
			if (g_fatal_quit_event)
			{
				// Fatal quit means cliff is triggered / bumper jamed / any over current event.
				disable_motors();
				robot::instance()->resetLowBatPause();
				cm_reset_go_home();
				return;
			}
			if (g_key_clean_pressed)
			{
				disable_motors();
				if (robot::instance()->isManualPaused())
					// The current home cell is still valid, so push it back to the home point list.
					cm_set_home(cell_to_count(current_home_cell.X), cell_to_count(current_home_cell.Y));
				if (get_clean_mode() == Clean_Mode_WallFollow)
					cm_reset_go_home();
				return;
			}

			if (get_clean_mode() != Clean_Mode_WallFollow && !all_old_path_failed)
			{
				// If can not reach this point, save this point to new path home point list.
				Point32_t new_home_point;
				new_home_point.X = cell_to_count(current_home_cell.X);
				new_home_point.Y = cell_to_count(current_home_cell.Y);
				g_home_point_new_path.push_back(new_home_point);
				ROS_WARN("%s %d: Can't reach this home point(%d, %d), push to home point of new path list.", __FUNCTION__, __LINE__, current_home_cell.X, current_home_cell.Y);
			}
		}
		else if (g_have_seen_charge_stub && cm_go_to_charger(current_home_cell))
		{
			if (g_fatal_quit_event)
			{
				// Fatal quit means cliff is triggered / bumper jamed / any over current event.
				disable_motors();
				robot::instance()->resetLowBatPause();
				cm_reset_go_home();
			}
			else if (g_key_clean_pressed)
			{
				disable_motors();
				if (robot::instance()->isManualPaused())
					// The current home cell is still valid, so push it back to the home point list.
					cm_set_home(cell_to_count(current_home_cell.X), cell_to_count(current_home_cell.Y));
				if (get_clean_mode() == Clean_Mode_WallFollow)
					cm_reset_go_home();
			}

			return;
		}
	}
}

/* Statement for cm_go_to_charger(void)
 * return : true -- going to charger has been stopped, either successfully or interrupted.
 *          false -- going to charger failed, move to next point.
 */
bool cm_go_to_charger(Cell_t current_home_cell)
{
	// Call GoHome() function to try to go to charger stub.
	ROS_WARN("%s,%d,Call GoHome()",__FUNCTION__,__LINE__);
	cm_unregister_events();
	go_home();
	cm_register_events();
	if (g_charge_detect)
	{
		if (robot::instance()->isLowBatPaused())
		{
			cm_reset_go_home();
			return true;
		}
		cm_reset_go_home();
		return true;
	}
	else if (g_battery_low)
	{
		// Battery too low.
		disable_motors();
		robot::instance()->resetLowBatPause();
		cm_reset_go_home();
		return true;
	}
	else if (g_fatal_quit_event || g_key_clean_pressed)
	{
		disable_motors();
#if MANUAL_PAUSE_CLEANING
		if (g_key_clean_pressed)
		{
			reset_stop_event_status();
			// The current home cell is still valid, so push it back to the home point list.
			cm_set_home(cell_to_count(current_home_cell.X), cell_to_count(current_home_cell.Y));
			return true;
		}
#endif
		robot::instance()->resetLowBatPause();
		reset_stop_event_status();
		cm_reset_go_home();
		return true;
	}
	return false;
}

uint8_t cm_touring(void)
{
	mt_set(get_clean_mode() == Clean_Mode_WallFollow ? CM_FOLLOW_LEFT_WALL : CM_LINEARMOVE);
	g_from_station = 0;
	g_trapped_mode = 0;
	g_motion_init_succeeded = false;
	event_manager_reset_status();
	MotionManage motion;
	g_turn_angle = 0;
	g_have_seen_charge_stub = false;

	if (!motion.initSucceeded())
	{
		//robot::instance()->resetLowBatPause();
		//robot::instance()->resetManualPause();
		return 0;
	}

	g_motion_init_succeeded = true;

	if (!g_go_home && (robot::instance()->isLowBatPaused()))
	{
		if (!cm_resume_cleaning())
		{
			return 0;
		}
	}
	if (cm_cleaning() == 0)
	{
		if (get_clean_mode() != Clean_Mode_Spot){
			cm_go_home();
		}
		else
			g_go_home = false;
	}
	return 0;
}

/*
 * Robot move to target cell
 * @param x	cell x
 * @param y	cell y
 * @param mode 2: Dynamic change cells near target cell
 *			   1: with escape mode, not finish
 *			   0: no escape mode
 * @return	-2: Robot is trapped
 *		-1: Robot cannot move to target cell
 *		1: Robot arrive target cell
 */
bool cm_move_to_cell(int16_t target_x, int16_t target_y)
{
	if (is_block_accessible(target_x, target_y) == 0) {
		ROS_WARN("%s %d: target is blocked.\n", __FUNCTION__, __LINE__);
		map_set_cells(ROBOT_SIZE, target_x, target_y, CLEANED);
	}

	ROS_INFO("%s %d: Path Find: target: (%d, %d)", __FUNCTION__, __LINE__, target_x, target_y);
	map_set_cells(ROBOT_SIZE, target_x, target_y, CLEANED);

	while (ros::ok()) {
		Cell_t pos{target_x, target_y};
		Cell_t	tmp;
		auto pathFind = (int8_t) path_next_best(pos, map_get_x_cell(), map_get_y_cell(), tmp.X, tmp.Y);

		ROS_INFO("%s %d: Path Find: %d\tTarget: (%d, %d)\tNow: (%d, %d)", __FUNCTION__, __LINE__, pathFind, tmp.X, tmp.Y,
						 map_get_x_cell(), map_get_y_cell());
		if ( pathFind == 1 || pathFind == SCHAR_MAX ) {
			path_update_cell_history();

			if (cm_check_loop_back(tmp) == 1)
				return false;

			ROS_INFO("%s %d: Move to target...", __FUNCTION__, __LINE__ );
			debug_map(MAP, tmp.X, tmp.Y);
			Point32_t	Next_Point{cell_to_count(tmp.X), cell_to_count(tmp.Y) };
			if (path_get_path_points_count() < 3 || !cm_curve_move_to_point())
				cm_linear_move_to_point(Next_Point, RUN_TOP_SPEED);

			if (g_fatal_quit_event || g_key_clean_pressed )
				return false;

			if ((g_battery_home || g_remote_home) && !g_go_home )
				return false;

			//Arrive exit cell, set < 3 when ROBOT_SIZE == 5
			if ( TwoPointsDistance( target_x , target_y , map_get_x_cell(), map_get_y_cell() ) < ROBOT_SIZE / 2 + 1 ) {
				ROS_WARN("%s %d: Now: (%d, %d)\tDest: (%d, %d)", __FUNCTION__, __LINE__, map_get_x_cell(), map_get_y_cell(), target_x , target_y);
				return true;
			}
		} else
			return false;
	}
	return false;
}

void cm_reset_go_home(void)
{
	ROS_DEBUG("Reset go home flags here.");
	g_go_home = false;
	g_map_boundary_created = false;
}

void cm_set_home(int32_t x, int32_t y) {
	Cell_t tmpPnt;

	bool found = false;
	Point32_t new_home_point;

	ROS_INFO("%s %d: Push new reachable home: (%d, %d) to home point list.", __FUNCTION__, __LINE__, count_to_cell(x),
					 count_to_cell(y));
	new_home_point.X = x;
	new_home_point.Y = y;

	for (list<Point32_t>::iterator it = g_home_point_old_path.begin(); found == false && it != g_home_point_old_path.end(); ++it) {
		if (it->X == x && it->Y == y) {
			found = true;
		}
	}
	if (found == false) {
		g_home_point_old_path.push_front(new_home_point);
		// If new_home_point near (0, 0)
		if (abs(count_to_cell(x)) <= 5 && abs(count_to_cell(y)) <= 5)
		{
			// Update the trapped reference points
			tmpPnt.X = count_to_cell(x);
			tmpPnt.Y = count_to_cell(y);
			for (int8_t i = ESCAPE_TRAPPED_REF_CELL_SIZE - 1; i > 0; i--)
			{
				g_pnt16_ar_tmp[i] = g_pnt16_ar_tmp[i-1];
				ROS_DEBUG("i = %d, g_pnt16_ar_tmp[i].X = %d, g_pnt16_ar_tmp[i].Y = %d", i, g_pnt16_ar_tmp[i].X, g_pnt16_ar_tmp[i].Y);
			}
			g_pnt16_ar_tmp[0] = tmpPnt;
			ROS_DEBUG("g_pnt16_ar_tmp[0].X = %d, g_pnt16_ar_tmp[0].Y = %d", g_pnt16_ar_tmp[0].X, g_pnt16_ar_tmp[0].Y);
			path_escape_set_trapped_cell(g_pnt16_ar_tmp, ESCAPE_TRAPPED_REF_CELL_SIZE);
		}
	}
}

void cm_set_continue_point(int32_t x, int32_t y)
{
	ROS_INFO("%s %d: Set continue point: (%d, %d).", __FUNCTION__, __LINE__, count_to_cell(x), count_to_cell(y));
	g_continue_point.X = x;
	g_continue_point.Y = y;
}

uint8_t cm_check_loop_back(Cell_t target) {
	uint8_t retval = 0;
	if ( target == g_cell_history[1] && target == g_cell_history[3]) {
		ROS_WARN("%s %d Possible loop back (%d, %d)", __FUNCTION__, __LINE__, target.X, target.Y);
		retval	= 1;
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

	if (g_bumper_jam || g_cliff_jam)
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

	robotbase_obs_adjust_count(50);
	while (ros::ok) {
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}

		if (g_fatal_quit_event || g_key_clean_pressed)
			break;

		if (g_cliff_all_cnt >= 2)
		{
			ROS_WARN("%s %d: Robot lifted up.", __FUNCTION__, __LINE__);
			g_fatal_quit_event = true;
			break;
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
			if (!get_cliff_trig())
			{
				ROS_WARN("%s %d: Cliff resume succeeded.", __FUNCTION__, __LINE__);
				g_cliff_triggered = false;
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
				g_bumper_hitted = false;
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
						if (get_cliff_trig() && ++g_cliff_cnt > 2)
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
						if (get_cliff_trig() && ++g_cliff_cnt > 2)
						{
							g_cliff_jam = true;
							resume_cnt = 0;
						}
						else
						{
							bumper_jam_state++;
							ROS_WARN("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state);
							target_angle = Gyro_GetAngle() - 900;
							if (target_angle < 0)
								target_angle += 3600;
							ROS_WARN("%s %d: target_angle:%d.", __FUNCTION__, __LINE__, target_angle);
						}
					}
					break;
				}
				case 4:
				{
					ROS_DEBUG("%s %d: Gyro_GetAngle(): %d", __FUNCTION__, __LINE__, Gyro_GetAngle());
					// If cliff jam during bumper self resume.
					if (get_cliff_trig() && ++g_cliff_cnt > 2)
					{
						g_cliff_jam = true;
						resume_cnt = 0;
					}
					else if (abs(Gyro_GetAngle() - target_angle) < 50)
					{
						bumper_jam_state++;
						ROS_WARN("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state);
						target_angle = Gyro_GetAngle() + 900;
						if (target_angle > 3600)
							target_angle -= 3600;
						ROS_WARN("%s %d: target_angle:%d.", __FUNCTION__, __LINE__, target_angle);
					}
					break;
				}
				case 5:
				{
					// If cliff jam during bumper self resume.
					if (get_cliff_trig() && ++g_cliff_cnt > 2)
					{
						g_cliff_jam = true;
						resume_cnt = 0;
					}
					else if (abs(Gyro_GetAngle() - target_angle) < 50)
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
		else
			break;

		if(! regulator.adjustSpeed(bumper_jam_state))
			break;
	}
}

bool cm_should_self_check(void)
{
	return (g_oc_wheel_left || g_oc_wheel_right || g_bumper_jam || g_cliff_jam || g_oc_suction);
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

#undef event_manager_register_and_disable_x

	event_manager_set_current_mode(EVT_MODE_USER_INTERFACE);

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
	g_bumper_hitted = true;

	if (!state_last && g_move_back_finished)
	{
		saved_pos_x = robot::instance()->getOdomPositionX();
		saved_pos_y = robot::instance()->getOdomPositionY();
		set_wheel_speed(0, 0);
	}

//	if (g_move_back_finished && !g_bumper_jam)
//		ROS_WARN("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, get_bumper_status(), state_now ? "true" : "false", state_last ? "true" : "false");

}

void cm_handle_bumper_left(bool state_now, bool state_last)
{
	g_bumper_hitted = true;

	if (!state_last && g_move_back_finished)
	{
		saved_pos_x = robot::instance()->getOdomPositionX();
		saved_pos_y = robot::instance()->getOdomPositionY();
		set_wheel_speed(0, 0);
	}

//	if (g_move_back_finished && !g_bumper_jam)
//		ROS_WARN("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, get_bumper_status(), state_now ? "true" : "false", state_last ? "true" : "false");
}

void cm_handle_bumper_right(bool state_now, bool state_last)
{
	g_bumper_hitted = true;

	if (!state_last && g_move_back_finished)
	{
		saved_pos_x = robot::instance()->getOdomPositionX();
		saved_pos_y = robot::instance()->getOdomPositionY();
		set_wheel_speed(0, 0);
	}

//	if (g_move_back_finished && !g_bumper_jam)
//		ROS_WARN("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, get_bumper_status(), state_now ? "true" : "false", state_last ? "true" : "false");
}

/* OBS */
void cm_handle_obs_front(bool state_now, bool state_last)
{
//	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
	g_obs_triggered = true;
}

void cm_handle_obs_left(bool state_now, bool state_last)
{
//	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
	g_obs_triggered = true;
}

void cm_handle_obs_right(bool state_now, bool state_last)
{
//	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
	g_obs_triggered = true;
}

/* Cliff */
void cm_handle_cliff_all(bool state_now, bool state_last)
{
	g_cliff_all_triggered = true;
	g_cliff_all_cnt++;
	g_cliff_triggered = true;
	if (g_move_back_finished && !g_cliff_jam && !state_last)
		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void cm_handle_cliff_front_left(bool state_now, bool state_last)
{
	g_cliff_all_triggered = false;
	g_cliff_triggered = true;

	if (!state_last && g_move_back_finished)
	{
		saved_pos_x = robot::instance()->getOdomPositionX();
		saved_pos_y = robot::instance()->getOdomPositionY();
		set_wheel_speed(0, 0);
	}

	if (g_move_back_finished && !g_cliff_jam && !state_last)
		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");

}

void cm_handle_cliff_front_right(bool state_now, bool state_last)
{
	g_cliff_all_triggered = false;
	g_cliff_triggered = true;

	if (!state_last && g_move_back_finished)
	{
		saved_pos_x = robot::instance()->getOdomPositionX();
		saved_pos_y = robot::instance()->getOdomPositionY();
		set_wheel_speed(0, 0);
	}

	if (g_move_back_finished && !g_cliff_jam && !state_last)
		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");

}

void cm_handle_cliff_left_right(bool state_now, bool state_last)
{
	g_cliff_all_triggered = false;
	g_cliff_triggered = true;

	if (!state_last && g_move_back_finished)
	{
		saved_pos_x = robot::instance()->getOdomPositionX();
		saved_pos_y = robot::instance()->getOdomPositionY();
		set_wheel_speed(0, 0);
	}

	if (g_move_back_finished && !g_cliff_jam && !state_last)
		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void cm_handle_cliff_front(bool state_now, bool state_last)
{
	g_cliff_all_triggered = false;
	g_cliff_triggered = true;

	if (!state_last && g_move_back_finished)
	{
		saved_pos_x = robot::instance()->getOdomPositionX();
		saved_pos_y = robot::instance()->getOdomPositionY();
		set_wheel_speed(0, 0);
	}

	if (g_move_back_finished && !g_cliff_jam && !state_last)
		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void cm_handle_cliff_left(bool state_now, bool state_last)
{
	g_cliff_all_triggered = false;
	g_cliff_triggered = true;

	if (!state_last && g_move_back_finished)
	{
		saved_pos_x = robot::instance()->getOdomPositionX();
		saved_pos_y = robot::instance()->getOdomPositionY();
		set_wheel_speed(0, 0);
	}

	if (g_move_back_finished && !g_cliff_jam && !state_last)
		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void cm_handle_cliff_right(bool state_now, bool state_last)
{
	g_cliff_all_triggered = false;
	g_cliff_triggered = true;

	if (!state_last && g_move_back_finished)
	{
		saved_pos_x = robot::instance()->getOdomPositionX();
		saved_pos_y = robot::instance()->getOdomPositionY();
		set_wheel_speed(0, 0);
	}

	if (g_move_back_finished && !g_cliff_jam && !state_last)
		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

/* RCON */
void cm_handle_rcon(bool state_now, bool state_last)
{
	// lt_cnt means count for rcon left receive home top signal. rt_cnt, etc, can be understood like this.
	static int8_t lt_cnt = 0, rt_cnt = 0, flt_cnt = 0, frt_cnt = 0, fl2t_cnt = 0, fr2t_cnt = 0;

	/*
	 * direction indicates which cell should robot mark for blocking.
	 * -2: left
	 * -1: front left
	 *  0: front
	 *  1: front right
	 *  2: right
	 *  9: do not need to block
	 */
	int8_t direction = 9;
	int8_t max_cnt = 0;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_go_home) {
		ROS_DEBUG("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
		return;
	}
	//if (!(get_rcon_status() & (RconL_HomeT | RconR_HomeT | RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT)))
	// Since we have front left 2 and front right 2 rcon receiver, seems it is not necessary to handle left or right rcon receives home signal.
	if (!(get_rcon_status() & (RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT)))
		// Skip other rcon signals.
		return;

	if (get_rcon_status() & RconL_HomeT)
		lt_cnt++;
	if (get_rcon_status() & RconR_HomeT)
		rt_cnt++;
	if (get_rcon_status() & RconFL_HomeT)
		flt_cnt++;
	if (get_rcon_status() & RconFR_HomeT)
		frt_cnt++;
	if (get_rcon_status() & RconFL2_HomeT)
		fl2t_cnt++;
	if (get_rcon_status() & RconFR2_HomeT)
		fr2t_cnt++;

	if (lt_cnt > 2)
	{
		max_cnt = lt_cnt;
		direction = -2;
	}
	if (fl2t_cnt > 2 && fl2t_cnt > max_cnt)
	{
		max_cnt = fl2t_cnt;
		direction = -1;
	}
	if (flt_cnt > 2 && flt_cnt > max_cnt)
	{
		max_cnt = flt_cnt;
		direction = 0;
	}
	if (frt_cnt > 2 && frt_cnt > max_cnt)
	{
		max_cnt = frt_cnt;
		direction = 0;
	}
	if (fr2t_cnt > 2 && fr2t_cnt > max_cnt)
	{
		max_cnt = fr2t_cnt;
		direction = 1;
	}
	if (rt_cnt > 2 && rt_cnt > max_cnt)
	{
		direction = 2;
	}

	if(direction != 9)
	{
		g_have_seen_charge_stub = true;
		cm_block_charger_stub(direction);
		lt_cnt = fl2t_cnt = flt_cnt = frt_cnt = fr2t_cnt = rt_cnt = 0;
	}
	reset_rcon_status();
}

void cm_block_charger_stub(int8_t direction)
{
	int32_t x, y, x2, y2;

	ROS_WARN("%s %d: Robot meet charger stub, stop and mark the block.", __FUNCTION__, __LINE__);
	set_wheel_speed(0, 0);

	switch (direction)
	{
		case -2:
		{
			cm_world_to_point(Gyro_GetAngle(), CELL_SIZE_2, CELL_SIZE, &x, &y);
			if (mt_is_left())
					g_turn_angle = 300;
				//else
				//	g_turn_angle = 1100;
			break;
		}
		case -1:
		{
			cm_world_to_point(Gyro_GetAngle(), CELL_SIZE_2, CELL_SIZE, &x, &y);
			cm_world_to_point(Gyro_GetAngle(), CELL_SIZE, CELL_SIZE_2, &x2, &y2);
			map_set_cell(MAP, x2, y2, BLOCKED_BUMPER);
			ROS_INFO("%s %d: is called. marking (%d, %d)", __FUNCTION__, __LINE__, count_to_cell(x2), count_to_cell(y2));
			if (mt_is_left())
					g_turn_angle = 600;
				//else
				//	g_turn_angle = 950;
			break;
		}
		case 0:
		{
			cm_world_to_point(Gyro_GetAngle(), 0, CELL_SIZE_2, &x, &y);
			if (mt_is_fallwall())
				g_turn_angle = 850;
			break;
		}
		case 1:
		{
			cm_world_to_point(Gyro_GetAngle(), -CELL_SIZE_2, CELL_SIZE, &x, &y);
			cm_world_to_point(Gyro_GetAngle(), -CELL_SIZE, CELL_SIZE_2, &x2, &y2);
			map_set_cell(MAP, x2, y2, BLOCKED_BUMPER);
			ROS_INFO("%s %d: is called. marking (%d, %d)", __FUNCTION__, __LINE__, count_to_cell(x2), count_to_cell(y2));
			if (mt_is_right())
					g_turn_angle = 600;
				//else
				//	g_turn_angle = 950;
			break;
		}
		case 2:
		{
			cm_world_to_point(Gyro_GetAngle(), -CELL_SIZE_2, CELL_SIZE, &x, &y);
			if (mt_is_right())
					g_turn_angle = 300;
				//else
				//	g_turn_angle = 1100;
			break;
		}
		default:
			ROS_ERROR("%s %d: Receive wrong direction: %d.", __FUNCTION__, __LINE__, direction);
	}
	cm_world_to_point(Gyro_GetAngle(), 0, CELL_SIZE_2, &x, &y);
	map_set_cell(MAP, x, y, BLOCKED_BUMPER);
	ROS_INFO("%s %d: is called. marking (%d, %d)", __FUNCTION__, __LINE__, count_to_cell(x), count_to_cell(y));

	g_rcon_triggered = true;
	cm_set_home(map_get_x_count(), map_get_y_count());

}
/*
void cm_handle_rcon_front_left(bool state_now, bool state_last)
{
	int32_t	x, y;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_go_home) {
		ROS_DEBUG("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
		return;
	}

	Set_Wheel_Speed(0, 0);

	cm_count_normalize(Gyro_GetAngle(), 0, CELL_SIZE_2, &x, &y);
	map_set_cell(MAP, x, y, BLOCKED_BUMPER);
	ROS_INFO("%s %d: is called. marking (%d, %d)", __FUNCTION__, __LINE__, count_to_cell(x), count_to_cell(y));

	g_rcon_triggered = true;
	cm_set_home(map_get_x_count(), map_get_y_count());
	Reset_Rcon_Status();

	if (g_cm_move_type == CM_FOLLOW_WALL) {
		g_turn_angle = 850;
	}
}

void cm_handle_rcon_front_left2(bool state_now, bool state_last)
{
	int32_t	x, y;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_go_home) {
		ROS_DEBUG("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
		return;
	}

	Set_Wheel_Speed(0, 0);

	cm_count_normalize(Gyro_GetAngle(), CELL_SIZE, CELL_SIZE_2, &x, &y);
	map_set_cell(MAP, x, y, BLOCKED_BUMPER);
	ROS_INFO("%s %d: is called. marking (%d, %d)", __FUNCTION__, __LINE__, count_to_cell(x), count_to_cell(y));
	cm_count_normalize(Gyro_GetAngle(), CELL_SIZE_2, CELL_SIZE, &x, &y);
	map_set_cell(MAP, x, y, BLOCKED_BUMPER);
	ROS_INFO("%s %d: is called. marking (%d, %d)", __FUNCTION__, __LINE__, count_to_cell(x), count_to_cell(y));

	g_rcon_triggered = true;
	cm_set_home(map_get_x_count(), map_get_y_count());
	Reset_Rcon_Status();
	if (g_cm_move_type == CM_FOLLOW_WALL) {
		if (g_rounding_type == ROUNDING_LEFT) {
			g_turn_angle = 600;
		}
	}
}

void cm_handle_rcon_front_right(bool state_now, bool state_last)
{
	int32_t	x, y;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_go_home) {
		ROS_DEBUG("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
		return;
	}

	Set_Wheel_Speed(0, 0);

	cm_count_normalize(Gyro_GetAngle(), 0, CELL_SIZE_2, &x, &y);
	map_set_cell(MAP, x, y, BLOCKED_BUMPER);
	ROS_INFO("%s %d: is called. marking (%d, %d)", __FUNCTION__, __LINE__, count_to_cell(x), count_to_cell(y));

	g_rcon_triggered = true;
	cm_set_home(map_get_x_count(), map_get_y_count());
	Reset_Rcon_Status();

	if (g_cm_move_type == CM_FOLLOW_WALL) {
		g_turn_angle = 850;
	}
}

void cm_handle_rcon_front_right2(bool state_now, bool state_last)
{
	int32_t	x, y;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_go_home) {
		ROS_DEBUG("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
		return;
	}

	Set_Wheel_Speed(0, 0);

	cm_count_normalize(Gyro_GetAngle(), -CELL_SIZE, CELL_SIZE_2, &x, &y);
	map_set_cell(MAP, x, y, BLOCKED_BUMPER);
	ROS_INFO("%s %d: is called. marking (%d, %d)", __FUNCTION__, __LINE__, count_to_cell(x), count_to_cell(y));
	cm_count_normalize(Gyro_GetAngle(), -CELL_SIZE_2, CELL_SIZE, &x, &y);
	map_set_cell(MAP, x, y, BLOCKED_BUMPER);
	ROS_INFO("%s %d: is called. marking (%d, %d)", __FUNCTION__, __LINE__, count_to_cell(x), count_to_cell(y));

	g_rcon_triggered = true;
	cm_set_home(map_get_x_count(), map_get_y_count());
	Reset_Rcon_Status();
	if (g_cm_move_type == CM_FOLLOW_WALL) {
		if (g_rounding_type == ROUNDING_LEFT) {
			g_turn_angle = 950;
		}
	}
}

void cm_handle_rcon_left(bool state_now, bool state_last)
{
	int32_t	x, y;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_go_home) {
		ROS_DEBUG("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
		return;
	}

	set_wheel_speed(0, 0);

	cm_count_normalize(Gyro_GetAngle(), CELL_SIZE, CELL_SIZE_2, &x, &y);
	map_set_cell(MAP, x, y, BLOCKED_BUMPER);
	ROS_INFO("%s %d: is called. marking (%d, %d)", __FUNCTION__, __LINE__, count_to_cell(x), count_to_cell(y));

	g_rcon_triggered = true;
	cm_set_home(map_get_x_count(), map_get_y_count());
	Reset_Rcon_Status();

	if (g_cm_move_type == CM_FOLLOW_WALL) {
		if (g_rounding_type == ROUNDING_LEFT) {
			g_turn_angle = 300;
		}
	}
}

void cm_handle_rcon_right(bool state_now, bool state_last)
{
	int32_t	x, y;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_go_home) {
		ROS_DEBUG("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
		return;
	}

	set_wheel_speed(0, 0);

	cm_world_to_point(Gyro_GetAngle(), -CELL_SIZE, CELL_SIZE_2, &x, &y);
	map_set_cell(MAP, x, y, BLOCKED_BUMPER);
	ROS_INFO("%s %d: is called. marking (%d, %d)", __FUNCTION__, __LINE__, count_to_cell(x), count_to_cell(y));

	g_rcon_triggered = true;
	cm_set_home(map_get_x_count(), map_get_y_count());
	reset_rcon_status();
	if (g_cm_move_type == CM_FOLLOW_WALL) {
		if (g_rounding_type == ROUNDING_LEFT) {
			g_turn_angle = 1100;
		}
	}
}
*/

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
		ROS_WARN("%s %d: left wheel over current, %u mA", __FUNCTION__, __LINE__, (uint32_t) robot::instance()->getLwheelCurrent());

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
		ROS_WARN("%s %d: right wheel over current, %u mA", __FUNCTION__, __LINE__, (uint32_t) robot::instance()->getRwheelCurrent());

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
	beep_for_command(true);
	set_wheel_speed(0, 0);
	g_key_clean_pressed = true;

	if(SpotMovement::instance()->getSpotType() != NORMAL_SPOT && get_clean_mode() != Clean_Mode_WallFollow)
		robot::instance()->setManualPause();

	start_time = time(NULL);
	while (get_key_press() & KEY_CLEAN)
	{
		if (time(NULL) - start_time > 3) {
			if (!reset_manual_pause)
			{
				beep_for_command(true);
				reset_manual_pause = true;
			}
			robot::instance()->resetManualPause();
			ROS_WARN("%s %d: Key clean is not released and manual pause has been reset.", __FUNCTION__, __LINE__);
		}
		else
			ROS_WARN("%s %d: Key clean is not released.", __FUNCTION__, __LINE__);
		usleep(20000);
	}

	reset_touch();
}

/* Remote */

void cm_handle_remote_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	beep_for_command(true);
	g_key_clean_pressed = true;
	if(SpotMovement::instance()->getSpotType() != NORMAL_SPOT && get_clean_mode() != Clean_Mode_WallFollow){
		SpotMovement::instance()->spotDeinit();
		robot::instance()->setManualPause();
	}
	reset_rcon_remote();
}

void cm_handle_remote_home(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_motion_init_succeeded && !g_go_home && !cm_should_self_check()) {

		if( SpotMovement::instance()->getSpotType()  == NORMAL_SPOT){
			beep_for_command(false);
		}
		else{
			g_remote_home = true;
			beep_for_command(true);
			if (get_clean_mode() == Clean_Mode_WallFollow)
				wf_clear();
			if (SpotMovement::instance()->getSpotType() == CLEAN_SPOT){
				SpotMovement::instance()->spotDeinit();
			}
		}
		ROS_INFO("g_remote_home = %d", g_remote_home);
	}
	else {
		beep_for_command(false);
		ROS_INFO("g_remote_home = %d", g_remote_home);
	}
	reset_rcon_remote();
}

static bool g_remote_spot_pressed = false;
static time_t g_spot_pressed_duration;

void cm_handle_remote_spot(bool state_now, bool state_last)
{
	
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (mt_is_fallwall())
	{
		beep_for_command(false);
		reset_rcon_remote();
		return;
	}

	bool b_time_short = false;
	if(g_remote_spot_pressed == false){
		g_remote_spot_pressed = true;
		g_spot_pressed_duration = time(NULL);
	}
	else{
		g_remote_spot_pressed = false;
		if (difftime(time(NULL),g_spot_pressed_duration) < 3){
			ROS_WARN("%s,%d, press spot key to fast",__FUNCTION__,__LINE__);
			b_time_short = true;	
		}
	}

	if (g_motion_init_succeeded && !g_go_home && !cm_should_self_check() && !b_time_short && !g_remote_spot)
	{
		if( SpotMovement::instance() -> getSpotType() == NO_SPOT){
			/*----store current position---*/
			map_set_position((robot::instance()->getPositionX() * 1000 * CELL_COUNT_MUL/CELL_SIZE), 
						(robot::instance()->getPositionY() * 1000 * CELL_COUNT_MUL/CELL_SIZE));
			ROS_WARN("%s,%d,cur cell x = %d,cur cell y = %d",__FUNCTION__,__LINE__,map_get_x_cell(),map_get_y_cell());
			SpotMovement::instance() ->setSpotType(CLEAN_SPOT);
			set_wheel_speed(0, 0);
			beep_for_command(true);
			g_remote_spot = true;
		}
		else if(SpotMovement::instance()->getSpotType() == CLEAN_SPOT){
			beep_for_command(true);
			SpotMovement::instance()->spotDeinit();
			set_wheel_speed(0, 0);
			g_remote_spot = true;
		}
		else{
			beep_for_command(false);
		}
	}
	else
		beep_for_command(false);

	reset_rcon_remote();
}

void cm_handle_remote_wallfollow(bool state_now,bool state_last)
{
	ROS_WARN("%s,%d: is called.",__FUNCTION__,__LINE__);
	beep_for_command(false);
	reset_rcon_remote();
}

void cm_handle_remote_max(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_motion_init_succeeded && !g_go_home && !cm_should_self_check() && SpotMovement::instance()->getSpotType() == NO_SPOT)
	{
		beep_for_command(true);
		switch_vac_mode(true);
	}
	else
		beep_for_command(false);
	reset_rcon_remote();
}

void cm_handle_remote_direction(bool state_now,bool state_last)
{
	ROS_WARN("%s,%d: is called.",__FUNCTION__,__LINE__);
	beep_for_command(false);
	reset_rcon_remote();
}

/* Battery */
void cm_handle_battery_home(bool state_now, bool state_last)
{
	if (g_motion_init_succeeded && ! g_go_home) {
		ROS_WARN("%s %d: low battery, battery = %dmv ", __FUNCTION__, __LINE__,
						 robot::instance()->getBatteryVoltage());
		g_battery_home = true;

		if (get_vac_mode() == Vac_Max) {
			switch_vac_mode(false);
		}
#if CONTINUE_CLEANING_AFTER_CHARGE
		if (SpotMovement::instance()->getSpotType() != NORMAL_SPOT ){
			cm_set_continue_point(map_get_x_count(), map_get_y_count());
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
	if (robot::instance()->getChargeStatus() == 3)
	{
		if (g_charge_detect_cnt++ > 5)
		{
			g_charge_detect = robot::instance()->getChargeStatus();
			g_fatal_quit_event = true;
			ROS_WARN("%s %d: g_charge_detect has been set to %d.", __FUNCTION__, __LINE__, g_charge_detect);
			g_charge_detect_cnt = 0;
		}
	}
}
