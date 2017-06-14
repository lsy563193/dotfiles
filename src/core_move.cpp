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
//#include "obstacle_detector.h"
//using namespace obstacle_detector;


#ifdef TURN_SPEED
#undef TURN_SPEED
#endif

#define TURN_SPEED	17
#define ROTATE_LOW_SPEED			(7)
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

CMMoveType g_cm_move_type;

uint8_t	should_mark = 0;

RoundingType	g_rounding_type;
uint16_t g_rounding_turn_angle;
uint16_t g_rounding_move_speed;
uint16_t g_rounding_wall_distance;
uint16_t g_rounding_wall_straight_distance;
int16_t g_rounding_left_wall_buffer[3];
int16_t g_rounding_right_wall_buffer[3];

extern uint32_t g_cur_wtime;//temporary work time
Point32_t g_next_point, g_targets_point;

// This list is for storing the position that robot sees the charger stub.
std::list <Point32_t> g_home_point;

// This is for the continue point for robot to go after charge.
Point32_t g_continue_point;

bool	g_go_home = false;
bool	g_from_station = 0;
int16_t g_map_gyro_offset = 0;
uint8_t	g_should_follow_wall = 0;

// This flag is for checking whether map boundary is created.
bool g_map_boundary_created = false;

// This g_pnt16_ar_tmp is for trapped reference point.
Cell_t g_pnt16_ar_tmp[3];

Cell_t g_relativePos[MOVE_TO_CELL_SEARCH_ARRAY_LENGTH * MOVE_TO_CELL_SEARCH_ARRAY_LENGTH] = {{0, 0}};

extern PositionType g_pos_history[];

extern int16_t g_x_min, g_x_max, g_y_min, g_y_max;

// This status is for rounding function to decide the angle it should turn.
uint8_t g_bumper_status_for_rounding;

// This time count is for checking how many times of 20ms did the user press the key.
uint16_t g_press_time = 0;

/* Events variables */
/* The fatal quit event includes any of the following case:
 *  g_bumper_jam
 * 	g_cliff_all_triggered
 * 	g_oc_brush_main
 * 	g_oc_wheel_left
 * 	g_oc_wheel_right
 * 	g_oc_suction
 * 	g_battery_low
 */
bool g_fatal_quit_event = false;

/* Bumper */
bool g_bumper_jam = false;
bool g_bumper_hitted = false;

/* OBS */
bool g_obs_triggered = false;

/* Cliff */
bool g_cliff_all_triggered = false;

bool g_cliff_jam = false;

bool g_cliff_triggered = false;
uint8_t g_cliff_cnt = 0;

/* RCON */
bool g_rcon_triggered = false;

/* Over Current */
uint8_t g_oc_brush_left_cnt = 0;

bool g_oc_brush_main = false;
uint8_t g_oc_brush_main_cnt = 0;

uint8_t g_oc_brush_right_cnt = 0;

bool g_oc_wheel_left = false;
uint8_t	g_oc_wheel_left_cnt = 0;

bool g_oc_wheel_right = false;
uint8_t g_oc_wheel_right_cnt = 0;

bool g_oc_suction = false;
uint8_t g_oc_suction_cnt = 0;

/* Key */
bool g_key_clean_pressed = false;

/* Remote */
bool g_remote_home = false;

/* Battery */
bool g_battery_home = false;

bool g_battery_low = false;
uint8_t	g_battery_low_cnt = 0;

int g_bumper_cnt = 0;

static int16_t ranged_angle(int16_t angle)
{
	if (angle >= 1800) {
			angle -= 3600;
	} else
	if (angle <= -1800) {
			angle += 3600;
	}
	return angle;
}

typedef struct Regulator1_{
	Regulator1_(int32_t max):integrated_(0),integration_cycle_(0),base_speed_(BASE_SPEED),tick_(0),speed_max_(max){};
	bool adjustSpeed(Point32_t Target, uint8_t &left_speed, uint8_t &right_speed, bool);
	int32_t speed_max_;
	int32_t integrated_;
	int32_t base_speed_;
	uint8_t integration_cycle_;
	uint32_t tick_;
}LinearSpeedRegulator;

// Target:	robot coordinate
static bool check_map_boundary(bool& slow_down)
{
		int32_t		x, y;
		for (auto dy = -1; dy <= 1; dy++) {
			CM_count_normalize(Gyro_GetAngle(), dy * CELL_SIZE, CELL_SIZE_3, &x, &y);
			if (Map_GetCell(MAP, countToCell(x), countToCell(y)) == BLOCKED_BOUNDARY)
				slow_down = true;

			CM_count_normalize(Gyro_GetAngle(), dy * CELL_SIZE, CELL_SIZE_2, &x, &y);
			if (Map_GetCell(MAP, countToCell(x), countToCell(y)) == BLOCKED_BOUNDARY) {
				ROS_INFO("%s, %d: Blocked boundary.", __FUNCTION__, __LINE__);
				Stop_Brifly();
				CM_update_map();
				return true;
			}
		}
	return false;
}

bool LinearSpeedRegulator::adjustSpeed(Point32_t Target, uint8_t &left_speed, uint8_t &right_speed, bool slow_down)
{
	auto rotate_angle = ranged_angle(course2dest(Map_GetXCount(), Map_GetYCount(), Target.X, Target.Y) - Gyro_GetAngle());

	if ( std::abs(rotate_angle) > 300)
	{
		ROS_WARN("%s %d: warning: angle is too big, angle: %d", __FUNCTION__, __LINE__, rotate_angle);
		return false;
	}

	if (integration_cycle_++ > 10)
	{
		integration_cycle_ = 0;
		integrated_ += rotate_angle;
		check_limit(integrated_, -150, 150);
	}

	auto distance = TwoPointsDistance(Map_GetXCount(), Map_GetYCount(), Target.X, Target.Y);
	auto obstcal_detected = MotionManage::s_laser->laserObstcalDetected(0.2, 0, -1.0);

	if ( Get_OBS_Status() || Is_OBS_Near() || (distance < SLOW_DOWN_DISTANCE) || slow_down || obstcal_detected)
	{
		integrated_ = 0;
		rotate_angle = 0;

		//if ( Get_OBS_Status() )
		//	base_speed_ -= 5;
		//else if ( Is_OBS_Near() )
		//	base_speed_ -= 4;
		//else
		//	base_speed_ -= 3;
		base_speed_ -= 1;

		base_speed_ = base_speed_ < BASE_SPEED ? BASE_SPEED : base_speed_;
	} else
	if (base_speed_ < (int32_t) speed_max_)
	{
		if (tick_++ > 5)
		{
			tick_ = 0;
			base_speed_++;
		}
		integrated_ = 0;
	}

	left_speed = base_speed_ - rotate_angle / 20 - integrated_ / 150; // - Delta / 20; // - Delta * 10 ; // - integrated_ / 2500;
	right_speed = base_speed_ + rotate_angle / 20 + integrated_ / 150; // + Delta / 20;// + Delta * 10 ; // + integrated_ / 2500;

	check_limit(left_speed, BASE_SPEED, speed_max_);
	check_limit(right_speed, BASE_SPEED, speed_max_);
	base_speed_ = (left_speed + right_speed) / 2;
//	ROS_ERROR("left_speed(%d),right_speed(%d), base_speed_(%d), slow_down(%d)",left_speed, right_speed, base_speed_, slow_down);
	return true;
}

typedef struct Regulator2_{
	Regulator2_(uint8_t speed_max,uint8_t speed_min, uint8_t speed_start):
					speed_max_(speed_max),speed_min_(speed_min),tick_(0), speed_(speed_start){};
	bool adjustSpeed(int16_t diff, uint8_t& speed_up);
	uint8_t speed_max_;
	uint8_t speed_min_;
	uint8_t speed_;
	uint32_t tick_;
}TurnSpeedRegulator;

bool TurnSpeedRegulator::adjustSpeed(int16_t diff, uint8_t& speed)
{
	if ((diff >= 0) && (diff <= 1800))
		Set_Dir_Left();
	else if ((diff <= 0) && (diff >= (-1800)))
		Set_Dir_Right();

	tick_++;
	if (tick_ > 2)
	{
		tick_ = 0;
		if (std::abs(diff) > 350){
			speed_ = std::min(++speed_, speed_max_);
		}
		else{
			--speed_;
			speed_ = std::max(--speed_, speed_min_);
		}
	}
	speed = speed_;
	return true;
}

static double radius_of(Cell_t cell_0,Cell_t cell_1)
{
	return (abs(cellToCount(cell_0.X - cell_1.X)) + abs(cellToCount(cell_0.Y - cell_1.Y))) / 2;
}

void CM_count_normalize(uint16_t heading, int16_t offset_lat, int16_t offset_long, int32_t *x, int32_t *y)
{
	*x = cellToCount(countToCell(Map_GetRelativeX(heading, offset_lat, offset_long)));
	*y = cellToCount(countToCell(Map_GetRelativeY(heading, offset_lat, offset_long)));
}

int CM_Get_grid_index(float position_x, float position_y, uint32_t width, uint32_t height, float resolution, double origin_x, double origin_y )
{
	int index, grid_x, grid_y;

	/*get index*/
	grid_x = int(round((position_x - origin_x) / resolution));
	grid_y = int(round((position_y - origin_y) / resolution));
	index = grid_x + grid_y * width;
	
	return index;
}

void CM_update_map_cleaning()
{
	int32_t i,j;
	for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; ++dy) {
			for (auto dx = -ROBOT_SIZE_1_2; dx <= ROBOT_SIZE_1_2; ++dx){
				CM_count_normalize(Gyro_GetAngle(), CELL_SIZE * dy, CELL_SIZE * dx, &i, &j);
				Map_SetCell(MAP, i, j, CLEANED);
			}
	}

	MotionManage::pubCleanMapMarkers(MAP, g_next_point, g_targets_point);
}

void CM_update_map_obs()
{
	if (Get_OBS_Status() & Status_Left_OBS) {
		int32_t i,j;
		CM_count_normalize(Gyro_GetAngle(), CELL_SIZE_2, CELL_SIZE, &i, &j);
		if (Get_Wall_ADC(0) > 200) {
			if (Map_GetCell(MAP, countToCell(i), countToCell(j)) != BLOCKED_BUMPER) {
				Map_SetCell(MAP, i, j, BLOCKED_OBS); //BLOCKED_OBS);
			}
		}
	}

	if (Get_OBS_Status() & Status_Right_OBS) {
		int32_t i,j;
		CM_count_normalize(Gyro_GetAngle(), -CELL_SIZE_2, CELL_SIZE, &i, &j);
		if (Get_Wall_ADC(1) > 200) {
			if (Map_GetCell(MAP, countToCell(i), countToCell(j)) != BLOCKED_BUMPER) {
				Map_SetCell(MAP, i, j, BLOCKED_OBS); //BLOCKED_OBS);
			}
		}
	}

	for (auto dy = 0; dy < 3; ++dy) {
		auto i = SHRT_MAX;
		switch (dy) {
			case 0:
				i = Get_OBS_Status() & Status_Right_OBS;
				break;
			case 1:
				i = Get_OBS_Status() & Status_Front_OBS;
				break;
			case 2:
				i = Get_OBS_Status() & Status_Left_OBS;
				break;
		}
		int32_t x_tmp, y_tmp;
		CM_count_normalize(Gyro_GetAngle(), (dy - 1) * CELL_SIZE, CELL_SIZE_2, &x_tmp, &y_tmp);
		if (i) {
			if (Map_GetCell(MAP, countToCell(x_tmp), countToCell(y_tmp)) != BLOCKED_BUMPER) {
				Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_OBS);
			}
		} else {
			if (Map_GetCell(MAP, countToCell(x_tmp), countToCell(y_tmp)) == BLOCKED_OBS) {
				Map_SetCell(MAP, x_tmp, y_tmp, UNCLEAN);
			}
		}
	}
}

void CM_update_map_bumper()
{
	auto bumper = Get_Bumper_Status();
	std::vector<Cell_t> d_cells;

	if ((bumper & RightBumperTrig) && (bumper & LeftBumperTrig))
		d_cells = {{2,-1}, {2,0}, {2,1}};
	else if (bumper & LeftBumperTrig) {
		d_cells = {{2, 1}, {2,2},{1,2}};
		if (g_pos_history[0] == g_pos_history[1] && g_pos_history[0] == g_pos_history[2])
			d_cells.push_back({2,0});
	} else if (bumper & RightBumperTrig) {
		d_cells = {{2,-2},{2,-1},{1,-2}};
		if (g_pos_history[0] == g_pos_history[1]  && g_pos_history[0] == g_pos_history[2])
			d_cells.push_back({2,0});
	}

	int32_t	x_tmp, y_tmp;
	for(auto& d_cell : d_cells){
		CM_count_normalize(Gyro_GetAngle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x_tmp, &y_tmp);
		ROS_INFO("%s %d: marking (%d, %d)", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
	}
}

void CM_update_map_cliff()
{
	std::vector<Cell_t> d_cells;
	if (Get_Cliff_Trig() & Status_Cliff_Front){
		d_cells.push_back({2,-1});
		d_cells.push_back({2, 0});
		d_cells.push_back({2, 1});
	}
	if (Get_Cliff_Trig() & Status_Cliff_Left){
		d_cells.push_back({2, 1});
		d_cells.push_back({2, 2});
	}
	if (Get_Cliff_Trig() & Status_Cliff_Right){
		d_cells.push_back({2,-1});
		d_cells.push_back({2,-2});
	}

	int32_t	x_tmp, y_tmp;
	for (auto& d_cell : d_cells) {
		CM_count_normalize(Gyro_GetAngle(), d_cell.Y * CELL_SIZE, d_cell.X * CELL_SIZE, &x_tmp, &y_tmp);
		ROS_INFO("%s %d: marking (%d, %d)", __FUNCTION__, __LINE__, countToCell(x_tmp), countToCell(y_tmp));
		Map_SetCell(MAP, x_tmp, y_tmp, BLOCKED_BUMPER);
	}
}

void CM_update_position(bool is_turn)
{
	auto last_x = Map_GetXCell();
	auto last_y = Map_GetYCell();
	auto pos_x = robot::instance()->getPositionX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	auto pos_y = robot::instance()->getPositionY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	Map_SetPosition(pos_x, pos_y);

	if (last_x != Map_GetXCell() || last_y != Map_GetYCell())
		CM_update_map_cleaning();

	if(! is_turn)
		CM_update_map_obs();

	robot::instance()->pubCleanMarkers();
}

void CM_update_map()
{

	CM_update_map_obs();

	CM_update_map_bumper();

	CM_update_map_cliff();

}

/*--------------------------Head Angle--------------------------------*/
void CM_HeadToCourse(uint8_t speed_max, int16_t angle)
{
	Stop_Brifly();

	CM_event_manager_turn(true);

	TurnSpeedRegulator regulator(speed_max, ROTATE_LOW_SPEED, 4);
	bool		eh_status_now, eh_status_last;
	while (ros::ok()) {
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}

		if (g_fatal_quit_event || g_key_clean_pressed || g_remote_home) {
			break;
		}

		auto diff = ranged_angle(angle - Gyro_GetAngle());

		if (std::abs(diff) < 10) {
			Stop_Brifly();
			CM_update_position(true);
			ROS_INFO("%s %d: angle: %d\tGyro: %d\tDiff: %d", __FUNCTION__, __LINE__, angle, Gyro_GetAngle(), diff);
			break;
		}
		uint8_t speed_up;
		regulator.adjustSpeed(diff, speed_up);
		Set_Wheel_Speed(speed_up, speed_up);

	}

	CM_event_manager_turn(false);
	Set_Wheel_Speed(0, 0);
	CM_update_position(true);
}

bool CM_LinearMoveToPoint(Point32_t Target, int32_t speed_max, bool stop_is_needed, bool rotate_is_needed)
{
	// Reset the g_bumper_status_for_rounding.
	g_bumper_status_for_rounding = 0;
	g_should_follow_wall =  0;
	g_bumper_hitted = g_obs_triggered = g_cliff_triggered = g_rcon_triggered = false;

	Point32_t	position{Map_GetXCount(), Map_GetYCount()};

	CM_update_position();

	if (rotate_is_needed) {
		auto Target_Course = course2dest(Map_GetXCount(), Map_GetYCount(), Target.X, Target.Y);
		CM_HeadToCourse(ROTATE_TOP_SPEED, Target_Course);	//turn to target position
	}

	if (position.X != Map_GetXCount() && position.X == Target.X)
		Target.X = Map_GetXCount();
	else if (position.Y != Map_GetYCount() && position.Y == Target.Y)
		Target.Y = Map_GetYCount();

	CM_set_event_manager_handler_state(true);

	Reset_Rcon_Status();

	LinearSpeedRegulator regulator(speed_max);
	bool	eh_status_now=false, eh_status_last=false;
	while (ros::ok) {
#ifdef WALL_DYNAMIC
		Wall_Dynamic_Base(50);
#endif

#ifdef OBS_DYNAMIC_MOVETOTARGET
		/* Dyanmic adjust obs trigger val . */
		robotbase_OBS_adjust_count(100);
#endif

		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}

		if (g_fatal_quit_event || g_key_clean_pressed || g_remote_home) {
			break;
		}

		if ( g_bumper_hitted || g_obs_triggered || g_cliff_triggered || g_rcon_triggered ) {
			Set_Wheel_Speed(0, 0);
			CM_update_map();
			g_should_follow_wall = 1;

			if (g_cliff_triggered ||g_rcon_triggered)
				g_should_follow_wall = 0;
			break;
		}

		if (std::abs(Map_GetXCount() - Target.X) < 150 && std::abs(Map_GetYCount() - Target.Y) < 150) {
			ROS_INFO("%s, %d: Reach target.", __FUNCTION__, __LINE__);
			CM_update_map();
			break;
		}

		CM_update_position();
//		MotionManage::pubCleanMapMarkers(MAP, Target, Target);

		bool slow_down=false;
		if(check_map_boundary(slow_down))
			break;

		uint8_t left_speed,right_speed;
		if(! regulator.adjustSpeed(Target, left_speed, right_speed, slow_down))
			break;

		Move_Forward(left_speed, right_speed);
	}

	CM_set_event_manager_handler_state(false);

	if (stop_is_needed)
		Stop_Brifly();

	CM_update_position();

	ROS_INFO("%s %d: Gyro Calibration: %d", __FUNCTION__, __LINE__, Gyro_GetCalibration());
	robot::instance()->displayPositions();
	usleep(10000);

	return true;
}

bool CM_TurnMoveToPoint(Point32_t Target,uint8_t speed_left,uint8_t speed_right)
{
	auto angle_start = Gyro_GetAngle();
	Move_Forward(speed_left, speed_right);

	bool eh_status_now = false;
	bool eh_status_last = false;
	CM_set_event_manager_handler_state(true);
	while (ros::ok())
	{
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1)
		{
			usleep(100);
			continue;
		}

		CM_update_position();

		if (g_fatal_quit_event || g_key_clean_pressed)
			return false;

		if (g_bumper_hitted || g_obs_triggered || g_cliff_triggered || g_rcon_triggered)
		{
			CM_update_map();
			return false;
		}

		auto angle_diff = ranged_angle(Gyro_GetAngle() - angle_start);
		if (abs(angle_diff) >= 900){
			ROS_WARN("%s %d: reach line between point 1 & 2: \n", __FUNCTION__, __LINE__);
			break;
		}
	}
	Stop_Brifly();
	CM_update_position();
	return true;
}

void CM_MoveToPoint(Point32_t target)
{
#ifdef PP_CURVE_MOVE
	if (path_get_path_points_count() >= 3){
		if (! CM_CurveMoveToPoint())
			CM_LinearMoveToPoint(target, RUN_TOP_SPEED, true, true);
	}
	else
#endif
		CM_LinearMoveToPoint(target, RUN_TOP_SPEED, true, true);
}

bool CM_CurveMoveToPoint()
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

	Point32_t target{cellToCount(cells[1].X), cellToCount(cells[1].Y)};
#define IS_MOVE_FOLLOW_Y_AXIS_FIRST (cells[0].X == cells[1].X)
#define IS_IN_NAV_X_AXIS (cells[0].X < cells[2].X)
#define IS_IN_NAV_Y_AXIS (cells[0].Y < cells[2].Y)
	if (IS_MOVE_FOLLOW_Y_AXIS_FIRST) {
		target.Y += (IS_IN_NAV_X_AXIS ? -radius : radius);
	} else {
		target.X += (IS_IN_NAV_Y_AXIS ? -radius : radius);
	}

	//1/3 move to first target
	if(! CM_LinearMoveToPoint(target, MAX_SPEED, false, true) )
		return false;

	//2/3 turn to  target
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

	//3/3 move to last route
	if(! CM_TurnMoveToPoint(target, speed_left, speed_right))
		return false;

	CM_set_event_manager_handler_state(false);

	target.X = cellToCount(cells[2].X);
	target.Y = cellToCount(cells[2].Y);

	//3 continue to move to target
	ROS_ERROR("is_speed_right(%d),speed_left(%d),speed_right(%d)",is_speed_right,speed_left,speed_right);
	if(! CM_LinearMoveToPoint(target, MAX_SPEED, true, false))
		return false;

	return true;
}

#if (PP_ROUNDING_OBSTACLE_LEFT) || (PP_ROUNDING_OBSTACLE_RIGHT)

RoundingType CM_get_rounding_direction(Point32_t *Next_Point, Point32_t Target_Point, uint16_t dir) {
	int32_t		y_coordinate;
	RoundingType	rounding_type = ROUNDING_NONE;

	ROS_INFO("Enter rounding detection.");
	if (g_should_follow_wall == 0 || Next_Point->Y == Map_GetYCount()) {
		return rounding_type;
	}

	if (Get_Cliff_Trig()) {
		ROS_INFO("%s %d, cliff triggered.", __FUNCTION__, __LINE__);
	} else if (Get_Bumper_Status()) {
		ROS_INFO("%s %d, bumper event.", __FUNCTION__, __LINE__);
	} else if (Get_OBS_Status()) {
		ROS_INFO("%s %d, OBS detected.", __FUNCTION__, __LINE__);
	} else if (Get_FrontOBS() > Get_FrontOBST_Value()) {
		ROS_INFO("%s %d, front OBS detected.", __FUNCTION__, __LINE__);
	} else if (Get_Wall_ADC(0) > 170) {
		ROS_INFO("%s %d, wall sensor exceed 170.", __FUNCTION__, __LINE__);
	}
	/*					South (Xmin)
	 *						|
	 *	West (Ymin)  --  robot	--	 East (Ymax)
	 *						|
	 *					North (Xmax)
	**/
	y_coordinate = countToCell(Next_Point->Y);
	if (y_coordinate != Map_GetYCell()) {
		// Robot need to go to new line
		ROS_INFO("Robot need to go to new line");

#if PP_ROUNDING_OBSTACLE_LEFT
		if ((dir == POS_X && y_coordinate < Map_GetYCell() && (y_coordinate == Map_GetYCell() - 1 || y_coordinate ==
																																																			 Map_GetYCell() - 2)) ||
			(dir == NEG_X && y_coordinate > Map_GetYCell() && (y_coordinate == Map_GetYCell() + 1 || y_coordinate ==
																																																		 Map_GetYCell() + 2 ))) {

			rounding_type = ROUNDING_LEFT;
		}
#endif

#if PP_ROUNDING_OBSTACLE_RIGHT
		if ((dir == POS_X && y_coordinate > Map_GetYCell() && (y_coordinate == Map_GetYCell() + 1 || y_coordinate ==
																																																			 Map_GetYCell() + 2)) ||
			(dir == NEG_X && y_coordinate < Map_GetYCell() && (y_coordinate == Map_GetYCell() - 1 || y_coordinate ==
																																																		 Map_GetYCell() - 2))) {

			rounding_type = ROUNDING_RIGHT;
		}
#endif
	} else {
		ROS_INFO("%s %d Robot don't need to go to new line. y: %d", __FUNCTION__, __LINE__, y_coordinate);
		if (!(countToCell(Next_Point->X) == SHRT_MAX || countToCell(Next_Point->X) == SHRT_MIN)) {
			y_coordinate = countToCell(Target_Point.Y);
			if (y_coordinate != Map_GetYCell()) {

#if PP_ROUNDING_OBSTACLE_LEFT
				if ((dir == POS_X && y_coordinate < Map_GetYCell() && (y_coordinate == Map_GetYCell() - 1 || y_coordinate ==
																																																					 Map_GetYCell() - 2)) ||
					(dir == NEG_X && y_coordinate > Map_GetYCell() && (y_coordinate == Map_GetYCell() + 1 || y_coordinate ==
																																																				 Map_GetYCell() + 2 ))) {

					rounding_type = ROUNDING_LEFT;
					Next_Point->Y = Target_Point.Y;
				}
#endif

#if PP_ROUNDING_OBSTACLE_RIGHT
				if ((dir == POS_X && y_coordinate > Map_GetYCell() && (y_coordinate == Map_GetYCell() + 1 || y_coordinate ==
																																																					 Map_GetYCell() + 2)) ||
					(dir == NEG_X && y_coordinate < Map_GetYCell() && (y_coordinate == Map_GetYCell() - 1 || y_coordinate ==
																																																				 Map_GetYCell() - 2))) {

					rounding_type = ROUNDING_RIGHT;
					Next_Point->Y = Target_Point.Y;
				}
#endif

			}
		}
	}
	return rounding_type;
}

void CM_rounding_turn(uint16_t speed, int16_t angle)
{
	uint8_t		accurate;
	int16_t		target_angle;
	uint16_t	speed_;

	bool eh_status_now, eh_status_last;

	eh_status_now = eh_status_last = false;

	Stop_Brifly();

	if (g_rounding_type == ROUNDING_RIGHT) {
		target_angle = Gyro_GetAngle() + angle + (Gyro_GetAngle() + angle >= 3600 ? (-3600) : 0);
	} else {
		target_angle = Gyro_GetAngle() - angle + (Gyro_GetAngle() - angle < 0 ? 3600 : 0);
	}
	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed);

	should_mark = 0;
	accurate = speed > 30 ? 30 : 10;
	while (ros::ok()) {
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}

		if (g_fatal_quit_event == true || g_key_clean_pressed == true) {
			break;
		}

		if (abs(target_angle - Gyro_GetAngle()) < accurate) {
			break;
		}

		if (g_rounding_type == ROUNDING_RIGHT) {
			Set_Dir_Left();
		} else {
			Set_Dir_Right();
		}

		speed_ = speed;
		if (abs(target_angle - Gyro_GetAngle()) < 50) {
			speed_ = std::min((uint16_t)5, speed);
		} else if (abs(target_angle - Gyro_GetAngle()) < 200) {
			speed_ = std::min((uint16_t)10, speed);
		}
		ROS_DEBUG("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed_);
		Set_Wheel_Speed(speed_, speed_);

		CM_update_position(true);
		ROS_DEBUG("%s %d: angle: %d(%d)\tcurrent: %d\tspeed: %d", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle(), speed);
	}

	Set_Dir_Forward();
	Set_Wheel_Speed(0, 0);

	CM_update_position(true);
	should_mark = 1;

	ROS_INFO("%s %d: angle: %d(%d)\tcurrent: %d\n", __FUNCTION__, __LINE__, angle, target_angle, Gyro_GetAngle());
}

uint8_t CM_rounding(RoundingType type, Point32_t target, uint8_t Origin_Bumper_Status)
{
	bool		eh_status_now, eh_status_last;
	int16_t		angle, *buffer_ptr;
	int32_t		y_start, Proportion = 0, Delta = 0, Previous = 0, speed_left = 0, speed_right = 0;

	g_rounding_left_wall_buffer[3] = { 0 }, g_rounding_right_wall_buffer[3] = { 0 };
	g_rounding_wall_distance = 400;
	eh_status_now = eh_status_last = false;

	g_rounding_type = type;
	y_start = Map_GetYCount();
	ROS_INFO("%s %d: Y: %d\tY abs: %d\ttarget Y: %d\ttarget Y abs: %d", __FUNCTION__, __LINE__, Map_GetYCount(), abs(Map_GetYCount()), target.Y, abs(target.Y));

	if ((Origin_Bumper_Status & LeftBumperTrig) && !(Origin_Bumper_Status & RightBumperTrig)) {
		ROS_INFO("%s %d: Left bumper", __FUNCTION__, __LINE__);
		angle = (type == ROUNDING_LEFT) ? 450 : 1350;
	} else if (!(Origin_Bumper_Status & LeftBumperTrig) && (Origin_Bumper_Status & RightBumperTrig)) {
		ROS_INFO("%s %d: Right bumper", __FUNCTION__, __LINE__);
		angle = (type == ROUNDING_LEFT) ? 1350 : 450;
	} else {
		// If bumper not hit or it just hit the front (Both left and right bumper triggered, robot should turn 90 degrees.
		ROS_INFO("%s %d: Same bumper", __FUNCTION__, __LINE__);
		angle = 900;
	}
	ROS_INFO("%s %d: ROUNDING_%s, turn %d degrees.", __FUNCTION__, __LINE__, type == ROUNDING_LEFT ? "LEFT" : "RIGHT", angle);
	CM_rounding_turn(TURN_SPEED, angle);

	CM_set_event_manager_handler_state(true);

	should_mark = 1;
	g_rounding_wall_straight_distance = 300;
	while (ros::ok()) {
#ifdef OBS_DYNAMIC
		robotbase_OBS_adjust_count(100);
#endif

		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}
		if (g_fatal_quit_event == true || g_key_clean_pressed == true) {
			break;
		}

		CM_update_position();

		if ((y_start > target.Y && Map_GetYCount() < target.Y) || (y_start < target.Y && Map_GetYCount() > target.Y)) {
			// Robot has reach the target.
			ROS_INFO("%s %d: Y: %d\tY abs: %d\ttarget Y: %d\ttarget Y abs: %d", __FUNCTION__, __LINE__, Map_GetYCount(), abs(Map_GetYCount()), target.Y, abs(target.Y));
			break;
		}

		/* Tolerance of distance allow to move back when roundinging the obstcal. */
		if ((target.Y > y_start && (y_start - Map_GetYCount()) > 120) || (target.Y < y_start && (Map_GetYCount() - y_start) > 120)) {
			// Robot has round to the opposite direcition.
			ROS_INFO("%s %d: Y: %d position: (%d, %d)", __FUNCTION__, __LINE__, target.Y, Map_GetXCount(), Map_GetYCount());

			Map_SetCell(MAP, Map_GetRelativeX(Gyro_GetAngle(), CELL_SIZE_3, 0), Map_GetRelativeY(Gyro_GetAngle(), CELL_SIZE_3, 0), CLEANED);
			break;
		}

		if (g_rounding_turn_angle != 0) {
			CM_rounding_turn(TURN_SPEED, g_rounding_turn_angle);
			g_rounding_turn_angle = 0;
			Move_Forward(g_rounding_move_speed, g_rounding_move_speed);
		}

		if (g_rounding_wall_distance >= 200) {
			buffer_ptr = (type == ROUNDING_LEFT) ? g_rounding_left_wall_buffer : g_rounding_right_wall_buffer;
			buffer_ptr[2] = buffer_ptr[1];
			buffer_ptr[1] = buffer_ptr[0];
			buffer_ptr[0] = (type == ROUNDING_LEFT) ? robot::instance()->getLeftWall() : robot::instance()->getRightWall();
			if (buffer_ptr[0] < 100) {
				if ((buffer_ptr[1] - buffer_ptr[0]) > (g_rounding_wall_distance / 25)) {
					if ((buffer_ptr[2] - buffer_ptr[1]) > (g_rounding_wall_distance / 25)) {
						//if (Get_WallAccelerate()>300) {
							//if ((type == ROUNDING_LEFT && (Get_RightWheel_Speed()- Get_LeftWheel_Speed()) >= -3) ||
							//	(type == ROUNDING_RIGHT && (Get_LeftWheel_Speed()- Get_RightWheel_Speed()) >= -3)) {
								// Away from the wall.
								speed_left = type == ROUNDING_LEFT ? 18 : 16;
								speed_right = type == ROUNDING_LEFT ? 16 : 18;
								Move_Forward(speed_left, speed_right);
								usleep(100000);
								g_rounding_wall_straight_distance = 300;
							//}
						//}
					}
				}
			}
		}

		/*------------------------------------------------------Wheel Speed adjustment-----------------------*/
		if (Get_FrontOBS() < Get_FrontOBST_Value()) {
			Proportion = (type == ROUNDING_LEFT ? robot::instance()->getLeftWall() : robot::instance()->getRightWall()) * 100 / g_rounding_wall_distance - 100;
			Delta = Proportion - Previous;

			if (g_rounding_wall_distance > 300) {
				speed_left = 25 + Proportion / 12 + Delta / 5;
				speed_right = 25 - Proportion / 10 - Delta / 5;
				if (speed_right > 33) {
					speed_left = 5;
					speed_right = 33;
				}
			} else {
				speed_right = 15 - Proportion / 18 - Delta / 10;
				speed_left = (speed_right > 18) ? 5 : (15 + Proportion / 22 + Delta / 10);

				check_limit(speed_left, 4, 20);
				check_limit(speed_right, 4, 18);
				speed_left = (speed_left - speed_right) > 5 ? speed_right + 5 : speed_left;
			}

			if ((type == ROUNDING_LEFT && Get_LeftOBS() > Get_LeftOBST_Value()) || (type == ROUNDING_RIGHT && Get_RightOBS() > Get_RightOBST_Value())) {
				if (g_rounding_wall_distance < Wall_High_Limit) {
					g_rounding_wall_distance++;
				}
			}
			if (Is_WallOBS_Near()){
				speed_left /= 2;
				speed_right /= 2;
			}

			Previous = Proportion;
			check_limit(speed_left, 0, 40);
			speed_right = speed_right < 0 ? 0 : speed_right;

			Move_Forward((type == ROUNDING_LEFT ? speed_left : speed_right), (type == ROUNDING_LEFT ? speed_right : speed_left));
		} else {
			angle = 900;
			if ((type == ROUNDING_LEFT && Get_LeftWheel_Step() < 12500) || (type == ROUNDING_RIGHT && Get_RightWheel_Step() < 12500)) {
				angle = Get_FrontOBS() > Get_FrontOBST_Value() ? 800 : 400;
			}
			CM_rounding_turn(TURN_SPEED, angle);
			Move_Forward(15, 15);
			g_rounding_wall_distance = Wall_High_Limit;
		}
	}
	CM_set_event_manager_handler_state(false);

	Stop_Brifly();
	return 0;
}
#endif

bool CM_resume_cleaning()
{
	robot::instance()->resetLowBatPause();

	CM_MoveToCell(countToCell(g_continue_point.X), countToCell(g_continue_point.Y));
	if (g_fatal_quit_event || g_key_clean_pressed)
	{
		robot::instance()->resetLowBatPause();
		return false;
	}
	return true;
}

void linearMarkClean(const Cell_t& start, const Cell_t& target)
{
	if (start.Y == target.Y)
	{
		Cell_t stop = {Map_GetXCell(), Map_GetYCell()};
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
					Map_SetCell(MAP, cellToCount(x), cellToCount(y + dy), CLEANED);
			}
		}
	}
}

int CM_cleaning()
{
	// Checking g_go_home is for case that manual pause when robot going home.
	if(g_go_home)
	{
		ROS_WARN("%s %d: Continue going home.", __FUNCTION__, __LINE__);
		return 0;
	}

	while (ros::ok())
	{
		if (g_key_clean_pressed || g_fatal_quit_event)
			return -1;

		if (g_remote_home || g_battery_home)
		{
			g_remote_home = false;
			return 0;
		}

		uint16_t last_dir = path_get_robot_direction();

		Cell_t start{Map_GetXCell(), Map_GetYCell()};
		int8_t state = path_next(&g_next_point.X, &g_next_point.Y, &g_targets_point);
		MotionManage::pubCleanMapMarkers(MAP, g_next_point, g_targets_point);
		ROS_ERROR("State: %d", state);
		if (state == 0) //No target point
		{
			g_go_home = true;
			return 0;
		} else
		if (state == 1)
		{
			auto rounding_type = CM_get_rounding_direction(&g_next_point, g_targets_point, last_dir);
			if (rounding_type != ROUNDING_NONE)
			{
				ROS_INFO("%s %d: Rounding %s.", __FUNCTION__, __LINE__, rounding_type == ROUNDING_LEFT ? "left" : "right");
				g_cm_move_type = CM_ROUNDING;
				CM_rounding(rounding_type, g_next_point, g_bumper_status_for_rounding);
				g_cm_move_type = CM_LINEARMOVE;
			} else
				CM_MoveToPoint(g_next_point);
			linearMarkClean(start,Map_PointToCell(g_next_point));

		} else
		if (state == 2)
		{    // Trapped
			/* FIXME: disable events, since Wall_Follow_Trapped() doesn't handle events for now. */
			CM_set_event_manager_handler_state(false);
			auto es_state = Wall_Follow_Trapped();
			event_manager_set_current_mode(EVT_MODE_NAVIGATION);
			CM_set_event_manager_handler_state(true);

			if (g_go_home)
				return 0;

			if (es_state != Escape_Trapped_Escaped)
			{
				Disable_Motors();
				if (es_state == Escape_Trapped_Key_Clean)
				{
					g_key_clean_pressed = true;
				} else if (es_state == Escape_Trapped_Fatal)
				{
					g_fatal_quit_event = true;
				}
				return -1;
			}
		}
	}
	return 0;
}

void CM_go_home()
{
	Set_VacMode(Vac_Normal, false);
	Set_Vac_Speed();

	if(robot::instance()->isLowBatPaused())
		wav_play(WAV_BATTERY_LOW);
	wav_play(WAV_BACK_TO_CHARGER);

	if (!robot::instance()->isLowBatPaused() && !g_map_boundary_created)
		CM_create_home_boundary();
	Cell_t current_home_cell = {countToCell(g_home_point.front().X), countToCell(g_home_point.front().Y)};
	ROS_WARN("%s, %d: Go home Target: (%d, %d), %u targets left.", __FUNCTION__, __LINE__, current_home_cell.X, current_home_cell.Y,
					 (uint) g_home_point.size());
	g_home_point.pop_front();

	while (ros::ok())
	{
		Set_LED(100, 0);
		// Set clean mode to navigation so GoHome() function will know this is during navigation mode.
		Set_Clean_Mode(Clean_Mode_Navigation);
		ROS_INFO("%s %d: Current Battery level: %d.", __FUNCTION__, __LINE__, GetBatteryVoltage());
		if (!CM_MoveToCell(current_home_cell.X, current_home_cell.Y))
		{
			if (g_fatal_quit_event)
			{
				// Fatal quit means cliff is triggered / bumper jamed / any over current event.
				Disable_Motors();
				robot::instance()->resetLowBatPause();
				if (g_battery_low)
					Set_Clean_Mode(Clean_Mode_Sleep);
				else
					Set_Clean_Mode(Clean_Mode_Userinterface);
				ROS_WARN("%s %d: Fatal quit and finish cleanning, cleaning time: %d(s).", __FUNCTION__, __LINE__, Get_Work_Time());
				g_cur_wtime = 0;
				ROS_INFO("%s ,%d ,set g_cur_wtime to zero",__FUNCTION__,__LINE__);
				CM_ResetGoHome();
				return;
			}
			if (g_key_clean_pressed)
			{
				Disable_Motors();
				Set_Clean_Mode(Clean_Mode_Userinterface);
#if MANUAL_PAUSE_CLEANING
				if (robot::instance()->isManualPaused())
					// The current home cell is still valid, so push it back to the home point list.
					CM_SetHome(cellToCount(current_home_cell.X), cellToCount(current_home_cell.Y));
				ROS_WARN("%s %d: Pause cleanning, cleaning time: %d(s), g_home_point list size: %u.", __FUNCTION__, __LINE__, Get_Work_Time()+g_cur_wtime, (uint)g_home_point.size());
#else
				ROS_WARN("%s %d: Clean key pressed, cleaning time: %d(s).", __FUNCTION__, __LINE__, Get_Work_Time()+g_cur_wtime);
#endif
				g_cur_wtime = 0;
				ROS_INFO("%s ,%d ,set g_cur_wtime to zero",__FUNCTION__,__LINE__);
				return;
			}
		}
		else
		{
			if (CM_go_to_charger(current_home_cell))
				return;
		}

		if (g_home_point.empty())
		{
			// If it is the last point, it means it it now at (0, 0).
			if (!g_from_station) {
				auto angle = static_cast<int16_t>(robot::instance()->offsetAngle() *10);
				CM_HeadToCourse(ROTATE_TOP_SPEED, -angle);
			}
			Disable_Motors();
			robot::instance()->resetLowBatPause();
			Set_Clean_Mode(Clean_Mode_Userinterface);
			ROS_WARN("%s %d: Can not go to charger stub after going to all home points. Finish cleaning, cleaning time: %d(s).", __FUNCTION__, __LINE__, Get_Work_Time()+g_cur_wtime);
			g_cur_wtime = 0;
			ROS_INFO("%s ,%d ,set g_cur_wtime to zero",__FUNCTION__,__LINE__);
			CM_ResetGoHome();
			return;
		}
		else
		{
			// Get next home cell.
			current_home_cell.X = countToCell(g_home_point.front().X);
			current_home_cell.Y = countToCell(g_home_point.front().Y);
			g_home_point.pop_front();
			ROS_WARN("%s, %d: Go home Target: (%d, %d), %u targets left.", __FUNCTION__, __LINE__, current_home_cell.X, current_home_cell.Y, (uint)g_home_point.size());
		}
	}
}

/* Statement for CM_go_to_charger(void)
 * return : true -- going to charger has been stopped, either successfully or interrupted.
 *          false -- going to charger failed, move to next point.
 */
bool CM_go_to_charger(Cell_t current_home_cell)
{
	// Call GoHome() function to try to go to charger stub.
	ROS_WARN("Call GoHome()");
	GoHome();
	// In GoHome() function the clean mode might be set to Clean_Mode_GoHome, it should keep try GoHome().
	while (Get_Clean_Mode() == Clean_Mode_GoHome)
	{
		// Set clean mode to navigation so GoHome() function will know this is during navigation mode.
		Set_Clean_Mode(Clean_Mode_Navigation);
		ROS_INFO("%s,%d set clean mode gohome",__FUNCTION__,__LINE__);
		GoHome();
	}
	// Check the clean mode to find out whether it has reach the charger.
	if (Get_Clean_Mode() == Clean_Mode_Charging)
	{
		if (robot::instance()->isLowBatPaused())
		{
			ROS_WARN("%s %d: Pause cleaning for low battery, will continue cleaning when charge finished. Current cleaning time: %d(s)", __FUNCTION__, __LINE__, Get_Work_Time()+g_cur_wtime);
			g_cur_wtime = g_cur_wtime+Get_Work_Time();//store current time
			Reset_Work_Time();//reset current time
			CM_ResetGoHome();
			return true;
		}
		ROS_WARN("%s %d: Finish cleaning and stop in charger stub, cleaning time: %d(s)", __FUNCTION__, __LINE__, Get_Work_Time()+g_cur_wtime);
		CM_ResetGoHome();
		return true;
	}
	else if (Get_Clean_Mode() == Clean_Mode_Sleep)
	{
		// Battery too low.
		Disable_Motors();
		robot::instance()->resetLowBatPause();
		ROS_WARN("%s %d: Battery too low, cleaning time: %d(s)", __FUNCTION__, __LINE__, Get_Work_Time()+g_cur_wtime);
		g_cur_wtime = 0;
		ROS_INFO("%s ,%d ,set g_cur_wtime to zero",__FUNCTION__,__LINE__);
		CM_ResetGoHome();
		return true;
	}
	// FIXME: else if (g_fatal_quit_event || g_key_clean_pressed)
	else if (Stop_Event())
	{
		Disable_Motors();
		Set_Clean_Mode(Clean_Mode_Userinterface);
#if MANUAL_PAUSE_CLEANING
		// FIXME: if (g_key_clean_pressed)
		if (Stop_Event() == 1 || Stop_Event() == 2)
		{
			Reset_Stop_Event_Status();
			// The current home cell is still valid, so push it back to the home point list.
			CM_SetHome(cellToCount(current_home_cell.X), cellToCount(current_home_cell.Y));
			ROS_WARN("%s %d: Pause cleanning, cleaning time: %d(s), g_home_point list size: %u.", __FUNCTION__, __LINE__, Get_Work_Time()+g_cur_wtime, (uint)g_home_point.size());
			return true;
		}
#endif
		robot::instance()->resetLowBatPause();
		Reset_Stop_Event_Status();
		ROS_WARN("%s %d: Finish cleanning, cleaning time: %d(s)", __FUNCTION__, __LINE__, Get_Work_Time()+g_cur_wtime);
		g_cur_wtime = 0;
		ROS_INFO("%s ,%d ,set g_cur_wtime to zero",__FUNCTION__,__LINE__);
		CM_ResetGoHome();
		return true;
	}

	return false;
}

uint8_t CM_Touring(void)
{
	g_cm_move_type = CM_LINEARMOVE;
	g_fatal_quit_event = false;
	g_bumper_jam = false;
	g_cliff_all_triggered = false;
	g_cliff_jam = false;
	g_cliff_triggered = false;
	g_rcon_triggered = false;
	g_oc_brush_main = g_oc_wheel_left = g_oc_wheel_right = g_oc_suction = false;
	g_key_clean_pressed = false;
	g_remote_home = false;
	g_battery_home = g_battery_low = false;
	g_oc_brush_left_cnt = g_oc_brush_main_cnt = g_oc_brush_right_cnt = g_oc_wheel_left_cnt = g_oc_wheel_right_cnt = g_oc_suction_cnt = 0;
	g_cliff_cnt = 0;
	g_bumper_cnt = g_press_time = 0;
	g_from_station = 0;

	MotionManage motion;

	if(! motion.initSucceeded()){
		robot::instance()->resetLowBatPause();
		robot::instance()->resetManualPause();
		return 0;
	}

	CM_regist_events();

	if (!g_go_home && (robot::instance()->isLowBatPaused()))
		if (! CM_resume_cleaning())
			return 0;

	if (CM_cleaning() == 0)
		CM_go_home();

	CM_unregist_events();
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
bool CM_MoveToCell( int16_t target_x, int16_t target_y)
{
	if (is_block_accessible(target_x, target_y) == 0) {
		ROS_WARN("%s %d: target is blocked.\n", __FUNCTION__, __LINE__);
		Map_Set_Cells(ROBOT_SIZE, target_x, target_y, CLEANED);
	}

	ROS_INFO("%s %d: Path Find: target: (%d, %d)", __FUNCTION__, __LINE__, target_x, target_y);
	Map_Set_Cells(ROBOT_SIZE, target_x, target_y, CLEANED);

	while (ros::ok()) {
		Cell_t pos{target_x, target_y};
		Cell_t	tmp;
		auto pathFind = (int8_t)path_move_to_unclean_area(pos, Map_GetXCell(), Map_GetYCell(), &tmp.X, &tmp.Y);

		ROS_INFO("%s %d: Path Find: %d\tTarget: (%d, %d)\tNow: (%d, %d)", __FUNCTION__, __LINE__, pathFind, tmp.X, tmp.Y,
						 Map_GetXCell(), Map_GetYCell());
		if ( pathFind == 1 || pathFind == SCHAR_MAX ) {
			path_set_current_pos();

			if (CM_CheckLoopBack(tmp) == 1)
				return false;

			ROS_INFO("%s %d: Move to target...", __FUNCTION__, __LINE__ );
			debug_map(MAP, tmp.X, tmp.Y);
			Point32_t	Next_Point{ cellToCount(tmp.X), cellToCount(tmp.Y) };
			CM_MoveToPoint(Next_Point);

			if (g_fatal_quit_event || g_key_clean_pressed )
				return false;

			if (g_remote_home && !g_go_home )
				return false;

			//Arrive exit cell, set < 3 when ROBOT_SIZE == 5
			if ( TwoPointsDistance( target_x , target_y , Map_GetXCell(), Map_GetYCell() ) < ROBOT_SIZE / 2 + 1 ) {
				ROS_WARN("%s %d: Now: (%d, %d)\tDest: (%d, %d)", __FUNCTION__, __LINE__, Map_GetXCell(), Map_GetYCell(), target_x , target_y);
				return true;
			}
		} else
			return false;
	}
	return false;
}

/*-------------- Move Back -----------------------------*/
void CM_CorBack(uint16_t dist)
{
	float pos_x, pos_y, distance;
	uint32_t SP = 10;
	uint16_t Counter_Watcher = 0;

	ROS_INFO("%s %d: Moving back...", __FUNCTION__, __LINE__);
	Stop_Brifly();
	CM_update_position();
	Set_Dir_Backward();
	Set_Wheel_Speed(8, 8);
	Reset_Wheel_Step();
	Counter_Watcher = 0;

	pos_x = robot::instance()->getOdomPositionX();
	pos_y = robot::instance()->getOdomPositionY();

	while (1) {
		distance = sqrtf(powf(pos_x - robot::instance()->getOdomPositionX(), 2) + powf(pos_y -
																																													 robot::instance()->getOdomPositionY(), 2));
		if (fabsf(distance) > 0.02f) {
			break;
		}

		CM_update_position();
		usleep(10000);
		Counter_Watcher++;
		SP = 8 + Counter_Watcher / 100;
		SP = (SP > 18) ? 18 : SP;

		Set_Wheel_Speed(SP, SP);
		if (Counter_Watcher > 3000) {
			if (Is_Encoder_Fail()) {
				Set_Error_Code(Error_Code_Encoder);
			}
			break;
		}
		if (Stop_Event()) {
			ROS_INFO("%s %d: Stop event!", __FUNCTION__, __LINE__);
			Stop_Brifly();
			break;
		}
		uint8_t octype = Check_Motor_Current();
		if ((octype == Check_Left_Wheel) || ( octype  == Check_Right_Wheel)) {
			ROS_INFO("%s ,%d, motor over current",__FUNCTION__,__LINE__);
			break;
		}
	}
	CM_update_position();
	Reset_TempPWM();
	Stop_Brifly();
	ROS_INFO("%s %d: Moving back done!", __FUNCTION__, __LINE__);
}

void CM_ResetGoHome(void)
{
	ROS_DEBUG("Reset go home flags here.");
	g_go_home = false;
	g_map_boundary_created = false;
}

void CM_SetHome(int32_t x, int32_t y) {
	Cell_t tmpPnt;

	bool found = false;
	Point32_t new_home_point;

	ROS_INFO("%s %d: Push new reachable home: (%d, %d) to home point list.", __FUNCTION__, __LINE__, countToCell(x), countToCell(y));
	new_home_point.X = x;
	new_home_point.Y = y;

	for (list<Point32_t>::iterator it = g_home_point.begin(); found == false && it != g_home_point.end(); ++it) {
		if (it->X == x && it->Y == y) {
			found = true;
		}
	}
	if (found == false) {
		g_home_point.push_front(new_home_point);
		// If new_home_point near (0, 0)
		if (abs(countToCell(x)) <= 5 && abs(countToCell(y)) <= 5)
		{
			// Update the trapped reference points
			tmpPnt.X = countToCell(x);
			tmpPnt.Y = countToCell(y);
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

void CM_SetContinuePoint(int32_t x, int32_t y)
{
	ROS_INFO("%s %d: Set continue point: (%d, %d).", __FUNCTION__, __LINE__, countToCell(x), countToCell(y));
	g_continue_point.X = x;
	g_continue_point.Y = y;
}

uint8_t CM_CheckLoopBack( Cell_t target ) {
	uint8_t retval = 0;
	if ( target.X == g_pos_history[1].x && target.Y == g_pos_history[1].y &&
		 target.X == g_pos_history[3].x && target.Y == g_pos_history[3].y ) {
		ROS_WARN("%s %d Possible loop back (%d, %d)", __FUNCTION__, __LINE__, target.X, target.Y);
		retval	= 1;
	}

	return retval;
}

void CM_create_home_boundary(void)
{
	int16_t i, j, k;
	int16_t xMinSearch, xMaxSearch, yMinSearch, yMaxSearch;

	k = 3;
	xMinSearch = xMaxSearch = yMinSearch = yMaxSearch = SHRT_MAX;
	for (i = g_x_min; xMinSearch == SHRT_MAX; i++) {
		for (j = g_y_min; j <= g_y_max; j++) {
			if (Map_GetCell(MAP, i, j) != UNCLEAN) {
				xMinSearch = i - k;
				break;
			}
		}
	}
	for (i = g_x_max; xMaxSearch == SHRT_MAX; i--) {
		for (j = g_y_min; j <= g_y_max; j++) {
			if (Map_GetCell(MAP, i, j) != UNCLEAN) {
				xMaxSearch = i + k;
				break;
			}
		}
	}
	for (i = g_y_min; yMinSearch == SHRT_MAX; i++) {
		for (j = g_x_min; j <= g_x_max; j++) {
			if (Map_GetCell(MAP, j, i) != UNCLEAN) {
				yMinSearch = i - k;
				break;
			}
		}
	}
	for (i = g_y_max; yMaxSearch == SHRT_MAX; i--) {
		for (j = g_x_min; j <= g_x_max; j++) {
			if (Map_GetCell(MAP, j, i) != UNCLEAN) {
				yMaxSearch = i + k;
				break;
			}
		}
	}
	ROS_INFO("%s %d: x: %d - %d\ty: %d - %d", __FUNCTION__, __LINE__, xMinSearch, xMaxSearch, yMinSearch, yMaxSearch);
	for (i = xMinSearch; i <= xMaxSearch; i++) {
		if (i == xMinSearch || i == xMaxSearch) {
			for (j = yMinSearch; j <= yMaxSearch; j++) {
				Map_SetCell(MAP, cellToCount(i), cellToCount(j), BLOCKED_BUMPER);
			}
		} else {
			Map_SetCell(MAP, cellToCount(i), cellToCount(yMinSearch), BLOCKED_BUMPER);
			Map_SetCell(MAP, cellToCount(i), cellToCount(yMaxSearch), BLOCKED_BUMPER);
		}
	}

	// Set the flag.
	g_map_boundary_created = true;
}

/* Event handler functions. */
void CM_regist_events()
{
	event_manager_set_current_mode(EVT_MODE_NAVIGATION);

	/* Bumper */
	event_manager_register_handler(EVT_BUMPER_ALL, &CM_handle_bumper_all);
	event_manager_register_handler(EVT_BUMPER_LEFT, &CM_handle_bumper_left);
	event_manager_register_handler(EVT_BUMPER_RIGHT, &CM_handle_bumper_right);

#define event_manager_register_and_enable_x(name, y, enabled) \
	event_manager_register_handler(y, &CM_handle_ ##name); \
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
	event_manager_register_and_enable_x(rcon_front_left, EVT_RCON_FRONT_LEFT, false);
	event_manager_register_and_enable_x(rcon_front_left2, EVT_RCON_FRONT_LEFT2, false);
	event_manager_register_and_enable_x(rcon_front_right, EVT_RCON_FRONT_RIGHT, false);
	event_manager_register_and_enable_x(rcon_front_right2, EVT_RCON_FRONT_RIGHT2, false);
	event_manager_register_and_enable_x(rcon_left, EVT_RCON_LEFT, false);
	event_manager_register_and_enable_x(rcon_right, EVT_RCON_RIGHT, false);

	/* Over Current */
	event_manager_register_and_enable_x(over_current_brush_left, EVT_OVER_CURRENT_BRUSH_LEFT, true);
	event_manager_register_and_enable_x(over_current_brush_main, EVT_OVER_CURRENT_BRUSH_MAIN, true);
	event_manager_register_and_enable_x(over_current_brush_right, EVT_OVER_CURRENT_BRUSH_RIGHT, true);
	event_manager_register_and_enable_x(over_current_wheel_left, EVT_OVER_CURRENT_WHEEL_LEFT, true);
	event_manager_register_and_enable_x(over_current_wheel_right, EVT_OVER_CURRENT_WHEEL_RIGHT, true);
	event_manager_register_and_enable_x(over_current_suction, EVT_OVER_CURRENT_SUCTION, true);

	/* Key */
	event_manager_register_and_enable_x(key_clean, EVT_KEY_CLEAN, true);

	/* Remote */
	event_manager_register_and_enable_x(remote_plan, EVT_REMOTE_APPOINMENT, true);
	event_manager_register_and_enable_x(remote_clean, EVT_REMOTE_CLEAN, true);
	event_manager_register_and_enable_x(remote_home, EVT_REMOTE_HOME, true);
	event_manager_register_and_enable_x(remote_mode_spot, EVT_REMOTE_MODE_SPOT, true);
	event_manager_register_and_enable_x(remote_suction, EVT_REMOTE_SUCTION, true);

	/* Battery */
	event_manager_register_and_enable_x(battery_home, EVT_BATTERY_HOME, true);
	event_manager_register_and_enable_x(battery_low, EVT_BATTERY_LOW, true);

#undef event_manager_register_and_enable_x

	event_manager_set_enable(true);
}

void CM_unregist_events()
{
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
	event_manager_register_and_disable_x(EVT_RCON_FRONT_LEFT);
	event_manager_register_and_disable_x(EVT_RCON_FRONT_LEFT2);
	event_manager_register_and_disable_x(EVT_RCON_FRONT_RIGHT);
	event_manager_register_and_disable_x(EVT_RCON_FRONT_RIGHT2);
	event_manager_register_and_disable_x(EVT_RCON_LEFT);
	event_manager_register_and_disable_x(EVT_RCON_RIGHT);

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
	event_manager_register_and_disable_x(EVT_REMOTE_APPOINMENT);
	event_manager_register_and_disable_x(EVT_REMOTE_CLEAN);
	event_manager_register_and_disable_x(EVT_REMOTE_HOME);
	event_manager_register_and_disable_x(EVT_REMOTE_MODE_SPOT);
	event_manager_register_and_disable_x(EVT_REMOTE_SUCTION);

	/* Battery */
	event_manager_register_and_disable_x(EVT_BATTERY_HOME);
	event_manager_register_and_disable_x(EVT_BATTERY_LOW);

#undef event_manager_register_and_disable_x

	event_manager_set_current_mode(EVT_MODE_USER_INTERFACE);

	event_manager_set_enable(false);
}

void CM_set_event_manager_handler_state(bool state)
{
	event_manager_enable_handler(EVT_BUMPER_ALL, state);
	event_manager_enable_handler(EVT_BUMPER_LEFT, state);
	event_manager_enable_handler(EVT_BUMPER_RIGHT, state);

	event_manager_enable_handler(EVT_OBS_FRONT, state);
	//event_manager_enable_handler(EVT_OBS_LEFT, state);
	//event_manager_enable_handler(EVT_OBS_RIGHT, state);

	event_manager_enable_handler(EVT_RCON_FRONT_LEFT, state);
	event_manager_enable_handler(EVT_RCON_FRONT_LEFT2, state);
	event_manager_enable_handler(EVT_RCON_FRONT_RIGHT, state);
	event_manager_enable_handler(EVT_RCON_FRONT_RIGHT2, state);
	event_manager_enable_handler(EVT_RCON_LEFT, state);
	event_manager_enable_handler(EVT_RCON_RIGHT, state);
}

void CM_event_manager_turn(bool state)
{
	event_manager_enable_handler(EVT_BUMPER_ALL, state);
	event_manager_enable_handler(EVT_BUMPER_LEFT, state);
	event_manager_enable_handler(EVT_BUMPER_RIGHT, state);
}

void CM_handle_bumper_all(bool state_now, bool state_last)
{
	uint8_t isBumperTriggered = Get_Bumper_Status();

	g_bumper_status_for_rounding = isBumperTriggered;

	Set_Wheel_Speed(0, 0);
	if (g_cm_move_type == CM_LINEARMOVE) {
		CM_update_map_bumper();
	}

	g_bumper_hitted = true;
	ROS_WARN("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, Get_Bumper_Status(), state_now ? "true" : "false", state_last ? "true" : "false");
	if (state_now == true && state_last == true) {
		g_bumper_cnt++;

		if (g_bumper_cnt > 2) {
			ROS_WARN("%s %d: all bumper jam, should quit, jam count: %d", __FUNCTION__, __LINE__, g_bumper_cnt);
			g_bumper_jam = g_fatal_quit_event = true;
		}
	} else {
		g_bumper_cnt = 0;
	}

	CM_CorBack(COR_BACK_20MM);

	if (g_cm_move_type == CM_ROUNDING) {
		if (g_rounding_type == ROUNDING_LEFT) {
			g_rounding_wall_distance = robot::instance()->getLeftWall() > (Wall_Low_Limit) ?
															 robot::instance()->getLeftWall() / 3 : g_rounding_wall_distance + 200;
			for (int i = 0; i < 3; i++) {
					g_rounding_left_wall_buffer[i] = 0;
			}
		} else {
			g_rounding_wall_distance = robot::instance()->getLeftWall() > (Wall_Low_Limit) ?
															 robot::instance()->getLeftWall() / 3 : g_rounding_wall_distance + 200;
			for (int i = 0; i < 3; i++) {
				g_rounding_right_wall_buffer[i] = 0;
			}
		}
		check_limit(g_rounding_wall_distance, Wall_Low_Limit, Wall_High_Limit)

		g_rounding_turn_angle = 900;
		g_rounding_move_speed = 10;
		g_rounding_wall_straight_distance = 150;
	}

	ROS_WARN("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, Get_Bumper_Status());
	if(Get_Bumper_Status() == 0) g_bumper_cnt = 0;
}

void CM_handle_bumper_left(bool state_now, bool state_last)
{
	uint8_t isBumperTriggered = Get_Bumper_Status();

	g_bumper_status_for_rounding = isBumperTriggered;

	Set_Wheel_Speed(0, 0);
	if (g_cm_move_type == CM_LINEARMOVE) {
		CM_update_map_bumper();
	}

	g_bumper_hitted = true;
	ROS_WARN("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, Get_Bumper_Status(), state_now ? "true" : "false", state_last ? "true" : "false");
	if (state_now == true && state_last == true) {
		g_bumper_cnt++;

		if (g_bumper_cnt > 2) {
			ROS_WARN("%s %d: all bumper jam, should quit, jam count: %d", __FUNCTION__, __LINE__, g_bumper_cnt);
			g_bumper_jam = g_fatal_quit_event = true;
		}
	} else {
		g_bumper_cnt = 0;
	}

	CM_CorBack(COR_BACK_20MM);
	if (g_cm_move_type == CM_ROUNDING) {
		if (g_rounding_type == ROUNDING_LEFT) {
			g_rounding_wall_distance = robot::instance()->getLeftWall() > (Wall_Low_Limit) ?
															 robot::instance()->getLeftWall() / 3 : g_rounding_wall_distance + 200;
			for (int i = 0; i < 3; i++) {
				g_rounding_left_wall_buffer[i] = 0;
			}
			check_limit(g_rounding_wall_distance, Wall_Low_Limit, Wall_High_Limit)

			g_rounding_turn_angle = 300;
			g_rounding_move_speed = 10;
			g_rounding_wall_straight_distance = 250;
		} else {
			ROS_WARN("%s %d: move back for left bumper.", __FUNCTION__, __LINE__);
			g_rounding_turn_angle = 1350;
			g_rounding_move_speed = 15;
			g_rounding_wall_straight_distance = 375;
		}
	}

	ROS_WARN("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, Get_Bumper_Status());
	if((Get_Bumper_Status() & LeftBumperTrig) == 0) g_bumper_cnt = 0;
}

void CM_handle_bumper_right(bool state_now, bool state_last)
{
	uint8_t isBumperTriggered = Get_Bumper_Status();

	g_bumper_status_for_rounding = isBumperTriggered;

	Set_Wheel_Speed(0, 0);
	if (g_cm_move_type == CM_LINEARMOVE) {
		CM_update_map_bumper();
	}

	g_bumper_hitted = true;
	ROS_WARN("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, Get_Bumper_Status(), state_now ? "true" : "false", state_last ? "true" : "false");
	if (state_now == true && state_last == true) {
		g_bumper_cnt++;

		if (g_bumper_cnt > 2) {
			ROS_WARN("%s %d: all bumper jam, should quit, jam count: %d", __FUNCTION__, __LINE__, g_bumper_cnt);
			g_bumper_jam = g_fatal_quit_event = true;
		}
	} else {
		g_bumper_cnt = 0;
	}

	CM_CorBack(COR_BACK_20MM);
	if (g_cm_move_type == CM_ROUNDING) {
		if (g_rounding_type == ROUNDING_LEFT) {
			ROS_INFO("%s %d: move back for right bumper.", __FUNCTION__, __LINE__);
			g_rounding_turn_angle = 1350;
			g_rounding_move_speed = 15;
			g_rounding_wall_straight_distance = 375;
		} else {
			g_rounding_wall_distance = robot::instance()->getRightWall() > (Wall_Low_Limit) ?
															 robot::instance()->getRightWall() / 3 : g_rounding_wall_distance + 200;
			for (int i = 0; i < 3; i++) {
				g_rounding_right_wall_buffer[i] = 0;
			}
			check_limit(g_rounding_wall_distance, Wall_Low_Limit, Wall_High_Limit)

			g_rounding_turn_angle = 300;
			g_rounding_move_speed = 10;
			g_rounding_wall_straight_distance = 250;
		}
	}

	ROS_WARN("%s %d: is called, bumper: %d", __FUNCTION__, __LINE__, Get_Bumper_Status());
	if((Get_Bumper_Status() & RightBumperTrig) == 0) g_bumper_cnt = 0;
}

/* OBS */
void CM_handle_obs_front(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
	g_obs_triggered = true;
}

void CM_handle_obs_left(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
	g_obs_triggered = true;
}

void CM_handle_obs_right(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
	g_obs_triggered = true;
}

/* Cliff */
void CM_handle_cliff_all(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
	g_cliff_all_triggered = true;
	g_fatal_quit_event = true;
}

void CM_handle_cliff_front_left(bool state_now, bool state_last)
{
	Set_Wheel_Speed(0, 0);
	CM_update_map();

	g_cliff_triggered = true;
	ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
	if (state_now == true && state_last == true) {
		g_cliff_cnt++;

		if (g_cliff_cnt > 2) {
			ROS_WARN("%s %d: should quit, jam count: %d", __FUNCTION__, __LINE__, g_cliff_cnt);
			g_cliff_jam = true;
			g_fatal_quit_event = true;
		}
	} else {
		g_cliff_cnt = 0;
	}

	CM_CorBack(COR_BACK_20MM);
	if (g_cm_move_type == CM_ROUNDING) {
		if (g_rounding_type == ROUNDING_LEFT) {
			g_rounding_turn_angle = 600;
		} else {
			g_rounding_turn_angle = 1350;
		}
	}
}

void CM_handle_cliff_front_right(bool state_now, bool state_last)
{
	Set_Wheel_Speed(0, 0);
	CM_update_map();

	g_cliff_triggered = true;
	ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
	if (state_now == true && state_last == true) {
		g_cliff_cnt++;

		if (g_cliff_cnt > 2) {
			ROS_WARN("%s %d: should quit, jam count: %d", __FUNCTION__, __LINE__, g_cliff_cnt);
			g_cliff_jam = true;
			g_fatal_quit_event = true;
		}
	} else {
		g_cliff_cnt = 0;
	}

	CM_CorBack(COR_BACK_20MM);
	if (g_cm_move_type == CM_ROUNDING) {
		if (g_rounding_type == ROUNDING_LEFT) {
			g_rounding_turn_angle = 1350;
		} else {
			g_rounding_turn_angle = 600;
		}
	}
}

void CM_handle_cliff_left_right(bool state_now, bool state_last)
{
	Set_Wheel_Speed(0, 0);
	CM_update_map();

	g_cliff_triggered = true;
	ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
	if (state_now == true && state_last == true) {
		g_cliff_cnt++;

		if (g_cliff_cnt > 2) {
			ROS_WARN("%s %d: should quit, jam count: %d", __FUNCTION__, __LINE__, g_cliff_cnt);
			g_cliff_jam = true;
			g_fatal_quit_event = true;
		}
	} else {
		g_cliff_cnt = 0;
	}

	CM_CorBack(COR_BACK_20MM);
	if (g_cm_move_type == CM_ROUNDING) {
		g_rounding_turn_angle = 900;
	}
}

void CM_handle_cliff_front(bool state_now, bool state_last)
{
	Set_Wheel_Speed(0, 0);
	CM_update_map();

	g_cliff_triggered = true;
	ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
	if (state_now == true && state_last == true) {
		g_cliff_cnt++;

		if (g_cliff_cnt > 2) {
			ROS_WARN("%s %d: should quit, jam count: %d", __FUNCTION__, __LINE__, g_cliff_cnt);
			g_cliff_jam = true;
			g_fatal_quit_event = true;
		}
	} else {
		g_cliff_cnt = 0;
	}

	CM_CorBack(COR_BACK_20MM);
	if (g_cm_move_type == CM_ROUNDING) {
		g_rounding_turn_angle = 900;
	}
}

void CM_handle_cliff_left(bool state_now, bool state_last)
{
	Set_Wheel_Speed(0, 0);
	CM_update_map();

	g_cliff_triggered = true;
	ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
	if (state_now == true && state_last == true) {
		g_cliff_cnt++;

		if (g_cliff_cnt > 2) {
			ROS_WARN("%s %d: should quit, jam count: %d", __FUNCTION__, __LINE__, g_cliff_cnt);
			g_cliff_jam = true;
			g_fatal_quit_event = true;
		}
	} else {
		g_cliff_cnt = 0;
	}

	CM_CorBack(COR_BACK_20MM);
	if (g_cm_move_type == CM_ROUNDING) {
		if (g_rounding_type == ROUNDING_LEFT) {
			g_rounding_turn_angle = 300;
		} else {
			g_rounding_turn_angle = 1350;
		}
	}
}

void CM_handle_cliff_right(bool state_now, bool state_last)
{
	Set_Wheel_Speed(0, 0);
	CM_update_map();

	g_cliff_triggered = true;
	ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
	if (state_now == true && state_last == true) {
		g_cliff_cnt++;

		if (g_cliff_cnt > 2) {
			ROS_WARN("%s %d: should quit, jam count: %d", __FUNCTION__, __LINE__, g_cliff_cnt);
			g_cliff_jam = true;
			g_fatal_quit_event = true;
		}
	} else {
		g_cliff_cnt = 0;
	}

	CM_CorBack(COR_BACK_20MM);
	if (g_cm_move_type == CM_ROUNDING) {
		if (g_rounding_type == ROUNDING_LEFT) {
			g_rounding_turn_angle = 1350;
		} else {
			g_rounding_turn_angle = 300;
		}
	}
}

/* RCON */
void CM_handle_rcon_front_left(bool state_now, bool state_last)
{
	int32_t	x, y;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_go_home) {
		ROS_DEBUG("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
		return;
	}

	Set_Wheel_Speed(0, 0);

	CM_count_normalize(Gyro_GetAngle(), 0, CELL_SIZE_2, &x, &y);
	Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
	ROS_INFO("%s %d: is called. marking (%d, %d)", __FUNCTION__, __LINE__,  countToCell(x), countToCell(y));

	g_rcon_triggered = true;
	CM_SetHome(Map_GetXCount(), Map_GetYCount());
	Reset_Rcon_Status();

	if (g_cm_move_type == CM_ROUNDING) {
		g_rounding_turn_angle = 850;
	}
}

void CM_handle_rcon_front_left2(bool state_now, bool state_last)
{
	int32_t	x, y;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_go_home) {
		ROS_DEBUG("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
		return;
	}

	Set_Wheel_Speed(0, 0);

	CM_count_normalize(Gyro_GetAngle(), CELL_SIZE, CELL_SIZE_2, &x, &y);
	Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
	ROS_INFO("%s %d: is called. marking (%d, %d)", __FUNCTION__, __LINE__,  countToCell(x), countToCell(y));
	CM_count_normalize(Gyro_GetAngle(), CELL_SIZE_2, CELL_SIZE, &x, &y);
	Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
	ROS_INFO("%s %d: is called. marking (%d, %d)", __FUNCTION__, __LINE__,  countToCell(x), countToCell(y));

	g_rcon_triggered = true;
	CM_SetHome(Map_GetXCount(), Map_GetYCount());
	Reset_Rcon_Status();
	if (g_cm_move_type == CM_ROUNDING) {
		if (g_rounding_type == ROUNDING_LEFT) {
			g_rounding_turn_angle = 600;
		}
	}
}

void CM_handle_rcon_front_right(bool state_now, bool state_last)
{
	int32_t	x, y;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_go_home) {
		ROS_DEBUG("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
		return;
	}

	Set_Wheel_Speed(0, 0);

	CM_count_normalize(Gyro_GetAngle(), 0, CELL_SIZE_2, &x, &y);
	Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
	ROS_INFO("%s %d: is called. marking (%d, %d)", __FUNCTION__, __LINE__,  countToCell(x), countToCell(y));

	g_rcon_triggered = true;
	CM_SetHome(Map_GetXCount(), Map_GetYCount());
	Reset_Rcon_Status();

	if (g_cm_move_type == CM_ROUNDING) {
		g_rounding_turn_angle = 850;
	}
}

void CM_handle_rcon_front_right2(bool state_now, bool state_last)
{
	int32_t	x, y;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_go_home) {
		ROS_DEBUG("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
		return;
	}

	Set_Wheel_Speed(0, 0);

	CM_count_normalize(Gyro_GetAngle(), -CELL_SIZE, CELL_SIZE_2, &x, &y);
	Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
	ROS_INFO("%s %d: is called. marking (%d, %d)", __FUNCTION__, __LINE__,  countToCell(x), countToCell(y));
	CM_count_normalize(Gyro_GetAngle(), -CELL_SIZE_2, CELL_SIZE, &x, &y);
	Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
	ROS_INFO("%s %d: is called. marking (%d, %d)", __FUNCTION__, __LINE__,  countToCell(x), countToCell(y));

	g_rcon_triggered = true;
	CM_SetHome(Map_GetXCount(), Map_GetYCount());
	Reset_Rcon_Status();
	if (g_cm_move_type == CM_ROUNDING) {
		if (g_rounding_type == ROUNDING_LEFT) {
			g_rounding_turn_angle = 950;
		}
	}
}

void CM_handle_rcon_left(bool state_now, bool state_last)
{
	int32_t	x, y;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_go_home) {
		ROS_DEBUG("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
		return;
	}

	Set_Wheel_Speed(0, 0);

	CM_count_normalize(Gyro_GetAngle(), CELL_SIZE, CELL_SIZE_2, &x, &y);
	Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
	ROS_INFO("%s %d: is called. marking (%d, %d)", __FUNCTION__, __LINE__,  countToCell(x), countToCell(y));

	g_rcon_triggered = true;
	CM_SetHome(Map_GetXCount(), Map_GetYCount());
	Reset_Rcon_Status();

	if (g_cm_move_type == CM_ROUNDING) {
		if (g_rounding_type == ROUNDING_LEFT) {
			g_rounding_turn_angle = 300;
		}
	}
}

void CM_handle_rcon_right(bool state_now, bool state_last)
{
	int32_t	x, y;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_go_home) {
		ROS_DEBUG("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
		return;
	}

	Set_Wheel_Speed(0, 0);

	CM_count_normalize(Gyro_GetAngle(), -CELL_SIZE, CELL_SIZE_2, &x, &y);
	Map_SetCell(MAP, x, y, BLOCKED_BUMPER);
	ROS_INFO("%s %d: is called. marking (%d, %d)", __FUNCTION__, __LINE__,  countToCell(x), countToCell(y));

	g_rcon_triggered = true;
	CM_SetHome(Map_GetXCount(), Map_GetYCount());
	Reset_Rcon_Status();
	if (g_cm_move_type == CM_ROUNDING) {
		if (g_rounding_type == ROUNDING_LEFT) {
			g_rounding_turn_angle = 1100;
		}
	}
}

/* Over Current */
void CM_handle_over_current_brush_left(bool state_now, bool state_last)
{
	static uint8_t stop_cnt = 0;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
	if(!robot::instance()->getLbrushOc()) {
		g_oc_brush_left_cnt = 0;
		if (stop_cnt++ > 250) {
			Set_LeftBrush_PWM(30);
		}
		return;
	}

	stop_cnt = 0;
	if (g_oc_brush_left_cnt++ > 40) {
		g_oc_brush_left_cnt = 0;
		Set_LeftBrush_PWM(0);
		ROS_WARN("%s %d: left brush over current", __FUNCTION__, __LINE__);
	}
}

void CM_handle_over_current_brush_main(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!robot::instance()->getMbrushOc()){
		g_oc_brush_main_cnt = 0;
		return;
	}

	if (g_oc_brush_main_cnt++ > 40) {
		g_oc_brush_main_cnt = 0;
		ROS_WARN("%s %d: main brush over current", __FUNCTION__, __LINE__);

		if (Self_Check(Check_Main_Brush) == 1) {
			g_oc_brush_main = true;
			g_fatal_quit_event = true;
		}
    }
}

void CM_handle_over_current_brush_right(bool state_now, bool state_last)
{
	static uint8_t stop_cnt = 0;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if(!robot::instance()->getRbrushOc()) {
		g_oc_brush_right_cnt = 0;
		if (stop_cnt++ > 250) {
			Set_RightBrush_PWM(30);
		}
		return;
	}

	stop_cnt = 0;
	if (g_oc_brush_right_cnt++ > 40) {
		g_oc_brush_right_cnt = 0;
		Set_RightBrush_PWM(0);
		ROS_WARN("%s %d: reft brush over current", __FUNCTION__, __LINE__);
	}
}

void CM_handle_over_current_wheel_left(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if ((uint32_t) robot::instance()->getLwheelCurrent() < Wheel_Stall_Limit) {
        g_oc_wheel_left_cnt = 0;
		return;
	}

	if (g_oc_wheel_left_cnt++ > 40){
		g_oc_wheel_left_cnt = 0;
		ROS_WARN("%s %d: left wheel over current, %lu mA", __FUNCTION__, __LINE__, (uint32_t) robot::instance()->getLwheelCurrent());

		if (Self_Check(Check_Left_Wheel) == 1) {
			g_oc_wheel_left = true;
			g_fatal_quit_event = true;
		}
	}
}

void CM_handle_over_current_wheel_right(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if ((uint32_t) robot::instance()->getRwheelCurrent() < Wheel_Stall_Limit) {
		g_oc_wheel_right_cnt = 0;
		return;
	}

	if (g_oc_wheel_right_cnt++ > 40){
		g_oc_wheel_right_cnt = 0;
		ROS_WARN("%s %d: right wheel over current, %lu mA", __FUNCTION__, __LINE__, (uint32_t) robot::instance()->getRwheelCurrent());

		if (Self_Check(Check_Right_Wheel) == 1) {
			g_oc_wheel_right = true;
			g_fatal_quit_event = true;
		}
	}
}

void CM_handle_over_current_suction(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!robot::instance()->getVacuumOc()) {
		g_oc_suction_cnt = 0;
		return;
	}

	if (g_oc_suction_cnt++ > 40) {
		g_oc_suction_cnt = 0;
		ROS_WARN("%s %d: vacuum over current", __FUNCTION__, __LINE__);

		if (Self_Check(Check_Vacuum) == 1) {
			g_oc_suction = false;
			g_fatal_quit_event = true;
		}
	}
}

/* Key */
void CM_handle_key_clean(bool state_now, bool state_last)
{
	time_t start_time;
	bool reset_manual_pause = false;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
	Set_Wheel_Speed(0, 0);
	g_key_clean_pressed = true;
	robot::instance()->setManualPause();
	start_time = time(NULL);

	while (Get_Key_Press() & KEY_CLEAN)
	{
		if (time(NULL) - start_time > 3) {
			if (!reset_manual_pause)
			{
				Beep(2, 5, 0, 1);
				reset_manual_pause = true;
			}
			robot::instance()->resetManualPause();
			ROS_WARN("%s %d: Key clean is not released and manual pause has been reset.", __FUNCTION__, __LINE__);
		}
		else
			ROS_WARN("%s %d: Key clean is not released.", __FUNCTION__, __LINE__);
		usleep(20000);
	}
	Reset_Touch();
}

/* Remote */

void CM_handle_remote_plan(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
	Set_Plan_Status(0);
	Beep(Beep_Error_Sounds, 2, 0, 1);
}

void CM_handle_remote_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	g_key_clean_pressed = true;
	robot::instance()->setManualPause();

	Reset_Rcon_Remote();
}

void CM_handle_remote_home(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	g_go_home = true;
	g_remote_home = true;
	Reset_Rcon_Remote();
}

void CM_handle_remote_mode_spot(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	Stop_Brifly();
	Reset_Rcon_Remote();
	auto modeTemp = Get_VacMode();
	//Spot_Mode(CleanSpot);
	Spot_WithCell(CleanSpot,1.0);
	Work_Motor_Configure();
	Set_VacMode(modeTemp,false);
}

void CM_handle_remote_suction(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!g_go_home)
		Switch_VacMode(true);
	Reset_Rcon_Remote();
}

/* Battery */
void CM_handle_battery_home(bool state_now, bool state_last)
{
	if (! g_go_home) {
		g_go_home = true;
		ROS_WARN("%s %d: low battery, battery < %dmv is detected.", __FUNCTION__, __LINE__,
						 robot::instance()->getBatteryVoltage());
		g_battery_home = true;

		if (Get_VacMode() == Vac_Max) {
			Switch_VacMode(false);
		}
#if CONTINUE_CLEANING_AFTER_CHARGE
		CM_SetContinuePoint(Map_GetXCount(), Map_GetYCount());
		robot::instance()->setLowBatPause();
#endif
	}
}

void CM_handle_battery_low(bool state_now, bool state_last)
{
	uint8_t		v_pwr, s_pwr, m_pwr;
	uint16_t	t_vol;

	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_battery_low_cnt++ > 50) {
		ROS_WARN("%s %d: low battery, battery < %dmv is detected.", __FUNCTION__, __LINE__,
						 robot::instance()->getBatteryVoltage());
		t_vol = GetBatteryVoltage();

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
		Set_BLDC_Speed(v_pwr);
		Set_SideBrush_PWM(s_pwr, s_pwr);
		Set_MainBrush_PWM(m_pwr);

		g_fatal_quit_event = true;
		g_battery_low = true;
	}
}

