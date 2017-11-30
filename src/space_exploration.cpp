#include "space_exploration.h"
#include "map.h"
#include "gyro.h"
#include "key.h"
#include "core_move.h"
#include <ros/ros.h>
#include <pp.h>
#include "motion_manage.h"
#include "movement.h"
#include "event_manager.h"
#include "path_planning.h"
#include "speaker.h"
#include "clean_mode.h"
#if 0
void map_set_cells(int8_t count, int16_t cell_x, int16_t cell_y, CellState state)
{
	int8_t i, j;

	for ( i = -(count / 2); i <= count / 2; i++ ) {
		for ( j = -(count / 2); j <= count / 2; j++ ) {
			map_set_cell(MAP, cell_to_count(cell_x + i), cell_to_count(cell_y + j), state);
		}
	}
}

bool map_mark_robot(MAP)
{
	int32_t x, y;
	bool ret = false;
	for (auto dy = -ROBOT_SIZE_1_2; dy <= ROBOT_SIZE_1_2; ++dy)
	{
		for (auto dx = -ROBOT_SIZE_1_2; dx <= ROBOT_SIZE_1_2; ++dx)
		{
			cm_world_to_point(robot::instance()->getPoseAngle(), CELL_SIZE * dy, CELL_SIZE * dx, &x, &y);
			auto status = map_get_cell(MAP, count_to_cell(x), count_to_cell(y));
			if (status > CLEANED && status < BLOCKED_BOUNDARY){
				ROS_ERROR("%s,%d: (%d,%d)", __FUNCTION__, __LINE__, count_to_cell(x), count_to_cell(y));
				ret = true;
			}
			map_set_cell(MAP, x, y, CLEANED);
		}
	}
	return ret;
}
#endif

void explore_update_map(void)
{
	int32_t x, y;
	bool ret = false;
	int16_t angle, lidar_angle;
	const int RADIUS_CELL = 10;//the radius of the robot can detect
	angle = robot::instance()->getPoseAngle();
	for (int16_t angle_i = 0; angle_i <= 359; angle_i += 1) {
		for (int dy = 0; dy < RADIUS_CELL; ++dy) {
			Point32_t point = {cost_map.getXCount(), cost_map.getYCell(), static_cast<int16_t>(angle + angle_i * 10)};
			cost_map.robot_to_point(point, CELL_SIZE * dy, CELL_SIZE * 0, &x, &y);
			auto status = cost_map.get_cell(MAP, cost_map.count_to_cell(x), cost_map.count_to_cell(y));
			if (status > CLEANED && status < BLOCKED_BOUNDARY) {
				//ROS_ERROR("%s,%d: (%d,%d)", __FUNCTION__, __LINE__, count_to_cell(x), count_to_cell(y));
				break;
			}
			cost_map.setCell(MAP, x, y, CLEANED);
		}
	}
}

void turn_into_exploration(bool is_reset_map)
{
	led.set_mode(LED_STEADY, LED_ORANGE);

	// Initialize motors and costmap.
	// Push the start point into the home point list
	ROS_INFO("map_init-----------------------------");
	if (is_reset_map)
		cost_map.reset(MAP);
	fw_map.reset(MAP);
	ros_map.reset(MAP);
	path_planning_initialize();
	cs_work_motor();
	cm_reset_go_home();
	robot_timer.initWorkTimer();


	g_have_seen_charger = false;
	g_start_point_seen_charger = false;

	g_homes.resize(1,g_zero_home);
	g_home_gen_rosmap = true;
	g_home_way_list.clear();

	c_rcon.resetStatus();
	key.resetTriggerStatus();
	// Can't register until the status has been checked. because if register too early, the handler may affect the pause status, so it will play the wrong wav.
	speaker.play(SPEAKER_EXPLORATION_START);

	ROS_INFO("\033[47;35m" "%s,%d,enable tilt detect" "\033[0m",__FUNCTION__,__LINE__);


	cm_set(Clean_Mode_Exploration);
	cost_map.ros_convert(MAP, false, false, true);
	explore_update_map();
}
