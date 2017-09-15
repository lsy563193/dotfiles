#include "space_exploration.h"
#include "map.h"
#include "gyro.h"
#include "core_move.h"
#include <ros/ros.h>
#include "motion_manage.h"
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
			cm_world_to_point(gyro_get_angle(), CELL_SIZE * dy, CELL_SIZE * dx, &x, &y);
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
	int16_t angle, laser_angle;
	const int RADIUS_CELL = 10;//the radius of the robot can detect
	angle = gyro_get_angle();
	for (int16_t angle_i = 0; angle_i <= 359; angle_i += 1) {
		for (int dy = 0; dy < RADIUS_CELL; ++dy) {
			cm_world_to_point(angle + angle_i * 10, CELL_SIZE * dy, CELL_SIZE * 0, &x, &y);
			auto status = map_get_cell(MAP, count_to_cell(x), count_to_cell(y));
			if (status > CLEANED && status < BLOCKED_BOUNDARY) {
				//ROS_ERROR("%s,%d: (%d,%d)", __FUNCTION__, __LINE__, count_to_cell(x), count_to_cell(y));
				break;
			}
			map_set_cell(MAP, x, y, CLEANED);
		}
	}
}
