#include <stdint.h>
#include <stdio.h>

#include <ros/ros.h>

#include "debug.h"
#include "path_planning.h"

char outString[256];

extern PositionType g_positions[];

#if defined(DEBUG_MAP) || defined(DEBUG_SM_MAP)

/*
 * Function to print the robot cleaning map.
 *
 * @param id    MAP id, 0 for cleaning map, 1 for shortest path map.
 * @param endx  X coordinate of target
 * @param endy  Y coordinate of target
 *
 * @return
 */
void debug_map(uint8_t id, int16_t endx, int16_t endy)
{
#if ENABLE_DEBUG
	int16_t		i, j, x_min, x_max, y_min, y_max, index;
	CellState	cs;

	path_get_range(&x_min, &x_max, &y_min, &y_max);

	ROS_INFO("Map: %s", id == MAP ? "MAP" : "SPMAP");
	index = 0;
	outString[index++] = '\t';
	for (j = y_min; j <= y_max; j++) {
		if (abs(j) % 10 == 0) {
			outString[index++] = (j < 0 ? '-' : ' ');
			outString[index++] = (abs(j) >= 100 ? abs(j) / 100 + 48 : ' ');
			outString[index++] = 48 + (abs(j) >= 10 ? ((abs(j) % 100) / 10) : 0);
			j += 3;
		} else {
			outString[index++] = ' ';
		}
	}
	outString[index++] = 0;
	ROS_INFO("%s", outString);

	index = 0;
	outString[index++] = '\t';
	for (j = y_min; j <= y_max; j++) {
		outString[index++] = abs(j) % 10 + 48;
	}
	outString[index++] = 0;
	ROS_INFO("%s\n", outString);

	for (i = x_min; i <= x_max; i++) {
		index = 0;

		outString[index++] = (i < 0 ? '-' : ' ');
		outString[index++] = 48 + (abs(i) >= 100 ? abs(i) / 100 : 0);
		outString[index++] = 48 + (abs(i) >= 10 ? ((abs(i) % 100) / 10) : 0);
		outString[index++] = abs(i) % 10 + 48;
		outString[index++] = '\t';

		for (j = y_min; j <= y_max; j++) {
			cs = Map_GetCell(id, i, j);
			if (i == g_positions[0].x && j == g_positions[0].y) {
				outString[index++] = 'x';
			} else if (i == endx && j == endy) {
				outString[index++] = 'e';
			} else {
				outString[index++] = cs + 48;
			}
		}
		ROS_INFO("%s", outString);
	}
	ROS_INFO("\n");
#endif
}
#endif
