#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

#include <list>

#include <ros/ros.h>

#include "config.h"
#include "map.h"
#include "curve_move.h"
#include "shortest_path.h"
#include "movement.h"
#include "core_move.h"
#include "gyro.h"

using namespace std;

#define	PATH_PLANNER_MAX_SPEED		(30)
#define WHEEL_BASE_HALF			(((double)(WHEEL_BASE)) / 2)
#define CELL_COUNT_MUL_TO_CELL_SIZE	(((double) (CELL_COUNT_MUL)) / CELL_SIZE)

MapTouringType CurveMove_MoveToPoint()
{
	int	i;
	double	diff_1, diff_2, radius;
	int16_t angle_start, angle_diff;
	int32_t course;
	uint8_t speed_left, speed_right, speed, isBumperTriggered;

	Point16_t	points[3];
	Point32_t	target;
	ActionType	action = ACTION_NONE;
	MapTouringType	retval = MT_None;

	list<Point16_t> *path_points = path_get_path_points();

	speed_left = speed_right = PATH_PLANNER_MAX_SPEED;

	i = 0;
	for (list<Point16_t>::iterator it = path_points->begin(); i < 3 && it != path_points->end(); ++it, i++) {
		points[i] = *it;
	}
	path_reset_path_points();

	ROS_WARN("%s %d: points[0](%d, %d)\tpoints[1](%d, %d)\tpoints[2](%d, %d)", __FUNCTION__, __LINE__, points[0].X, points[0].Y, points[1].X, points[1].Y, points[2].X, points[2].Y);

	diff_1 = (abs(cellToCount(points[0].X - points[1].X)) + abs(cellToCount(points[0].Y - points[1].Y))) / 2;
	diff_2 = (abs(cellToCount(points[2].X - points[1].X)) + abs(cellToCount(points[2].Y - points[1].Y))) / 2;
	if (diff_1 < 3 * CELL_COUNT_MUL || diff_2 < 3 * CELL_COUNT_MUL) {
		ROS_WARN("%s %d: diff_1: %f\tdiff_2: %f (%d)\n", __FUNCTION__, __LINE__, diff_1, diff_2, 3 * CELL_COUNT_MUL);
		return MT_CurveMove;
	}

	radius = diff_1 > diff_2 ? diff_2 : diff_1;

	target.X = cellToCount(points[1].X);
	target.Y = cellToCount(points[1].Y);
	if (points[0].X == points[1].X) {
		target.Y = target.Y + (points[0].Y > points[1].Y ? radius : -radius);
	} else {
		target.X = target.X + (points[0].X > points[1].X ? radius : -radius);
	}

	course = course2dest(Map_GetXCount(), Map_GetYCount(), target.X, target.Y);
        CM_HeadToCourse(ROTATE_TOP_SPEED, course);

	ROS_WARN("%s %d: target: (%d, %d)\tradius: %f\n", __FUNCTION__, __LINE__, target.X, target.Y, radius);
	if (CM_MoveToPoint(target, PATH_PLANNER_MAX_SPEED, false, true) != MT_None) {
		ROS_WARN("%s %d\n", __FUNCTION__, __LINE__);
		return MT_None;
	}
	ROS_WARN("%s %d: current position: (%d, %d)\tradius: %f", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos(), radius);

	speed = PATH_PLANNER_MAX_SPEED * (((double)radius) / CELL_COUNT_MUL_TO_CELL_SIZE - WHEEL_BASE_HALF) / (((double)radius) / CELL_COUNT_MUL_TO_CELL_SIZE + WHEEL_BASE_HALF);
	if (points[1].X == points[2].X) {
		if (points[0].X > points[1].X) {
			points[1].Y < points[2].Y ? speed_right = speed : speed_left = speed; 
		} else {
			points[1].Y < points[2].Y ? speed_left = speed :speed_right = speed;
		}
	} else {
		if (points[0].Y > points[1].Y) {
			points[1].X > points[2].X ? speed_right = speed : speed_left = speed;
		} else {
			points[1].X > points[2].X ? speed_left = speed : speed_right = speed;
		}
	}
	ROS_WARN("%s %d: speed(%d, %d)\n", __FUNCTION__, __LINE__, speed_left, speed_right);
	if ( speed_left < 0 || speed_left > PATH_PLANNER_MAX_SPEED || speed_right < 0 || speed_right > PATH_PLANNER_MAX_SPEED) {
		ROS_WARN("%s %d: left/right speed error (%d, %d)", speed_left, speed_right);
		return MT_CurveMove;
	}

	angle_start = Gyro_GetAngle(0);
	Move_Forward(speed_left, speed_right);

	ROS_WARN("%s %d: anlge_diff: %d(%d, %d)\n", __FUNCTION__, __LINE__, angle_diff, Gyro_GetAngle(0), angle_start);
	while (1) {
		CM_update_position(Gyro_GetAngle(0), Gyro_GetAngle(1));

		angle_diff = Gyro_GetAngle(0) - angle_start;
		if (angle_diff >= 1800) {
			angle_diff -= 3600;
		} else if (angle_diff <= -1800) {
			angle_diff += 3600;
		}
		if (abs(angle_diff) > 900) {
			break;
		}

		/* Check bumper & cliff event.*/
		if (Get_FrontOBS() > Get_FrontOBST_Value()) {
			Stop_Brifly();
			CM_update_map(action, Get_Bumper_Status());
			retval = MT_OBS;
			ROS_WARN("%s %d", __FUNCTION__, __LINE__);
			break;
		}

		isBumperTriggered = Get_Bumper_Status();
		if (isBumperTriggered) {
			Stop_Brifly();
			CM_update_map_bumper(action, isBumperTriggered);

			ROS_WARN("%s %d: calling moving back", __FUNCTION__, __LINE__);
			CM_CorBack(COR_BACK_20MM);

#ifdef BUMPER_ERROR
			if (Get_Bumper_Status()) {
				ROS_WARN("%s %d: calling moving back", __FUNCTION__, __LINE__);
				CM_CorBack(COR_BACK_20MM);
				if (Get_Bumper_Status()) {
					CM_CorBack(COR_BACK_20MM);
					if (Get_Bumper_Status()) {
						ROS_WARN("%s %d: calling moving back", __FUNCTION__, __LINE__);
						CM_CorBack(COR_BACK_20MM);
						Set_Error_Code(Error_Code_Bumper);
						Stop_Brifly();
						retval = MT_Key_Clean;
						ROS_WARN("%s %d", __FUNCTION__, __LINE__);
						break;
					}
				}
			}
#endif

			Stop_Brifly();
			CM_update_map(action, isBumperTriggered);
			retval = MT_Bumper;
			ROS_WARN("%s %d", __FUNCTION__, __LINE__);
			break;
		}
		usleep(10000);
	}
	ROS_WARN("%s %d: anlge_diff: %d(%d, %d)", __FUNCTION__, __LINE__, angle_diff, Gyro_GetAngle(0), angle_start);

	ROS_WARN("%s %d: current position: (%d, %d) (%d, %d)\tradius: %f", __FUNCTION__, __LINE__, Map_GetXPos(), Map_GetYPos(), Map_GetXCount(), Map_GetYCount(), radius);
	target.X = cellToCount(points[2].X);
	target.Y = cellToCount(points[2].Y);
	
	if (retval == MT_None && CM_MoveToPoint(target, PATH_PLANNER_MAX_SPEED, true, false) != MT_None) {
		ROS_WARN("%s %d", __FUNCTION__, __LINE__);
		retval = MT_None;
	}

	Stop_Brifly();
	return retval;
}
