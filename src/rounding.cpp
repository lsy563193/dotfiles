/**
******************************************************************************
* @file    AI Cleaning Robot
* @author  ILife Team Dxsong
* @version V1.0
* @date    17-Nov-2011
* @brief   Move near the wall on the left in a certain distance
******************************************************************************
* <h2><center>&copy; COPYRIGHT 2011 ILife CO.LTD</center></h2>
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <unistd.h>

#include "movement.h"

#include "robot.hpp"
#include "mathematics.h"
#include "path_planning.h"
#include "map.h"
#include "gyro.h"
#include "core_move.h"

#include "rounding.h"

#ifdef TURN_SPEED
#undef TURN_SPEED
#endif

#define TURN_SPEED	17

uint8_t	should_mark = 0;

RoundingType	rounding_type;

void update_position(uint16_t heading_0, int16_t heading_1) {
	float   pos_x, pos_y;
	int8_t	c, d, e;
	int16_t	x, y;
	uint16_t	path_heading;
	int32_t	i, j;

	if (heading_0 > heading_1 && heading_0 - heading_1 > 1800) {
		path_heading = (uint16_t)((heading_0 + heading_1 + 3600) >> 1) % 3600;
	} else if (heading_1 > heading_0 && heading_1 - heading_0 > 1800) {
		path_heading = (uint16_t)((heading_0 + heading_1 + 3600) >> 1) % 3600;
	} else {
		path_heading = (uint16_t)(heading_0 + heading_1) >> 1;
	}

	x = Map_GetXPos();
	y = Map_GetYPos();

	//Map_MoveTo(dd * cos(deg2rad(path_heading, 10)), dd * sin(deg2rad(path_heading, 10)));
	pos_x = robot::instance()->robot_get_position_x() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	pos_y = robot::instance()->robot_get_position_y() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	Map_SetPosition(pos_x, pos_y);

#if (ROBOT_SIZE == 5 || ROBOT_SIZE == 3)

	if (x != Map_GetXPos() || y != Map_GetYPos()) {
		for (c = 1; c >= -1; --c) {
			for (d = 1; d >= -1; --d) {
				i = Map_GetRelativeX(path_heading, CELL_SIZE * c, CELL_SIZE * d);
				j = Map_GetRelativeY(path_heading, CELL_SIZE * c, CELL_SIZE * d);
				e = Map_GetCell(MAP, countToCell(i), countToCell(j));

				if (e == BLOCKED_OBS || e == BLOCKED_BUMPER || e == BLOCKED_BOUNDARY ) {
					Map_SetCell(MAP, i, j, CLEANED);
				}
			}
		}
	}

	Map_SetCell(MAP, Map_GetRelativeX(heading_0, -CELL_SIZE, CELL_SIZE), Map_GetRelativeY(heading_0, -CELL_SIZE, CELL_SIZE), CLEANED);
	Map_SetCell(MAP, Map_GetRelativeX(heading_0, 0, CELL_SIZE), Map_GetRelativeY(heading_0, 0, CELL_SIZE), CLEANED);
	Map_SetCell(MAP, Map_GetRelativeX(heading_0, CELL_SIZE, CELL_SIZE), Map_GetRelativeY(heading_0, CELL_SIZE, CELL_SIZE), CLEANED);

	if (should_mark == 1) {
		if (rounding_type == ROUNDING_LEFT || rounding_type == ROUNDING_RIGHT) {
			i = Map_GetRelativeX(heading_0, rounding_type == ROUNDING_LEFT ? CELL_SIZE_2 : -CELL_SIZE_2, 0);
			j = Map_GetRelativeY(heading_0, rounding_type == ROUNDING_LEFT ? CELL_SIZE_2 : -CELL_SIZE_2, 0);
			if (Map_GetCell(MAP, countToCell(i), countToCell(j)) != BLOCKED_BOUNDARY) {
				Map_SetCell(MAP, i, j, BLOCKED_OBS);
			}
		}
	}

#else

	i = Map_GetRelativeX(path_heading, 0, 0);
	j = Map_GetRelativeY(path_heading, 0, 0);
	Map_SetCell(MAP, Map_GetRelativeX(heading_0, 0, CELL_SIZE), Map_GetRelativeY(heading_0, 0, CELL_SIZE), CLEANED);

	if (should_mark == 1) {
		//if (rounding_type == ROUNDING_LEFT && Get_Wall_ADC(0) > 200) {
		if (rounding_type == ROUNDING_LEFT) {
			Map_SetCell(MAP, Map_GetRelativeX(heading_0, CELL_SIZE, 0), Map_GetRelativeY(heading_0, CELL_SIZE, 0), BLOCKED_OBS);
			//Map_SetCell(MAP, Map_GetRelativeX(heading_0, CELL_SIZE, CELL_SIZE), Map_GetRelativeY(heading_0, CELL_SIZE, CELL_SIZE), BLOCKED_OBS);
		} else if (rounding_type == ROUNDING_RIGHT) {
			Map_SetCell(MAP, Map_GetRelativeX(heading_0, -CELL_SIZE, 0), Map_GetRelativeY(heading_0, -CELL_SIZE, 0), BLOCKED_OBS);
			//Map_SetCell(MAP, Map_GetRelativeX(heading_0, -CELL_SIZE, -CELL_SIZE), Map_GetRelativeY(heading_0, -CELL_SIZE, -CELL_SIZE), BLOCKED_OBS);
		}
	}
#endif
}

int8_t rounding_update()
{
	update_position(Gyro_GetAngle(0), Gyro_GetAngle(1));

	return 0;
}

void rounding_turn(uint8_t dir, uint16_t speed, uint16_t angle)
{
	uint16_t Counter_Watcher = 0;
	uint32_t step;

	Stop_Brifly();

	if (dir == 0) {
		Turn_Left(speed, angle);
	} else {
		Turn_Right(speed, angle);
	}

	if (Remote_Key(Remote_Clean | Remote_Home))
	{
		// This remote command should change the behavior of robot.
		Stop_Brifly();
	}
	if (Remote_Key(Remote_Max) && !lowBattery)
	{
		Switch_VacMode();
		Reset_Rcon_Remote();
	}
	if (Touch_Detect())
	{
		ROS_INFO("%s %d: Touch detect.", __FUNCTION__, __LINE__);
		// If touch detected, should change the behavior of robot.
		Stop_Brifly();
		Set_Touch();
	}

	rounding_update();

	should_mark = 1;
	ROS_INFO("%s %d: %d %d %d", __FUNCTION__, __LINE__, dir, speed, angle);
}

void rounding_move_back(uint16_t dist)
{
	float pos_x, pos_y, distance;
	uint16_t Counter_Watcher = 0;

	Stop_Brifly();
	Set_Dir_Backward();
	Set_Wheel_Speed(15, 15);
	Counter_Watcher = 0;

	should_mark = 0;

	pos_x = robot::instance()->robot_get_position_x();
	pos_y = robot::instance()->robot_get_position_y();
	while (1) {
		distance = sqrtf(powf(pos_x - robot::instance()->robot_get_position_x(), 2) + powf(pos_y - robot::instance()->robot_get_position_y(), 2));
		if (fabsf(distance) > 0.02f) {
			break;
		}

		rounding_update();
		usleep(10000);
		Counter_Watcher++;
		if (Counter_Watcher > 3000) {
			if(Is_Encoder_Fail()) {
				Set_Error_Code(Error_Code_Encoder);
				Set_Touch();
			}
			break;
		}
		if (Touch_Detect()) {
			ROS_INFO("%s %d: Touch detect.", __FUNCTION__, __LINE__);
			Set_Touch();
			break;
		}
		if ((Check_Motor_Current() == Check_Left_Wheel) || (Check_Motor_Current() == Check_Right_Wheel)) {
			break;
		}
	}
	rounding_update();
	should_mark = 1;
}

uint8_t rounding_boundary_check()
{
	uint8_t boundary_reach = 0;
	int16_t j;
	int32_t x, y;

	for (j = -1; boundary_reach == 0 && j <= 1; j++) {
#if (ROBOT_SIZE == 5)

		x = Map_GetRelativeX(Gyro_GetAngle(0), j * CELL_SIZE, CELL_SIZE_3);
		y = Map_GetRelativeY(Gyro_GetAngle(0), j * CELL_SIZE, CELL_SIZE_3);

#else

		x = Map_GetRelativeX(Gyro_GetAngle(0), j * CELL_SIZE, CELL_SIZE_3);
		y = Map_GetRelativeY(Gyro_GetAngle(0), j * CELL_SIZE, CELL_SIZE_3);

#endif

		if (Map_GetCell(MAP, countToCell(x), countToCell(y)) == BLOCKED_BOUNDARY) {
			boundary_reach = 1;
			rounding_update();
			Set_Wheel_Speed(0, 0);
			usleep(10000);

			rounding_update();
			rounding_move_back(350);
			rounding_update();
			if (Touch_Detect())
			{
				ROS_INFO("%s %d: Touch detect.", __FUNCTION__, __LINE__);
				return boundary_reach;
			}

			rounding_turn((rounding_type == ROUNDING_LEFT ? 1 : 0), TURN_SPEED, 600);
			if (Touch_Detect())
			{
				ROS_INFO("%s %d: Touch detect.", __FUNCTION__, __LINE__);
				return boundary_reach;
			}

			rounding_update();
		}
	}
	return boundary_reach;
}

uint8_t rounding(RoundingType type, Point32_t target, uint8_t Origin_Bumper_Status)
{
	uint8_t		Jam = 0, Temp_Counter = 0, RandomRemoteWall_Flag = 0, Mobility_Temp_Error = 0, Temp_Bumper_Status;
	int16_t		Left_Wall_Buffer[3] = { 0 }, Right_Wall_Buffer[3] = { 0 };
	int32_t		y_start, R = 0, Proportion = 0, Delta = 0, Previous = 0;
	uint32_t	WorkTime_Buffer = 0, Temp_Status = 0;

	volatile uint8_t	Motor_Check_Code = 0;
	volatile int32_t	L_B_Counter = 0, Wall_Distance = 400, Wall_Straight_Distance, Left_Wall_Speed = 0, Right_Wall_Speed = 0;

	rounding_type = type;
	y_start = Map_GetYCount();
	ROS_INFO("%s %d: Y: %d\tY abs: %d\ttarget Y: %d\ttarget Y abs: %d", __FUNCTION__, __LINE__, Map_GetYCount(), abs(Map_GetYCount()), target.Y, abs(target.Y));

	if ((Origin_Bumper_Status & LeftBumperTrig) && !(Origin_Bumper_Status & RightBumperTrig))
	{
		// Debug
		if (type == ROUNDING_LEFT)
		{
			ROS_INFO("%s %d: Left bumper ROUNDING_LEFT and turn 45 degrees.", __FUNCTION__, __LINE__);
		}
		else
		{
			ROS_INFO("%s %d: Left bumper ROUNDING_RIGHT and turn 135 degrees.", __FUNCTION__, __LINE__);
		}
		//rounding_turn((type == ROUNDING_LEFT ? 1 : 0), TURN_SPEED, 900);
		rounding_turn((type == ROUNDING_LEFT ? 1 : 0), TURN_SPEED, (type == ROUNDING_LEFT ? 450 : 1350));
	}
	else if (!(Origin_Bumper_Status & LeftBumperTrig) && (Origin_Bumper_Status & RightBumperTrig))
	{
		// Debug
		if (type == ROUNDING_LEFT)
		{
			ROS_INFO("%s %d: Right bumper ROUNDING_LEFT and turn 135 degrees.", __FUNCTION__, __LINE__);
		}
		else
		{
			ROS_INFO("%s %d: Right bumper ROUNDING_RIGHT and turn 45 degrees.", __FUNCTION__, __LINE__);
		}
		rounding_turn((type == ROUNDING_LEFT ? 1 : 0), TURN_SPEED, (type == ROUNDING_LEFT ? 1350 : 450));
	}
	else
	{
		// If bumper not hit or it just hit the front (Both left and right bumper triggered, robot should turn 90 degrees.
		rounding_turn((type == ROUNDING_LEFT ? 1 : 0), TURN_SPEED, 900);
		// Debug
		if (type == ROUNDING_LEFT)
		{
			ROS_INFO("%s %d: Same bumper ROUNDING_LEFT and turn 90 degrees.", __FUNCTION__, __LINE__);
		}
		else
		{
			ROS_INFO("%s %d: Same bumper ROUNDING_RIGHT and turn 90 degrees.", __FUNCTION__, __LINE__);
		}
	}
	if (Touch_Detect())
	{
		Stop_Brifly();
		Set_Touch();
		return 0;
	}

	Wall_Straight_Distance = 300;

	should_mark = 1;
	while (1) {

#ifdef OBS_DYNAMIC
		OBS_Dynamic_Base(100);
#endif

		rounding_update();

		if (Remote_Key(Remote_Clean | Remote_Home))
		{
			// This remote command should change the behavior of robot.
			Stop_Brifly();
			return 0;
		}
		if (Touch_Detect())
		{
			Stop_Brifly();
			Set_Touch();
			return 0;
		}
		if (Remote_Key(Remote_Max) && !lowBattery)
		{
			Switch_VacMode();
			Reset_Rcon_Remote();
		}
		if ((y_start > target.Y && Map_GetYCount() < target.Y) || (y_start < target.Y && Map_GetYCount() > target.Y)) {
			// Robot has reach the target.
			ROS_INFO("%s %d: Y: %d\tY abs: %d\ttarget Y: %d\ttarget Y abs: %d", __FUNCTION__, __LINE__, Map_GetYCount(), abs(Map_GetYCount()), target.Y, abs(target.Y));
			Stop_Brifly();
			return 0;
		}

		/* Tolerance of distance allow to move back when roundinging the obstcal. */
		if ((target.Y > y_start && (y_start - Map_GetYCount()) > 120) || (target.Y < y_start && (Map_GetYCount() - y_start) > 120)) {
			// Robot has round to the opposite direcition.
			ROS_INFO("%s %d: Y: %d position: (%d, %d)", __FUNCTION__, __LINE__, target.Y, Map_GetXCount(), Map_GetYCount());

			Map_SetCell(MAP, Map_GetRelativeX(Gyro_GetAngle(0), CELL_SIZE_3, 0), Map_GetRelativeY(Gyro_GetAngle(0), CELL_SIZE_3, 0), CLEANED);

			Stop_Brifly();
			return 0;
		}

		Temp_Bumper_Status = Get_Bumper_Status();
		// Use the left wall sensor.
		if (type == ROUNDING_LEFT) {
			if (!(Temp_Bumper_Status & LeftBumperTrig) && (Temp_Bumper_Status & RightBumperTrig)) {
				// Only right bumper is triggered.
				//Stop_Brifly();
				ROS_INFO("%s %d: move back for right bumper.", __FUNCTION__, __LINE__);
				rounding_move_back(100);
				if (Touch_Detect())
				{
					ROS_INFO("%s %d: Touch detect.", __FUNCTION__, __LINE__);
					break;
				}
				// Turn right for 135 degrees.
				rounding_turn(1, TURN_SPEED, 1350);
				if (Touch_Detect())
				{
					ROS_INFO("%s %d: Touch detect.", __FUNCTION__, __LINE__);
					break;
				}
				Move_Forward(15, 15);
				Wall_Straight_Distance = 375;
			}

			if (Temp_Bumper_Status & LeftBumperTrig) {
				Set_Wheel_Speed(0, 0);
				usleep(300000);
				if (robot::instance()->robot_get_left_wall() > (Wall_Low_Limit)) {
					Wall_Distance = robot::instance()->robot_get_left_wall() / 3;
				} else {
					Wall_Distance += 200;
				}

				if (Wall_Distance < Wall_Low_Limit) {
					Wall_Distance = Wall_Low_Limit;
				}
				if (Wall_Distance > Wall_High_Limit) {
					Wall_Distance = Wall_High_Limit;
				}

				if (Temp_Bumper_Status & RightBumperTrig) {
					// Both left and right bumper are triggered.
					rounding_move_back(350);
					if (Touch_Detect())
					{
						ROS_INFO("%s %d: Touch detect.", __FUNCTION__, __LINE__);
						break;
					}
					// Turn right for 90 degrees.
					rounding_turn(1, TURN_SPEED, 900);
					if (Touch_Detect())
					{
						ROS_INFO("%s %d: Touch detect.", __FUNCTION__, __LINE__);
						break;
					}
					Wall_Straight_Distance = 150;
				} else {
					// Only left bumper is triggered.
					rounding_move_back(350);
					if (Touch_Detect())
					{
						ROS_INFO("%s %d: Touch detect.", __FUNCTION__, __LINE__);
						break;
					}
					// Turn right for 30 degrees.
					rounding_turn(1, TURN_SPEED, 300);
					if (Touch_Detect())
					{
						ROS_INFO("%s %d: Touch detect.", __FUNCTION__, __LINE__);
						break;
					}
					Wall_Straight_Distance = 250;
				}
				Move_Forward(10, 10);

				for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++) {
					Left_Wall_Buffer[Temp_Counter] = 0;
				}
			}

			if (Wall_Distance >= 200) {
				Left_Wall_Buffer[2] = Left_Wall_Buffer[1];
				Left_Wall_Buffer[1] = Left_Wall_Buffer[0];
				Left_Wall_Buffer[0] = robot::instance()->robot_get_left_wall();
				if (Left_Wall_Buffer[0] < 100) {
					if ((Left_Wall_Buffer[1] - Left_Wall_Buffer[0]) > (Wall_Distance / 25)) {
						if ((Left_Wall_Buffer[2] - Left_Wall_Buffer[1]) > (Wall_Distance / 25)) {
							//if (Get_WallAccelerate()>300) {
								//if ((Get_RightWheel_Speed()-Get_LeftWheel_Speed()) >= -3) {
									// Away from the wall.
									Move_Forward(18, 16);
									usleep(100000);
									Wall_Straight_Distance = 300;
								//}
							//}
						}
					}
				}
			}

			/*------------------------------------------------------Wheel Speed adjustment-----------------------*/
			if (Get_FrontOBS() < Get_FrontOBST_Value()) {
				Proportion = robot::instance()->robot_get_left_wall();

				Proportion = Proportion * 100 / Wall_Distance;

				Proportion -= 100;

				Delta = Proportion - Previous;

				if (Wall_Distance > 300) {	//over left
					Left_Wall_Speed = 25 + Proportion / 12 + Delta / 5;
					Right_Wall_Speed = 25 - Proportion / 10 - Delta / 5;
					if (Right_Wall_Speed > 33) {
						Left_Wall_Speed = 5; //9;
						Right_Wall_Speed = 33;
					}
				} else {
					Left_Wall_Speed = 15 + Proportion / 22 + Delta / 10;
					Right_Wall_Speed = 15 - Proportion / 18 - Delta / 10;

					if (Right_Wall_Speed > 18) {
						Left_Wall_Speed = 5;
						Right_Wall_Speed = 18;
					}

					if (Left_Wall_Speed > 20) {
						Left_Wall_Speed = 20;
					}
					if (Left_Wall_Speed < 4) {
						Left_Wall_Speed = 4;
					}
					if (Right_Wall_Speed < 4) {
						Right_Wall_Speed = 4;
					}
					if ((Left_Wall_Speed - Right_Wall_Speed) > 5) {
						Left_Wall_Speed = Right_Wall_Speed+5;
					}
				}

				/*slow move if left obs near a wall*/
				if (Get_LeftOBS() > Get_LeftOBST_Value()) {
					if (Wall_Distance < Wall_High_Limit) {
						Wall_Distance++;
					}
				}
				if (Is_WallOBS_Near()){
					Left_Wall_Speed = Left_Wall_Speed / 2;
					Right_Wall_Speed = Right_Wall_Speed / 2;
				}

				Previous = Proportion;

				if (Left_Wall_Speed < 0) {
					Left_Wall_Speed = 0;
				}
				if (Left_Wall_Speed > 40) {
					Left_Wall_Speed = 40;
				}
				if (Right_Wall_Speed < 0) {
					Right_Wall_Speed = 0;
				}

				Move_Forward(Left_Wall_Speed, Right_Wall_Speed);
			} else {
				// 12500 steps means around 2 meters.
				if (Get_LeftWheel_Step() < 12500) {
					if (Get_FrontOBS() > Get_FrontOBST_Value()) {
						// Turn right for 80 degrees.
						rounding_turn(1, TURN_SPEED, 800);
						Move_Forward(15, 15);
					} else {
						// Turn right for 40 degrees.
						rounding_turn(1, TURN_SPEED, 400);
						Move_Forward(15, 15);
					}
				} else {
					// Turn right for 90 degrees.
					rounding_turn(1, TURN_SPEED, 900);
					Move_Forward(15, 15);
					/*
					if (!Is_MoveWithRemote()) {
						Set_Clean_Mode(Clean_Mode_RandomMode);
					}
					*/
				}
				Wall_Distance = Wall_High_Limit;
			}
		} else {		/* ROUNDING_RIGHT */
			if ((Temp_Bumper_Status & LeftBumperTrig) && !(Temp_Bumper_Status & RightBumperTrig)) {
				// Only left bumper is triggered.
				//Stop_Brifly();
				ROS_WARN("%s %d: move back for left bumper.", __FUNCTION__, __LINE__);
				rounding_move_back(100);
				if (Touch_Detect())
				{
					ROS_INFO("%s %d: Touch detect.", __FUNCTION__, __LINE__);
					break;
				}
				// Turn left for 135 degrees.
				rounding_turn(0, TURN_SPEED, 1350);
				if (Touch_Detect())
				{
					ROS_INFO("%s %d: Touch detect.", __FUNCTION__, __LINE__);
					break;
				}
				Move_Forward(15, 15);
				Wall_Straight_Distance = 375;
			}

			if (Temp_Bumper_Status & RightBumperTrig) {
				Set_Wheel_Speed(0, 0);
				usleep(300000);
				if (robot::instance()->robot_get_right_wall() > (Wall_Low_Limit)) {
					Wall_Distance = robot::instance()->robot_get_right_wall() / 3;
				} else {
					Wall_Distance += 200;
				}

				if (Wall_Distance < Wall_Low_Limit) {
					Wall_Distance = Wall_Low_Limit;
				}
				if (Wall_Distance > Wall_High_Limit) {
					Wall_Distance = Wall_High_Limit;
				}

				if (Temp_Bumper_Status & LeftBumperTrig) {
					// Both left and right bumper are triggered.
					rounding_move_back(350);
					if (Touch_Detect())
					{
						ROS_INFO("%s %d: Touch detect.", __FUNCTION__, __LINE__);
						break;
					}
					// Turn left for 90 degrees.
					rounding_turn(0, TURN_SPEED, 900);
					if (Touch_Detect())
					{
						ROS_INFO("%s %d: Touch detect.", __FUNCTION__, __LINE__);
						break;
					}
					Wall_Straight_Distance = 150;
				} else {
					// Only right bumper is triggered.
					rounding_move_back(350);
					if (Touch_Detect())
					{
						ROS_INFO("%s %d: Touch detect.", __FUNCTION__, __LINE__);
						break;
					}
					// Turn left for 30 degrees.
					rounding_turn(0, TURN_SPEED, 300);
					if (Touch_Detect())
					{
						ROS_INFO("%s %d: Touch detect.", __FUNCTION__, __LINE__);
						break;
					}
					Wall_Straight_Distance = 250;
				}
				Move_Forward(10, 10);

				for (Temp_Counter = 0; Temp_Counter < 3; Temp_Counter++) {
					Right_Wall_Buffer[Temp_Counter] = 0;
				}
			}

			if (Wall_Distance >= 200) {
				Right_Wall_Buffer[2] = Right_Wall_Buffer[1];
				Right_Wall_Buffer[1] = Right_Wall_Buffer[0];
				Right_Wall_Buffer[0] = robot::instance()->robot_get_right_wall();
				if (Right_Wall_Buffer[0] < 100) {
					if ((Right_Wall_Buffer[1] - Right_Wall_Buffer[0]) > (Wall_Distance / 25)) {
						if ((Right_Wall_Buffer[2] - Right_Wall_Buffer[1]) > (Wall_Distance / 25)) {
							//if (Get_WallAccelerate()>300) {
								//if ((Get_RightWheel_Speed()-Get_LeftWheel_Speed()) >= -3) {
									// Away from the wall.
									Move_Forward(18, 16);
									usleep(100000);
									Wall_Straight_Distance = 300;
								//}
							//}
						}
					}
				}
			}

			/*------------------------------------------------------Wheel Speed adjustment-----------------------*/
			if (Get_FrontOBS() < Get_FrontOBST_Value()) {
				Proportion = robot::instance()->robot_get_right_wall();

				Proportion = Proportion * 100 / Wall_Distance;

				Proportion -= 100;

				Delta = Proportion - Previous;

				if (Wall_Distance > 300) {	//over right
					Right_Wall_Speed = 25 + Proportion / 12 + Delta / 5;
					Left_Wall_Speed = 25 - Proportion / 10 - Delta / 5;
					if (Left_Wall_Speed > 33) {
						Right_Wall_Speed = 5; //9;
						Left_Wall_Speed = 33;
					}
				} else {
					Right_Wall_Speed = 15 + Proportion / 22 + Delta / 10;
					Left_Wall_Speed = 15 - Proportion / 18 - Delta / 10;

					if (Left_Wall_Speed > 18) {
						Right_Wall_Speed = 5;
						Left_Wall_Speed = 18;
					}

					if (Right_Wall_Speed > 20) {
						Right_Wall_Speed = 20;
					}
					if (Right_Wall_Speed < 4) {
						Right_Wall_Speed = 4;
					}
					if (Left_Wall_Speed < 4) {
						Left_Wall_Speed = 4;
					}
					if ((Right_Wall_Speed - Left_Wall_Speed) > 5) {
						Right_Wall_Speed = Left_Wall_Speed+5;
					}
				}

				/*slow move if left obs near a wall*/
				if (Get_LeftOBS() > Get_LeftOBST_Value()) {
					if (Wall_Distance < Wall_High_Limit) {
						Wall_Distance++;
					}
				}
				if (Is_WallOBS_Near()){
					Right_Wall_Speed = Right_Wall_Speed / 2;
					Left_Wall_Speed = Left_Wall_Speed / 2;
				}

				Previous = Proportion;

				if (Right_Wall_Speed < 0) {
					Right_Wall_Speed = 0;
				}
				if (Right_Wall_Speed > 40) {
					Right_Wall_Speed = 40;
				}
				if (Left_Wall_Speed < 0) {
					Left_Wall_Speed = 0;
				}

				Move_Forward(Left_Wall_Speed, Right_Wall_Speed);
			} else {
				// 12500 steps means around 2 meters.
				if (Get_RightWheel_Step() < 12500) {
					if (Get_FrontOBS() > Get_FrontOBST_Value()) {
						// Turn left for 80 degrees.
						rounding_turn(0, TURN_SPEED, 800);
						Move_Forward(15, 15);
					} else {
						// Turn left for 40 degrees.
						rounding_turn(0, TURN_SPEED, 400);
						Move_Forward(15, 15);
					}
				} else {
					// Turn left for 90 degrees.
					rounding_turn(0, TURN_SPEED, 900);
					Move_Forward(15, 15);
					/*
					if (!Is_MoveWithRemote()) {
						Set_Clean_Mode(Clean_Mode_RandomMode);
					}
					*/
				}
				Wall_Distance = Wall_High_Limit;
			}
		}
		usleep(10000);
	}

	Stop_Brifly();
	return 0;
}
