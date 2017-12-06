//
// Created by lsy563193 on 6/28/17.
//

#include "pp.h"
#include "arch.hpp"

MovementGoToCharger::MovementGoToCharger()
{
	ROS_INFO("%s %d: Init", __FUNCTION__, __LINE__);
	go_home_state_now = GO_TO_CHARGER_INIT;
}

bool MovementGoToCharger::isReach()
{
	if (ev.charge_detect)
		return true;
	return false;
}

bool MovementGoToCharger::isChargerReach()
{
	if (ev.charge_detect)
		return true;
	return false;
}

void MovementGoToCharger::resetGoToChargerVariables() {
	no_signal_cnt = 0;
	move_away_from_charger_cnt = 0;
	receive_code = 0;
	current_angle = 0;
	last_angle = robot::instance()->getPoseAngle();
	angle_offset = 0;
	gyro_step = 0;
	around_charger_stub_dir = 0;
	go_home_bumper_cnt = 0;
	around_move_cnt = 0;
	position_far = true;
	near_counter = 0;
	side_counter = 0;
	by_path_move_cnt = 0;
	c_rcon.resetStatus();
	turn_connect_cnt = 0;
	turn_connect_dir = ROUND_RIGHT;
}

bool MovementGoToCharger::isSwitch()
{
	if (go_home_state_now == GO_TO_CHARGER_INIT)
	{
		resetGoToChargerVariables();
		g_go_to_charger_back_30cm = false;
		g_go_to_charger_back_10cm = false;
		g_go_to_charger_back_0cm = false;
		go_home_state_now = CHECK_NEAR_CHARGER_STATION;
	}
	if (go_home_state_now == CHECK_NEAR_CHARGER_STATION)
	{
		extern bool g_charge_turn_connect_fail;
		if(g_charge_turn_connect_fail && no_signal_cnt < 10)
		{
			receive_code = c_rcon.getTrig();
			ROS_INFO("%s, %d: check near home, receive_code: %8x", __FUNCTION__, __LINE__, receive_code);
			if(receive_code&RconAll_Home_T)
			{
				ROS_INFO("receive LR");
				if(receive_code&(RconFL_HomeT|RconFR_HomeT|RconFL2_HomeT|RconFR2_HomeT))
				{
					ROS_INFO("%s %d: turn 180", __FUNCTION__, __LINE__);
					g_go_to_charger_back_10cm = true;
					g_turn_angle = 1800;
				}
				else if(receive_code&RconR_HomeT)
				{
					ROS_INFO("%s %d: turn left 90", __FUNCTION__, __LINE__);
					g_go_to_charger_back_10cm = true;
					g_turn_angle = 900;
				}
				else if(receive_code&RconL_HomeT)
				{
					ROS_INFO("%s %d: turn right 90", __FUNCTION__, __LINE__);
					g_go_to_charger_back_10cm = true;
					g_turn_angle = 900;
				}
				else //receive_code&RconBL_HomeT || receive_code&RconBR_HomeT
				{
					ROS_INFO("%s %d: go straight", __FUNCTION__, __LINE__);
					g_go_to_charger_back_0cm = true;
					g_turn_angle = 0;
				}
				go_home_state_now = AWAY_FROM_CHARGER_STATION;
				resetGoToChargerVariables();
				return true;
			}
			else
				no_signal_cnt++;
		}
		else
		{
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			resetGoToChargerVariables();
			g_charge_turn_connect_fail = false;
		}
	}
	if (go_home_state_now == AWAY_FROM_CHARGER_STATION)
	{
		ev.bumper_triggered = bumper.get_status();
		if(ev.bumper_triggered)
		{
			ROS_WARN("%s %d: Get bumper trigered.", __FUNCTION__, __LINE__);
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			g_turn_angle = 0;
			resetGoToChargerVariables();
			return true;
		}
		ev.cliff_triggered = cliff.get_status();
		if(ev.cliff_triggered)
		{
			ROS_WARN("%s %d: Get cliff trigered.", __FUNCTION__, __LINE__);
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			g_turn_angle = 0;
			resetGoToChargerVariables();
			return true;
		}
		if(move_away_from_charger_cnt++ > 50)
		{
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			resetGoToChargerVariables();
		}
	}
	if (go_home_state_now == TURN_FOR_CHARGER_SIGNAL_INIT)
	{
		resetGoToChargerVariables();
		g_go_to_charger_back_30cm = false;
		g_go_to_charger_back_10cm = false;
		g_go_to_charger_back_0cm = false;
		go_home_state_now = TURN_FOR_CHARGER_SIGNAL;
	}
	if (go_home_state_now == TURN_FOR_CHARGER_SIGNAL)
	{
		if(gyro_step < 360)
		{
			// Handle for angle
			current_angle = robot::instance()->getPoseAngle();
			angle_offset = static_cast<float>(ranged_angle((current_angle - last_angle) * 10)) / 10;
			ROS_DEBUG("Current_Angle = %f, Last_Angle = %f, Angle_Offset = %f, Gyro_Step = %f.", current_angle, last_angle, angle_offset, gyro_step);
			if (angle_offset < 0)
				gyro_step += (-angle_offset);
			last_angle = current_angle;

			// Handle for bumper and cliff
			ev.bumper_triggered = bumper.get_status();
			if(ev.bumper_triggered)
			{
				ROS_WARN("%s %d: Get bumper trigered.", __FUNCTION__, __LINE__);
				g_turn_angle = 0;
				resetGoToChargerVariables();
				return true;
			}
			ev.cliff_triggered = cliff.get_status();
			if(ev.cliff_triggered)
			{
				ROS_WARN("%s %d: Get cliff trigered.", __FUNCTION__, __LINE__);
				resetGoToChargerVariables();
				g_turn_angle = 0;
				go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
				return true;
			}

			// Handle for rcon signal
			receive_code = c_rcon.getTrig();
			if (receive_code)
			{
				go_home_state_now = AROUND_CHARGER_STATION_INIT;
			}
			else
				g_turn_angle = 0;

			// HomeL
			if(receive_code & (RconFL_HomeL | RconFR_HomeL))
			{
				g_turn_angle = -900;
				around_charger_stub_dir = 1;
				if(receive_code & RconFL_HomeL)//FL H_L
					ROS_INFO("Start with FL-L.");
				else if(receive_code&RconFR_HomeL)//FR H_L
					ROS_INFO("Start with FR-L.");
			}
			else if(receive_code&RconFL2_HomeL)//FL2 H_L
			{
				ROS_INFO("Start with FL2-L.");
				g_turn_angle = -600;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconFR2_HomeL)//FR2 H_L
			{
				ROS_INFO("Start with FR2-L.");
				g_turn_angle = -850;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconL_HomeL)// L  H_L
			{
				ROS_INFO("Start with L-L.");
				g_turn_angle = 0;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconR_HomeL)// R  H_L
			{
				ROS_INFO("Start with R-L.");
				g_turn_angle = -1500;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconBL_HomeL)//BL H_L
			{
				ROS_INFO("Start with BL-L.");
				g_turn_angle = 800;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconBR_HomeL)//BL H_L
			{
				ROS_INFO("Start with BR-L.");
				g_turn_angle = -800;
				around_charger_stub_dir = 0;
			}
			// HomeR
			else if(receive_code & (RconFL_HomeR | RconFR_HomeR))
			{
				g_turn_angle = 900;
				around_charger_stub_dir = 0;
				if(receive_code&RconFL_HomeR)//FL H_R
					ROS_INFO("Start with FL-R.");
				else if(receive_code&RconFR_HomeR)//FR H_R
					ROS_INFO("Start with FR-R.");
			}
			else if(receive_code&RconFL2_HomeR)//FL2 H_R
			{
				ROS_INFO("Start with FL2-R.");
				g_turn_angle = 850;
				around_charger_stub_dir = 0;
			}
			else if(receive_code&RconFR2_HomeR)//FR2 H_R
			{
				ROS_INFO("Start with FR2-R.");
				g_turn_angle = 600;
				around_charger_stub_dir = 0;
			}
			else if(receive_code&RconL_HomeR)// L  H_R
			{
				ROS_INFO("Start with L-R.");
				g_turn_angle = 1500;
				around_charger_stub_dir = 0;
			}
			else if(receive_code&RconR_HomeR)// R  H_R
			{
				ROS_INFO("Start with R-R.");
				g_turn_angle = 0;
				around_charger_stub_dir = 0;
			}
			else if(receive_code&RconBR_HomeR)//BR H_R
			{
				ROS_INFO("Start with BR-R.");
				g_turn_angle = -800;
				around_charger_stub_dir = 0;
			}
			else if(receive_code&RconBL_HomeR)//BL H_R
			{
				ROS_INFO("Start with BL-R.");
				g_turn_angle = 800;
				around_charger_stub_dir = 1;
			}
			// HomeT
			else if(receive_code&RconFL_HomeT)//FL H_T
			{
				ROS_INFO("Start with FL-T.");
				g_turn_angle = -600;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconFR_HomeT)//FR H_T
			{
				ROS_INFO("Start with FR-T.");
				g_turn_angle = -800;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconFL2_HomeT)//FL2 H_T
			{
				ROS_INFO("Start with FL2-T.");
				g_turn_angle = -600;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconFR2_HomeT)//FR2 H_T
			{
				ROS_INFO("Start with FR2-T.");
				g_turn_angle = -800;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconL_HomeT)// L  H_T
			{
				ROS_INFO("Start with L-T.");
				g_turn_angle = -1200;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconR_HomeT)// R  H_T
			{
				ROS_INFO("Start with R-T.");
				g_turn_angle = -1200;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconBL_HomeT)//BL H_T
			{
				ROS_INFO("Start with BL-T.");
				g_turn_angle = 300;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconBR_HomeT)//BR H_T
			{
				ROS_INFO("Start with BR-T.");
				g_turn_angle = -300;
				around_charger_stub_dir = 0;
			}

			if (g_turn_angle != 0)
			{
				g_go_to_charger_back_0cm = true;
				return true;
			}
		}
		// gyro_step > 360 is handled in MovementGoToCharger::_isStop()
	}
	if (go_home_state_now == AROUND_CHARGER_STATION_INIT)
	{
		go_home_bumper_cnt = 0;
		//wheel.move_forward(9, 9);
		c_rcon.resetStatus();
		ROS_INFO("%s, %d: Call Around_ChargerStation with dir = %d.", __FUNCTION__, __LINE__, around_charger_stub_dir);
		go_home_state_now = AROUND_CHARGER_STATION;
		around_move_cnt = 0;
	}
	else if (go_home_state_now == AROUND_CHARGER_STATION)
	{
		ev.cliff_triggered = cliff.get_status();
		if(ev.cliff_triggered)
		{
			ROS_WARN("%s %d: Get cliff trigered.", __FUNCTION__, __LINE__);
			g_turn_angle = 1750;
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			return true;
		}
		ev.bumper_triggered = bumper.get_status();
		if(ev.bumper_triggered)
		{
			ROS_WARN("%s %d: Get bumper trigered.", __FUNCTION__, __LINE__);
			around_charger_stub_dir = 1 - around_charger_stub_dir;
			g_turn_angle = 1800;
			if(++go_home_bumper_cnt > 1)
				go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			return true;
		}

		if (--around_move_cnt <= 0)
		{
			around_move_cnt = 7;
			receive_code = c_rcon.getTrig();
			if(receive_code)
				no_signal_cnt = 0;
			else if(++no_signal_cnt > 60)
			{
				ROS_WARN("%s %d:No charger signal received.", __FUNCTION__, __LINE__);
				go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			}

			//ROS_DEBUG("%s %d Check DIR: %d, and do something", __FUNCTION__, __LINE__, around_charger_stub_dir);
			if(around_charger_stub_dir == 1)//10.30
			{
				if(receive_code&(RconFR_HomeR|RconFL_HomeR))
				{
					go_home_state_now = BY_PATH_INIT;
					g_turn_angle = 0;
					if(receive_code&RconFR_HomeR)
						ROS_INFO("%s, %d: Detect FR-R, call By_Path().", __FUNCTION__, __LINE__);
					else if(receive_code&RconFL_HomeR)
						ROS_INFO("%s, %d: Detect FL-R, call By_Path().", __FUNCTION__, __LINE__);
				}
				else if(receive_code&RconL_HomeR)
				{
					ROS_INFO("%s, %d: Detect L-R, Check position left.", __FUNCTION__, __LINE__);
					check_position_dir = ROUND_LEFT;
					go_home_state_now = CHECK_POSITION_INIT;
					g_turn_angle = 0;
				}
				else if(receive_code&(RconFL_HomeL|RconFL_HomeT))
				{
					g_turn_angle = -500;
					if(receive_code&RconFL_HomeL)//FL_HL
						ROS_DEBUG("%s, %d: Detect FL-L.", __FUNCTION__, __LINE__);
					else if(receive_code&RconFL_HomeT)//FL_HT
						ROS_DEBUG("%s, %d: Detect FL-T.", __FUNCTION__, __LINE__);
				}
				else if(receive_code&RconFR_HomeT)//FR_HT
				{
					ROS_DEBUG("%s, %d: Detect FR-T.", __FUNCTION__, __LINE__);
					g_turn_angle = -800;
				}
				else if(receive_code&RconFR2_HomeT)//FR2_T
				{
					ROS_DEBUG("%s, %d: Detect FR2-T.", __FUNCTION__, __LINE__);
					g_turn_angle = -900;
				}
				else if(receive_code&RconR_HomeT)//R_HT
				{
					ROS_DEBUG("%s, %d: Detect R-T.", __FUNCTION__, __LINE__);
					g_turn_angle = -1100;
					around_charger_stub_dir = 0;
				}
				else
					g_turn_angle = 0;

				if (g_turn_angle != 0)
				{
					g_go_to_charger_back_0cm = true;
					return true;
				}
			}
			else //around_charger_stub_dir == 0
			{
				if(receive_code&(RconFL_HomeL|RconFR_HomeL))
				{
					go_home_state_now = BY_PATH_INIT;
					g_turn_angle = 0;
					if(receive_code&(RconFL_HomeL))
						ROS_INFO("%s, %d: Detect FL-L, call By_Path().", __FUNCTION__, __LINE__);
					else if(receive_code&(RconFR_HomeL))
						ROS_INFO("%s, %d: Detect FR-L, call By_Path().", __FUNCTION__, __LINE__);
				}
				else if(receive_code&(RconR_HomeL))
				{
					ROS_INFO("%s, %d: Detect R-L, check position right.", __FUNCTION__, __LINE__);
					check_position_dir = ROUND_RIGHT;
					go_home_state_now = CHECK_POSITION_INIT;
					g_turn_angle = 0;
				}
				else if(receive_code&(RconFR_HomeR|RconFR_HomeT))
				{
					g_turn_angle = 500;
					if(receive_code&RconFR_HomeR)
						ROS_DEBUG("%s, %d: Detect FR-R.", __FUNCTION__, __LINE__);
					else if(receive_code&RconFR_HomeT)
						ROS_DEBUG("%s, %d: Detect FR-T.", __FUNCTION__, __LINE__);
				}
				else if(receive_code&RconFL_HomeT)
				{
					ROS_DEBUG("%s, %d: Detect FL-T.", __FUNCTION__, __LINE__);
					g_turn_angle = 800;
				}
				else if(receive_code&RconFL2_HomeT)
				{
					ROS_DEBUG("%s, %d: Detect FL2-T.", __FUNCTION__, __LINE__);
					g_turn_angle = 900;
				}
				else if(receive_code&RconL_HomeT)
				{
					ROS_DEBUG("%s, %d: Detect L-T.", __FUNCTION__, __LINE__);
					g_turn_angle = 1100;
					around_charger_stub_dir = 1;
				}
				else
					g_turn_angle = 0;

				if (g_turn_angle != 0)
				{
					g_go_to_charger_back_0cm = true;
					return true;
				}
			}
		}
	}
	if (go_home_state_now == CHECK_POSITION_INIT)
	{
		resetGoToChargerVariables();
		go_home_state_now = CHECK_POSITION;
	}
	if (go_home_state_now == CHECK_POSITION)
	{
		ev.bumper_triggered = bumper.get_status();
		if(ev.bumper_triggered)
		{
			ROS_WARN("%s %d: Get bumper trigered.", __FUNCTION__, __LINE__);
			around_charger_stub_dir = 1 - around_charger_stub_dir;
			if(++go_home_bumper_cnt > 1)
				go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			else
				go_home_state_now = AROUND_CHARGER_STATION_INIT;
			g_turn_angle = 1800;
			return true;
		}
		ev.cliff_triggered = cliff.get_status();
		if(ev.cliff_triggered)
		{
			ROS_WARN("%s %d: Get cliff trigered.", __FUNCTION__, __LINE__);
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			g_turn_angle = 1800;
			return true;
		}

		if(gyro_step < 360)
		{
			current_angle = robot::instance()->getPoseAngle();
			angle_offset = static_cast<float>(ranged_angle((current_angle - last_angle) * 10)) / 10;
			ROS_DEBUG("%s %d: Current_Angle = %f, Last_Angle = %f, Angle_Offset = %f, Gyro_Step = %f.", __FUNCTION__, __LINE__, current_angle, last_angle, angle_offset, gyro_step);
			if (check_position_dir == ROUND_LEFT && angle_offset > 0)
				gyro_step += angle_offset;
			if (check_position_dir == ROUND_RIGHT && angle_offset < 0)
				gyro_step += (-angle_offset);
			last_angle = current_angle;

			ROS_DEBUG("%s %d: Check_Position c_rcon.getStatus() == %8x, R... == %8x.", __FUNCTION__, __LINE__,
								c_rcon.getStatus(), RconFrontAll_Home_LR);
			receive_code = (c_rcon.getTrig()&RconFrontAll_Home_LR);
			ROS_DEBUG("%s %d: receive code: %8x.", __FUNCTION__, __LINE__, receive_code);
			if(receive_code)
			{
				if (receive_code & RconL_HomeL)ROS_DEBUG("Check_Position get L-L");
				if (receive_code & RconL_HomeR)ROS_DEBUG("Check_Position get L-R");
				if (receive_code & RconFL_HomeL)ROS_DEBUG("Check_Position get FL-L");
				if (receive_code & RconFL_HomeR)ROS_DEBUG("Check_Position get FL-R");
				if (receive_code & RconFL2_HomeL)ROS_DEBUG("Check_Position get FL2-L");
				if (receive_code & RconFL2_HomeR)ROS_DEBUG("Check_Position get FL2-R");
				if (receive_code & RconR_HomeL)ROS_DEBUG("Check_Position get R-L");
				if (receive_code & RconR_HomeR)ROS_DEBUG("Check_Position get R-R");
				if (receive_code & RconFR_HomeL)ROS_DEBUG("Check_Position get FR-L");
				if (receive_code & RconFR_HomeR)ROS_DEBUG("Check_Position get FR-R");
				if (receive_code & RconFR2_HomeL)ROS_DEBUG("Check_Position get FR2-L");
				if (receive_code & RconFR2_HomeR)ROS_DEBUG("Check_Position get FR2-R");
			}

			if(receive_code & (RconFR_HomeL|RconFR_HomeR) && check_position_dir == ROUND_LEFT)
				go_home_state_now = BY_PATH_INIT;
			if(receive_code & (RconFL_HomeL|RconFL_HomeR) && check_position_dir == ROUND_RIGHT)
				go_home_state_now = BY_PATH_INIT;
		}
		if(gyro_step >= 360)
		{
			ROS_INFO("%s, %d: Robot can't see charger, restart go to charger process.", __FUNCTION__, __LINE__);
			g_turn_angle = 1000;
			g_go_to_charger_back_0cm = true;
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			return true;
		}
	}
	if (go_home_state_now == BY_PATH_INIT)
	{
		resetGoToChargerVariables();
		go_home_state_now = BY_PATH;
	}
	if (go_home_state_now == BY_PATH)
	{
		ev.bumper_triggered = bumper.get_status();
		if(ev.bumper_triggered)
		{
			ROS_INFO("bumper in by path!");
			if(!position_far)
			{
				go_home_state_now = TURN_CONNECT;
				turn_connect_dir = ROUND_RIGHT;
				turn_connect_cnt = 0;
			}
			else
			{
				//if((c_rcon.getStatus()&RconFront_Home_LR) == 0)
				//	go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
				go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
				g_go_to_charger_back_10cm = true;
				//if(ev.bumper_triggered & BLOCK_LEFT)
				//	g_turn_angle = -1100;
				//else
				//	g_turn_angle = 1100;
				ROS_WARN("%d: quick_back in position_far", __LINE__);
				return true;
			}
		}
		ev.cliff_triggered = cliff.get_status();
		if(ev.cliff_triggered)
		{
			g_turn_angle = 1750;
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			return true;
		}

		if (--by_path_move_cnt < 0)
		{
			by_path_move_cnt = 25;
			receive_code = c_rcon.getTrig();
			if(receive_code)
			{
				if(receive_code&RconFR_HomeT && receive_code&RconFL_HomeT)
				{
					position_far = false;
					ROS_DEBUG("%s, %d: Robot face HomeT, position_far = false.", __FUNCTION__, __LINE__);
				}
				else if(receive_code&(RconFL2_HomeT|RconFR2_HomeT|RconL_HomeT|RconR_HomeT))
				{
					position_far = false;
					ROS_DEBUG("%s, %d: Robot side face HomeT, position_far = false.", __FUNCTION__, __LINE__);
				}
				if(receive_code&RconFrontAll_Home_T)
				{
					if(++near_counter > 1)
					{
						position_far = false;
						ROS_DEBUG("%s, %d: Robot near HomeT counter > 1, position_far = false.", __FUNCTION__, __LINE__);
					}
					else
						near_counter = 0;
					if((receive_code&RconFront_Home_LR) == 0 && ++side_counter > 5)
					{
						ROS_INFO("%s, %d: Robot away from the front of charger stub, back to gohome mode_.", __FUNCTION__, __LINE__);
						go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
						g_go_to_charger_back_0cm = true;
						g_turn_angle = 0;
						return true;
					}
					else
						side_counter = 0;
				}

				if(receive_code&RconFL_HomeL && receive_code&RconFR_HomeR)
				{
					ROS_DEBUG("%s, %d: Robot sees HomeL or HomeR, position_far = false.", __FUNCTION__, __LINE__);
					position_far = false;
				}

				auto temp_code = receive_code;
				temp_code &= RconFrontAll_Home_LR;
				if (temp_code)
				{
					g_go_to_charger_back_0cm = true;
					if(position_far)
					{
						switch(temp_code)
						{
							case (RconFR2_HomeL):
								ROS_DEBUG("%s, %d: position_far, FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -250;
								return true;
							case (RconFR2_HomeL|RconFR2_HomeR):
								ROS_DEBUG("%s, %d: position_far, FR2_L/FR2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = -350;
								return true;
							case (RconR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
								ROS_DEBUG("%s, %d: position_far, R_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = -250;
								return true;
							case (RconR_HomeR|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: position_far, R_R/FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -400;
								return true;
							case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
								ROS_DEBUG("%s, %d: position_far, R_R/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = -250;
								return true;
							case (RconR_HomeR|RconR_HomeL|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: position_far, R_R/R_L/FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -450;
								return true;
							case (RconR_HomeR|RconR_HomeL):
								ROS_DEBUG("%s, %d: position_far, R_R/R_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -250;
								return true;
							case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: position_far, R_R/FR_L/FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -550;
								return true;
							case (RconR_HomeL|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: position_far, R_L/FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -500;
								return true;
							case (RconFL2_HomeR):
								ROS_DEBUG("%s, %d: position_far, FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 250;
								return true;
							case (RconFL2_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: position_far, FL2_/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 350;
								return true;
							case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_L/FL2_L/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 250;
								return true;
							case (RconL_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_L/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 400;
								return true;
							case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_L/FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 250;
								return true;
							case (RconL_HomeR|RconL_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_R/L_L/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 450;
								return true;
							case (RconL_HomeR|RconL_HomeL):
								ROS_DEBUG("%s, %d: position_far, L_R/L_L.", __FUNCTION__, __LINE__);
								g_turn_angle = 500;
								return true;
							case (RconL_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 550;
								return true;
							case (RconL_HomeL|RconFL2_HomeR|RconFL_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 250;
								return true;
							case (RconL_HomeR|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_R/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 500;
								return true;
						}
					}
					else
					{
						switch(temp_code)
						{
							case (RconFR2_HomeL):
								ROS_DEBUG("%s, %d: !position_far, FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -250;
								return true;
							case (RconFR2_HomeL|RconFR2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, FR2_L/FR2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = -350;
								return true;
							case (RconR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, R_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = -250;
								return true;
							case (RconR_HomeR|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: !position_far, R_R/FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -400;
								return true;
							case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, R_R/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = -250;
								return true;
							case (RconR_HomeR|RconR_HomeL|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: !position_far, R_R/R_L/FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -450;
								return true;
							case (RconR_HomeR|RconR_HomeL):
								ROS_DEBUG("%s, %d: !position_far, R_R/R_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -500;
								return true;
							case (RconR_HomeL):
								ROS_DEBUG("%s, %d: !position_far, R_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -550;
								return true;
							case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: !position_far, R_R/FR_L/FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -250;
								return true;
							case (RconR_HomeL|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: !position_far, R_L/FR2_L.", __FUNCTION__, __LINE__);
								g_turn_angle = -500;
								return true;
							case (RconFL2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 250;
								return true;
							case (RconFL2_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, FL2_/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 350;
								return true;
							case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_L/FL2_L/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 250;
								return true;
							case (RconL_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_L/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 400;
								return true;
							case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_L/FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 250;
								return true;
							case (RconL_HomeR|RconL_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_R/L_L/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 450;
								return true;
							case (RconL_HomeR|RconL_HomeL):
								ROS_DEBUG("%s, %d: !position_far, L_R/L_L.", __FUNCTION__, __LINE__);
								g_turn_angle = 500;
								return true;
							case (RconL_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 250;
								return true;
							case (RconL_HomeL|RconFL2_HomeR|RconFL_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 250;
								return true;
							case (RconL_HomeR|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_R/FL2_R.", __FUNCTION__, __LINE__);
								g_turn_angle = 500;
								return true;
						}
					}
					g_go_to_charger_back_0cm = false;
				}
				no_signal_cnt = 0;
			}
			else
			{
				near_counter = 0;
				if(++no_signal_cnt > 1)
				{
					ROS_INFO("%s %d: No signal in by path, switch to check position.", __FUNCTION__, __LINE__);
					check_position_dir = ROUND_LEFT;
					go_home_state_now = CHECK_POSITION_INIT;
				}
			}
		}
	}
	if (go_home_state_now == TURN_CONNECT)
	{
		if (turn_connect_dir == ROUND_RIGHT && ++turn_connect_cnt > 50)
		{
			turn_connect_dir = ROUND_LEFT;
			turn_connect_cnt = 0;
		}
		if (turn_connect_dir == ROUND_LEFT && ++turn_connect_cnt > 50)
		{
			turn_connect_cnt = 0;
			g_go_to_charger_back_30cm = true;
			ROS_WARN("%s %d: Turn connect failed, move back for 0.1m.", __FUNCTION__, __LINE__);
			g_turn_angle = 0;
			go_home_state_now = TURN_FOR_CHARGER_SIGNAL_INIT;
			return true;
		}
	}

	return false;
}

bool MovementGoToCharger::_isStop()
{
	if (g_robot_stuck || (go_home_state_now == TURN_FOR_CHARGER_SIGNAL && gyro_step > 360))
	{
		ROS_WARN("%s %d: Stop here", __FUNCTION__, __LINE__);
		cs.setNext(CS_CLEAN);
		return true;
	}
	return false;
}

void MovementGoToCharger::setTarget()
{
	g_turn_angle = 0;
}

void MovementGoToCharger::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
	/*---check if near charger station---*/
	if (go_home_state_now == CHECK_NEAR_CHARGER_STATION)
	{
		wheel.setDirectionForward();
		l_speed = r_speed = 0;
	}
	else if (go_home_state_now == AWAY_FROM_CHARGER_STATION)
	{
		wheel.setDirectionForward();
		l_speed = r_speed = 30;
	}
	else if (go_home_state_now == TURN_FOR_CHARGER_SIGNAL)
	{
		wheel.setDirectionRight();
		l_speed = r_speed = 10;
	}
	else if (go_home_state_now == AROUND_CHARGER_STATION_INIT)
	{
		wheel.setDirectionForward();
		l_speed = r_speed = 9;
		around_move_cnt = 0;
	}
	else if (go_home_state_now == AROUND_CHARGER_STATION)
	{
		if (around_move_cnt == 7 && around_charger_stub_dir == 1)
		{
			if(receive_code&RconL_HomeT)
			{
				ROS_DEBUG("%s, %d: Detect L-T.", __FUNCTION__, __LINE__);
				wheel.setDirectionForward();
				l_speed = 22;
				r_speed = 12;
			}
			else if(receive_code&RconL_HomeL)
			{
				ROS_DEBUG("%s, %d: Detect L-L.", __FUNCTION__, __LINE__);
				wheel.setDirectionForward();
				l_speed = 22;
				r_speed = 12;
			}
			else if(receive_code&RconFL2_HomeT)
			{
				ROS_DEBUG("%s, %d: Detect FL2-T.", __FUNCTION__, __LINE__);
				wheel.setDirectionForward();
				l_speed = 23;
				r_speed = 14;
			}
			else if(receive_code&RconFL2_HomeL)
			{
				ROS_DEBUG("%s, %d: Detect FL2-L.", __FUNCTION__, __LINE__);
				wheel.setDirectionForward();
				l_speed = 20;
				r_speed = 14;
			}
			else if(receive_code&RconFL2_HomeR)
			{
				ROS_DEBUG("%s, %d: Detect FL2-R.", __FUNCTION__, __LINE__);
				wheel.setDirectionForward();
				l_speed = 15;
				r_speed = 21;
			}
			else
			{
				ROS_DEBUG("%s, %d: Else.", __FUNCTION__, __LINE__);
				wheel.setDirectionForward();
				l_speed = 14;
				r_speed = 21;
			}
		}
		else if (around_move_cnt == 7 && around_charger_stub_dir == 0)
		{
			if(receive_code&RconR_HomeT)
			{
				ROS_DEBUG("%s %d Detect R-T.", __FUNCTION__, __LINE__);
				wheel.setDirectionForward();
				l_speed = 12;
				r_speed = 22;
			}
			else if(receive_code&RconR_HomeR)
			{
				ROS_DEBUG("%s %d Detect R-R.", __FUNCTION__, __LINE__);
				wheel.setDirectionForward();
				l_speed = 12;
				r_speed = 22;
			}
			else if(receive_code&RconFR2_HomeT)
			{
				ROS_DEBUG("%s %d Detect FR2-T.", __FUNCTION__, __LINE__);
				wheel.setDirectionForward();
				l_speed = 14;
				r_speed = 23;
			}
			else if(receive_code&RconFR2_HomeR)
			{
				ROS_DEBUG("%s %d Detect FR2-R.", __FUNCTION__, __LINE__);
				wheel.setDirectionForward();
				l_speed = 14;
				r_speed = 20;
			}
			else if(receive_code&RconFR2_HomeL)
			{
				ROS_DEBUG("%s, %d: Detect FL2-R.", __FUNCTION__, __LINE__);
				wheel.setDirectionForward();
				l_speed = 21;
				r_speed = 15;
			}
			else
			{
				ROS_DEBUG("%s, %d: Else.", __FUNCTION__, __LINE__);
				wheel.setDirectionForward();
				l_speed = 21;
				r_speed = 14;
			}
		}
		else
		{
			wheel.setDirectionForward();
			l_speed = left_speed_;
			r_speed = right_speed_;
		}
	}
	else if (go_home_state_now == CHECK_POSITION)
	{
		ROS_DEBUG("%s, %d: Check position dir: %d.", __FUNCTION__, __LINE__, check_position_dir);
		if(check_position_dir == ROUND_LEFT)
			wheel.setDirectionLeft();
		else if(check_position_dir == ROUND_RIGHT)
			wheel.setDirectionRight();
		l_speed = r_speed = 10;
	}
	else if (go_home_state_now == BY_PATH)
	{
		wheel.setDirectionForward();
		auto temp_code = receive_code;
		temp_code &= RconFrontAll_Home_LR;
		if (by_path_move_cnt == 25 && temp_code)
		{
			if(position_far)
			{
				switch(temp_code)
				{
					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR2_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR2_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 11;
						break;
					case (RconFL2_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 9;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 9;
						break;
					case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 8;
						break;
					case (RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 8;
						break;
					case (RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FR_L.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 8;
						break;
					case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 7;
						break;
					case (RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 7;
						break;
					case (RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 7;
						break;
					case (RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 6;
						break;
					case (RconFL_HomeL|RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 10;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 9;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 10;
						break;
					case (RconFR_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: position_far, FR_L/FR2_L.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 7;
						break;
					case (RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 11;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 10;
						break;
					case (RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 8;
						break;
					case (RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 11;
						break;
					case (RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 10;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 10;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 11;
						break;
					case (RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 9;
						break;
					case (RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 11;
						break;
					case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 11;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 11;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 11;
						break;
					case (RconFL_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 11;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 12;
						break;
					case (RconR_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, R_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 2;
						break;
					case (RconR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, R_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 0;
						break;
					case (RconL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, L_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 1;
						break;
					case (RconL_HomeL|RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, L_L/FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 6;
						break;
					case (RconR_HomeR):
						ROS_DEBUG("%s, %d: position_far, R_R.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 0;
						break;
					case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 12;
						break;
					case (RconFL_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR2_L.", __FUNCTION__, __LINE__);
						l_speed = 13;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 14;
						r_speed = 4;
						break;
					case (RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 12;
						r_speed = 11;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 13;
						break;
					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 13;
						break;
					case (RconFL_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 12;
						break;
					case (RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 11;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FL_L/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 13;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 13;
						break;
					case (RconFL2_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 14;
						break;
					case (RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 12;
						break;
					case (RconFL2_HomeR|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_R/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 12;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_L/FR2_L.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 12;
						break;
					case (RconFL_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL_L.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR2_L.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 12;
						break;
					case (RconFL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 12;
						break;
					case (RconL_HomeL|RconFL2_HomeL|RconFL_HomeL):
						ROS_DEBUG("%s, %d: position_far, L_L/FL2_L/FL_L.", __FUNCTION__, __LINE__);
						l_speed = 2;
						r_speed = 9;
						break;
					case (RconL_HomeL|RconFL2_HomeL):
						ROS_DEBUG("%s, %d: position_far, L_L/FL2_L.", __FUNCTION__, __LINE__);
						l_speed = 0;
						r_speed = 9;
						break;
					case (RconR_HomeR|RconFL2_HomeL):
						ROS_DEBUG("%s, %d: position_far, R_R/FL2_L.", __FUNCTION__, __LINE__);
						l_speed = 1;
						r_speed = 9;
						break;
					case (RconR_HomeR|RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, R_R/FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 6;
						r_speed = 9;
						break;
					case (RconL_HomeL):
						ROS_DEBUG("%s, %d: position_far, L_L.", __FUNCTION__, __LINE__);
						l_speed = 0;
						r_speed = 10;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL2_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					case (RconFL_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL_R/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 12;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 11;
						r_speed = 12;
						break;
					default:
						ROS_DEBUG("%s, %d: position_far, else:%x.", __FUNCTION__, __LINE__, temp_code);
						l_speed = 10;
						r_speed = 12;
				}
			}
			else
			{
				switch(temp_code)
				{
					case (RconFL_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL-L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = r_speed = 8;
						break;
					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR2_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR2_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 7;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 6;
						break;
					case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 6;
						break;
					case (RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 6;
						break;
					case (RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FR_L.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 4;
						break;
					case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 3;
						break;
					case (RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 5;
						break;
					case (RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 3;
						break;
					case (RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 3;
						break;
					case (RconFL_HomeL|RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 6;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 7;
						break;
					case (RconFR_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FR_L/FR2_L.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 4;
						break;
					case (RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 6;
						break;
					case (RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 7;
						break;
					case (RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 6;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 3;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 3;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 8;
						break;
					case (RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 6;
						break;
					case (RconR_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, R_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 2;
						break;
					case (RconR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, R_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 0;
						break;
					case (RconL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, L_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 1;
						break;
					case (RconL_HomeL|RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, L_L/FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 6;
						break;
					case (RconR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, R_R.", __FUNCTION__, __LINE__);
						l_speed = 10;
						r_speed = 0;
						break;
					case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 7;
						break;
					case (RconFL_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR2_L.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 2;
						break;
					case (RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 9;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 10;
						break;
					case (RconFL2_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 9;
						break;
					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 6;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 6;
						r_speed = 9;
						break;
					case (RconFL_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 6;
						r_speed = 8;
						break;
					case (RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_R.", __FUNCTION__, __LINE__);
						l_speed = 4;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FL_L/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 3;
						r_speed = 7;
						break;
					case (RconFL2_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 5;
						r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 3;
						r_speed = 8;
						break;
					case (RconFL2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L.", __FUNCTION__, __LINE__);
						l_speed = 3;
						r_speed = 9;
						break;
					case (RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 6;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 9;
						break;
					case (RconFL2_HomeR|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_R/FL_R.", __FUNCTION__, __LINE__);
						l_speed = 4;
						r_speed = 9;
						break;
					case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_L/FR2_L.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 6;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 9;
						break;
					case (RconFL_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL_L.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR2_L.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 6;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 3;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 3;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 9;
						break;
					case (RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 6;
						r_speed = 9;
						break;
					case (RconL_HomeL|RconFL2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, L_L/FL2_L.", __FUNCTION__, __LINE__);
						l_speed = 0;
						r_speed = 9;
						break;
					case (RconR_HomeR|RconFL2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, R_R/FL2_L.", __FUNCTION__, __LINE__);
						l_speed = 1;
						r_speed = 9;
						break;
					case (RconR_HomeR|RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, R_R/FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 6;
						r_speed = 9;
						break;
					case (RconL_HomeL):
						ROS_DEBUG("%s, %d: !position_far, L_L.", __FUNCTION__, __LINE__);
						l_speed = 0;
						r_speed = 10;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						l_speed = 7;
						r_speed = 8;
						break;
					case (RconFL2_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 9;
						break;
					case (RconFL_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL_R/FR_L.", __FUNCTION__, __LINE__);
						l_speed = 2;
						r_speed = 9;
						break;
					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FR_R.", __FUNCTION__, __LINE__);
						l_speed = 8;
						r_speed = 9;
						break;
					default:
						ROS_DEBUG("%s, %d: !position_far, else:%x.", __FUNCTION__, __LINE__, temp_code);
						l_speed = 7;
						r_speed = 8;
						break;
				}
			}
		}
		else
		{
			wheel.setDirectionForward();
			l_speed = left_speed_;
			r_speed = right_speed_;
		}
	}
	else if (go_home_state_now == TURN_CONNECT)
	{
		if (turn_connect_dir == ROUND_RIGHT)
			wheel.setDirectionRight();
		else if (turn_connect_dir == ROUND_LEFT)
			wheel.setDirectionLeft();
		l_speed = r_speed = 5;
	}
	left_speed_ = l_speed;
	right_speed_ = r_speed;
}
