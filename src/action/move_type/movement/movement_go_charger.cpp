//
// Created by lsy563193 on 6/28/17.
//

#include <movement.hpp>
#include <move_type.hpp>
#include <robot.hpp>
#include <dev.h>

MovementGoToCharger::MovementGoToCharger()
{
	ROS_INFO("%s %d: Init", __FUNCTION__, __LINE__);
	gtc_state_now_ = gtc_init;
}

void MovementGoToCharger::resetGoToChargerVariables()
{
	turn_angle_ = 0;
	back_distance_ = 0;
	no_signal_cnt = 0;
	move_away_from_charger_time_stamp_ = ros::Time::now().toSec();
	receive_code = 0;
	current_radian_ = 0;
	last_radian_ = robot::instance()->getWorldPoseRadian();
	radian_offset_ = 0;
	gyro_radian_step_ = 0;
	around_charger_stub_dir = 0;
	go_home_bumper_cnt = 0;
	around_move_cnt = 0;
	position_far = true;
	near_counter = 0;
	side_counter = 0;
	by_path_move_cnt = 0;
	c_rcon.resetStatus();
	turn_connect_cnt = 0;
	turn_connect_dir = gtc_check_position_right;
}

bool MovementGoToCharger::isSwitch()
{
	if (gtc_state_now_ == gtc_init)
	{
		resetGoToChargerVariables();
		gtc_state_now_ = gtc_check_near_charger_station;
	}
	if (gtc_state_now_ == gtc_check_near_charger_station)
	{
//		extern bool g_charge_turn_connect_fail;
		if(/*g_charge_turn_connect_fail &&*/ no_signal_cnt < 10)
		{
			receive_code = c_rcon.getAll();
			ROS_INFO("%s, %d: check near home, receive_code: %8x", __FUNCTION__, __LINE__, receive_code);
			if(receive_code&RconAll_Home_T)
			{
				ROS_INFO("receive LR");
				if(receive_code&(RconFL_HomeT|RconFR_HomeT|RconFL2_HomeT|RconFR2_HomeT))
				{
					ROS_INFO("%s %d: turn 180", __FUNCTION__, __LINE__);
					back_distance_ = 0.1;
					turn_angle_ = 180;
				}
				else if(receive_code&RconR_HomeT)
				{
					ROS_INFO("%s %d: turn left 90", __FUNCTION__, __LINE__);
					back_distance_ = 0.1;
					turn_angle_ = 90;
				}
				else if(receive_code&RconL_HomeT)
				{
					ROS_INFO("%s %d: turn right 90", __FUNCTION__, __LINE__);
					back_distance_ = 0.1;
					turn_angle_ = 90;
				}
				else //receive_code&RconBL_HomeT || receive_code&RconBR_HomeT
				{
					ROS_INFO("%s %d: go straight", __FUNCTION__, __LINE__);
					back_distance_ = 0;
					turn_angle_ = 0;
				}
				gtc_state_now_ = gtc_away_from_charger_station;
				resetGoToChargerVariables();
				return true;
			}
			else
				no_signal_cnt++;
		}
		else
		{
			gtc_state_now_ = gtc_turn_for_charger_signal_init;
			resetGoToChargerVariables();
//			g_charge_turn_connect_fail = false;
		}
	}
	if (gtc_state_now_ == gtc_away_from_charger_station)
	{
		if(bumper.getStatus())
		{
			ROS_WARN("%s %d: Get bumper triggered.", __FUNCTION__, __LINE__);
			gtc_state_now_ = gtc_turn_for_charger_signal_init;
			turn_angle_ = 0;
			back_distance_ = 0.01;
			return true;
		}
		if(cliff.getStatus())
		{
			ROS_WARN("%s %d: Get cliff triggered.", __FUNCTION__, __LINE__);
			gtc_state_now_ = gtc_turn_for_charger_signal_init;
			turn_angle_ = 0;
			back_distance_ = 0.01;
			return true;
		}
		if(ros::Time::now().toSec() - move_away_from_charger_time_stamp_ > 2)
		{
			gtc_state_now_ = gtc_turn_for_charger_signal_init;
			resetGoToChargerVariables();
		}
	}
	if (gtc_state_now_ == gtc_turn_for_charger_signal_init)
	{
		resetGoToChargerVariables();
		gtc_state_now_ = gtc_turn_for_charger_signal;
	}
	if (gtc_state_now_ == gtc_turn_for_charger_signal)
	{
//		ROS_DEBUG("%s %d: current_radian_ = %f, last_radian_ = %f, radian_offset_ = %f, gyro_radian_step_ = %f.",
//				  __FUNCTION__, __LINE__, current_radian_, last_radian_, radian_offset_, gyro_radian_step_);
		if(gyro_radian_step_ < 2 * PI)
		{
			// Handle for angle
			current_radian_ = getPosition().th;
			radian_offset_ = ranged_radian(current_radian_ - last_radian_);
			ROS_DEBUG("%s %d: current_radian_ = %f, last_radian_ = %f, radian_offset_ = %f, gyro_radian_step_ = %f.",
					  __FUNCTION__, __LINE__, current_radian_, last_radian_, radian_offset_, gyro_radian_step_);
			if (radian_offset_ < 0)
				gyro_radian_step_ += fabs(radian_offset_);
			last_radian_ = current_radian_;

			// Handle for bumper and cliff
			if(bumper.getStatus())
			{
				ROS_WARN("%s %d: Get bumper triggered.", __FUNCTION__, __LINE__);
				turn_angle_ = 0;
				back_distance_ = 0.01;
				gtc_state_now_ = gtc_turn_for_charger_signal_init;
				return true;
			}
			if(cliff.getStatus())
			{
				ROS_WARN("%s %d: Get cliff triggered.", __FUNCTION__, __LINE__);
				turn_angle_ = 0;
				back_distance_ = 0.01;
				gtc_state_now_ = gtc_turn_for_charger_signal_init;
				return true;
			}

			// Handle for rcon signal
			receive_code = c_rcon.getAll();
			//ROS_INFO("%s %d: rcon recieve code:%d", __FUNCTION__, __LINE__, receive_code);
			if (receive_code)
			{
				gtc_state_now_ = gtc_around_charger_station_init;
			}
			else
				turn_angle_ = 0;

			// HomeL
			if(receive_code & (RconFL_HomeL | RconFR_HomeL))
			{
				turn_angle_ = -90;
				around_charger_stub_dir = 1;
				if(receive_code & RconFL_HomeL)//FL H_L
					ROS_INFO("Start with FL-L.");
				else if(receive_code&RconFR_HomeL)//FR H_L
					ROS_INFO("Start with FR-L.");
			}
			else if(receive_code&RconFL2_HomeL)//FL2 H_L
			{
				ROS_INFO("Start with FL2-L.");
				turn_angle_ = -60;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconFR2_HomeL)//FR2 H_L
			{
				ROS_INFO("Start with FR2-L.");
				turn_angle_ = -80;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconL_HomeL)// L  H_L
			{
				ROS_INFO("Start with L-L.");
				turn_angle_ = 0;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconR_HomeL)// R  H_L
			{
				ROS_INFO("Start with R-L.");
				turn_angle_ = -150;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconBL_HomeL)//BL H_L
			{
				ROS_INFO("Start with BL-L.");
				turn_angle_ = 80;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconBR_HomeL)//BL H_L
			{
				ROS_INFO("Start with BR-L.");
				turn_angle_ = -80;
				around_charger_stub_dir = 0;
			}
			// HomeR
			else if(receive_code & (RconFL_HomeR | RconFR_HomeR))
			{
				turn_angle_ = 90;
				around_charger_stub_dir = 0;
				if(receive_code&RconFL_HomeR)//FL H_R
					ROS_INFO("Start with FL-R.");
				else if(receive_code&RconFR_HomeR)//FR H_R
					ROS_INFO("Start with FR-R.");
			}
			else if(receive_code&RconFL2_HomeR)//FL2 H_R
			{
				ROS_INFO("Start with FL2-R.");
				turn_angle_ = 85;
				around_charger_stub_dir = 0;
			}
			else if(receive_code&RconFR2_HomeR)//FR2 H_R
			{
				ROS_INFO("Start with FR2-R.");
				turn_angle_ = 60;
				around_charger_stub_dir = 0;
			}
			else if(receive_code&RconL_HomeR)// L  H_R
			{
				ROS_INFO("Start with L-R.");
				turn_angle_ = 150;
				around_charger_stub_dir = 0;
			}
			else if(receive_code&RconR_HomeR)// R  H_R
			{
				ROS_INFO("Start with R-R.");
				turn_angle_ = 0;
				around_charger_stub_dir = 0;
			}
			else if(receive_code&RconBR_HomeR)//BR H_R
			{
				ROS_INFO("Start with BR-R.");
				turn_angle_ = -80;
				around_charger_stub_dir = 0;
			}
			else if(receive_code&RconBL_HomeR)//BL H_R
			{
				ROS_INFO("Start with BL-R.");
				turn_angle_ = 80;
				around_charger_stub_dir = 1;
			}
			// HomeT
			else if(receive_code&RconFL_HomeT)//FL H_T
			{
				ROS_INFO("Start with FL-T.");
				turn_angle_ = -60;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconFR_HomeT)//FR H_T
			{
				ROS_INFO("Start with FR-T.");
				turn_angle_ = -80;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconFL2_HomeT)//FL2 H_T
			{
				ROS_INFO("Start with FL2-T.");
				turn_angle_ = -60;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconFR2_HomeT)//FR2 H_T
			{
				ROS_INFO("Start with FR2-T.");
				turn_angle_ = -80;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconL_HomeT)// L  H_T
			{
				ROS_INFO("Start with L-T.");
				turn_angle_ = -120;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconR_HomeT)// R  H_T
			{
				ROS_INFO("Start with R-T.");
				turn_angle_ = -120;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconBL_HomeT)//BL H_T
			{
				ROS_INFO("Start with BL-T.");
				turn_angle_ = 30;
				around_charger_stub_dir = 1;
			}
			else if(receive_code&RconBR_HomeT)//BR H_T
			{
				ROS_INFO("Start with BR-T.");
				turn_angle_ = -30;
				around_charger_stub_dir = 0;
			}

			if (turn_angle_ != 0)
			{
				back_distance_ = 0;
				return true;
			}
		}
		// gyro_radian_step_ >= 2 * PI is handled in MovementGoToCharger::_isStop()
	}
	if (gtc_state_now_ == gtc_around_charger_station_init)
	{
		go_home_bumper_cnt = 0;
		//wheel.move_forward(9, 9);
		c_rcon.resetStatus();
		ROS_INFO("%s, %d: Call Around_ChargerStation with dir = %d.", __FUNCTION__, __LINE__, around_charger_stub_dir);
		gtc_state_now_ = gtc_around_charger_station;
		around_move_cnt = 0;
	}
	else if (gtc_state_now_ == gtc_around_charger_station)
	{
		if(cliff.getStatus())
		{
			ROS_WARN("%s %d: Get cliff triggered.", __FUNCTION__, __LINE__);
			turn_angle_ = 175;
			back_distance_ = 0.01;
			gtc_state_now_ = gtc_turn_for_charger_signal_init;
			return true;
		}
		if(bumper.getStatus())
		{
			ROS_WARN("%s %d: Get bumper triggered.", __FUNCTION__, __LINE__);
			around_charger_stub_dir = 1 - around_charger_stub_dir;
			turn_angle_ = 180;
			back_distance_ = 0.01;
			if(++go_home_bumper_cnt > 1)
				gtc_state_now_ = gtc_turn_for_charger_signal_init;
			return true;
		}

		if (--around_move_cnt <= 0)
		{
			around_move_cnt = 7;
			receive_code = c_rcon.getAll();
			if(receive_code)
				no_signal_cnt = 0;
			else if(++no_signal_cnt > 60)
			{
				ROS_WARN("%s %d:No charger signal received.", __FUNCTION__, __LINE__);
				gtc_state_now_ = gtc_turn_for_charger_signal_init;
			}

			//ROS_DEBUG("%s %d Check DIR: %d, and do something", __FUNCTION__, __LINE__, around_charger_stub_dir);
			if(around_charger_stub_dir == 1)//10.30
			{
				if(receive_code&(RconFR_HomeR|RconFL_HomeR))
				{
					gtc_state_now_ = gtc_by_path_init;
					turn_angle_ = 0;
					if(receive_code&RconFR_HomeR)
						ROS_INFO("%s, %d: Detect FR-R, call By_Path().", __FUNCTION__, __LINE__);
					else if(receive_code&RconFL_HomeR)
						ROS_INFO("%s, %d: Detect FL-R, call By_Path().", __FUNCTION__, __LINE__);
				}
				else if(receive_code&RconL_HomeR)
				{
					ROS_INFO("%s, %d: Detect L-R, Check position left.", __FUNCTION__, __LINE__);
					check_position_dir = gtc_check_position_left;
					gtc_state_now_ = gtc_check_position_init;
					turn_angle_ = 0;
				}
				else if(receive_code&(RconFL_HomeL|RconFL_HomeT))
				{
					turn_angle_ = -50;
					if(receive_code&RconFL_HomeL)//FL_HL
						ROS_DEBUG("%s, %d: Detect FL-L.", __FUNCTION__, __LINE__);
					else if(receive_code&RconFL_HomeT)//FL_HT
						ROS_DEBUG("%s, %d: Detect FL-T.", __FUNCTION__, __LINE__);
				}
				else if(receive_code&RconFR_HomeT)//FR_HT
				{
					ROS_DEBUG("%s, %d: Detect FR-T.", __FUNCTION__, __LINE__);
					turn_angle_ = -80;
				}
				else if(receive_code&RconFR2_HomeT)//FR2_T
				{
					ROS_DEBUG("%s, %d: Detect FR2-T.", __FUNCTION__, __LINE__);
					turn_angle_ = -90;
				}
				else if(receive_code&RconR_HomeT)//R_HT
				{
					ROS_DEBUG("%s, %d: Detect R-T.", __FUNCTION__, __LINE__);
					turn_angle_ = -110;
					around_charger_stub_dir = 0;
				}
				else
					turn_angle_ = 0;

				if (turn_angle_ != 0)
				{
					back_distance_ = 0;
					return true;
				}
			}
			else //around_charger_stub_dir == 0
			{
				if(receive_code&(RconFL_HomeL|RconFR_HomeL))
				{
					gtc_state_now_ = gtc_by_path_init;
					turn_angle_ = 0;
					if(receive_code&(RconFL_HomeL))
						ROS_INFO("%s, %d: Detect FL-L, call By_Path().", __FUNCTION__, __LINE__);
					else if(receive_code&(RconFR_HomeL))
						ROS_INFO("%s, %d: Detect FR-L, call By_Path().", __FUNCTION__, __LINE__);
				}
				else if(receive_code&(RconR_HomeL))
				{
					ROS_INFO("%s, %d: Detect R-L, check position right.", __FUNCTION__, __LINE__);
					check_position_dir = gtc_check_position_right;
					gtc_state_now_ = gtc_check_position_init;
					turn_angle_ = 0;
				}
				else if(receive_code&(RconFR_HomeR|RconFR_HomeT))
				{
					turn_angle_ = 50;
					if(receive_code&RconFR_HomeR)
						ROS_DEBUG("%s, %d: Detect FR-R.", __FUNCTION__, __LINE__);
					else if(receive_code&RconFR_HomeT)
						ROS_DEBUG("%s, %d: Detect FR-T.", __FUNCTION__, __LINE__);
				}
				else if(receive_code&RconFL_HomeT)
				{
					ROS_DEBUG("%s, %d: Detect FL-T.", __FUNCTION__, __LINE__);
					turn_angle_ = 80;
				}
				else if(receive_code&RconFL2_HomeT)
				{
					ROS_DEBUG("%s, %d: Detect FL2-T.", __FUNCTION__, __LINE__);
					turn_angle_ = 90;
				}
				else if(receive_code&RconL_HomeT)
				{
					ROS_DEBUG("%s, %d: Detect L-T.", __FUNCTION__, __LINE__);
					turn_angle_ = 110;
					around_charger_stub_dir = 1;
				}
				else
					turn_angle_ = 0;

				if (turn_angle_ != 0)
				{
					back_distance_ = 0;
					return true;
				}
			}
		}
	}
	if (gtc_state_now_ == gtc_check_position_init)
	{
		resetGoToChargerVariables();
		gtc_state_now_ = gtc_check_position;
	}
	if (gtc_state_now_ == gtc_check_position)
	{
		if(bumper.getStatus())
		{
			ROS_WARN("%s %d: Get bumper triggered.", __FUNCTION__, __LINE__);
			around_charger_stub_dir = 1 - around_charger_stub_dir;
			if(++go_home_bumper_cnt > 1)
				gtc_state_now_ = gtc_turn_for_charger_signal_init;
			else
				gtc_state_now_ = gtc_around_charger_station_init;
			turn_angle_ = 180;
			back_distance_ = 0.01;
			return true;
		}
		if(cliff.getStatus())
		{
			ROS_WARN("%s %d: Get cliff triggered.", __FUNCTION__, __LINE__);
			gtc_state_now_ = gtc_turn_for_charger_signal_init;
			turn_angle_ = 180;
			back_distance_ = 0.01;
			return true;
		}

		if(gyro_radian_step_ < 2 * PI)
		{
			current_radian_ = getPosition().th;
			radian_offset_ = last_radian_ - current_radian_;
			ROS_DEBUG("%s %d: current_radian_ = %f, last_radian_ = %f, radian_offset_ = %f, gyro_radian_step_ = %f.",
					  __FUNCTION__, __LINE__, current_radian_, last_radian_, radian_offset_, gyro_radian_step_);
			if (check_position_dir == gtc_check_position_left && radian_offset_ < 0)
				gyro_radian_step_ += fabs(radian_offset_);
			if (check_position_dir == gtc_check_position_right && radian_offset_ > 0)
				gyro_radian_step_ += radian_offset_;
			last_radian_ = current_radian_;

			receive_code = (c_rcon.getAll()&RconFrontAll_Home_LR);
			ROS_DEBUG("%s %d: Check_Position receive_code == %8x, R... == %8x.", __FUNCTION__, __LINE__,
					  receive_code, RconFrontAll_Home_LR);
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

			if(receive_code & (RconFR_HomeL|RconFR_HomeR) && check_position_dir == gtc_check_position_left)
				gtc_state_now_ = gtc_by_path_init;
			if(receive_code & (RconFL_HomeL|RconFL_HomeR) && check_position_dir == gtc_check_position_right)
				gtc_state_now_ = gtc_by_path_init;
		}
		if(gyro_radian_step_ >= 2 * PI)
		{
			ROS_INFO("%s, %d: Robot can't see charger, restart go to charger process.", __FUNCTION__, __LINE__);
			turn_angle_ = 100;
			back_distance_ = 0;
			gtc_state_now_ = gtc_turn_for_charger_signal_init;
			return true;
		}
	}
	if (gtc_state_now_ == gtc_by_path_init)
	{
		resetGoToChargerVariables();
		gtc_state_now_ = gtc_by_path;
	}
	if (gtc_state_now_ == gtc_by_path)
	{
		if(bumper.getStatus())
		{
			ROS_INFO("bumper in by path!");
			if(!position_far)
			{
				gtc_state_now_ = gtc_turn_for_charger;
				turn_connect_dir = gtc_check_position_right;
				turn_connect_cnt = 0;
			}
			else
			{
				//if((c_rcon.getAll()&RconFront_Home_LR) == 0)
				//	gtc_state_now_ = gtc_turn_for_charger_signal_init;
				gtc_state_now_ = gtc_turn_for_charger_signal_init;
				back_distance_ = 0.1;
				//if(ev.bumper_triggered & BLOCK_LEFT)
				//	turn_angle_ = -1100;
				//else
				//	turn_angle_ = 1100;
				ROS_WARN("%d: quick_back in position_far", __LINE__);
				return true;
			}
		}
		if(cliff.getStatus())
		{
			turn_angle_ = 175;
			back_distance_ = 0.01;
			gtc_state_now_ = gtc_turn_for_charger_signal_init;
			return true;
		}

		if (--by_path_move_cnt < 0)
		{
			by_path_move_cnt = 25;
			receive_code = c_rcon.getAll();
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
						gtc_state_now_ = gtc_turn_for_charger_signal_init;
						back_distance_ = 0;
						turn_angle_ = 0;
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
					back_distance_ = 0;
					if(position_far)
					{
						switch(temp_code)
						{
							case (RconFR2_HomeL):
								ROS_DEBUG("%s, %d: position_far, FR2_L.", __FUNCTION__, __LINE__);
								turn_angle_ = -25;
								return true;
							case (RconFR2_HomeL|RconFR2_HomeR):
								ROS_DEBUG("%s, %d: position_far, FR2_L/FR2_R.", __FUNCTION__, __LINE__);
								turn_angle_ = -35;
								return true;
							case (RconR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
								ROS_DEBUG("%s, %d: position_far, R_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
								turn_angle_ = -25;
								return true;
							case (RconR_HomeR|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: position_far, R_R/FR2_L.", __FUNCTION__, __LINE__);
								turn_angle_ = -40;
								return true;
							case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
								ROS_DEBUG("%s, %d: position_far, R_R/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
								turn_angle_ = -25;
								return true;
							case (RconR_HomeR|RconR_HomeL|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: position_far, R_R/R_L/FR2_L.", __FUNCTION__, __LINE__);
								turn_angle_ = -45;
								return true;
							case (RconR_HomeR|RconR_HomeL):
								ROS_DEBUG("%s, %d: position_far, R_R/R_L.", __FUNCTION__, __LINE__);
								turn_angle_ = -25;
								return true;
							case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: position_far, R_R/FR_L/FR2_L.", __FUNCTION__, __LINE__);
								turn_angle_ = -55;
								return true;
							case (RconR_HomeL|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: position_far, R_L/FR2_L.", __FUNCTION__, __LINE__);
								turn_angle_ = -50;
								return true;
							case (RconFL2_HomeR):
								ROS_DEBUG("%s, %d: position_far, FL2_R.", __FUNCTION__, __LINE__);
								turn_angle_ = 25;
								return true;
							case (RconFL2_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: position_far, FL2_/FL2_R.", __FUNCTION__, __LINE__);
								turn_angle_ = 35;
								return true;
							case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_L/FL2_L/FL2_R.", __FUNCTION__, __LINE__);
								turn_angle_ = 25;
								return true;
							case (RconL_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_L/FL2_R.", __FUNCTION__, __LINE__);
								turn_angle_ = 40;
								return true;
							case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_L/FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
								turn_angle_ = 25;
								return true;
							case (RconL_HomeR|RconL_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_R/L_L/FL2_R.", __FUNCTION__, __LINE__);
								turn_angle_ = 45;
								return true;
							case (RconL_HomeR|RconL_HomeL):
								ROS_DEBUG("%s, %d: position_far, L_R/L_L.", __FUNCTION__, __LINE__);
								turn_angle_ = 50;
								return true;
							case (RconL_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_R.", __FUNCTION__, __LINE__);
								turn_angle_ = 55;
								return true;
							case (RconL_HomeL|RconFL2_HomeR|RconFL_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
								turn_angle_ = 25;
								return true;
							case (RconL_HomeR|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: position_far, L_R/FL2_R.", __FUNCTION__, __LINE__);
								turn_angle_ = 50;
								return true;
						}
					}
					else
					{
						switch(temp_code)
						{
							case (RconFR2_HomeL):
								ROS_DEBUG("%s, %d: !position_far, FR2_L.", __FUNCTION__, __LINE__);
								turn_angle_ = -25;
								return true;
							case (RconFR2_HomeL|RconFR2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, FR2_L/FR2_R.", __FUNCTION__, __LINE__);
								turn_angle_ = -35;
								return true;
							case (RconR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, R_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
								turn_angle_ = -25;
								return true;
							case (RconR_HomeR|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: !position_far, R_R/FR2_L.", __FUNCTION__, __LINE__);
								turn_angle_ = -40;
								return true;
							case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, R_R/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
								turn_angle_ = -25;
								return true;
							case (RconR_HomeR|RconR_HomeL|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: !position_far, R_R/R_L/FR2_L.", __FUNCTION__, __LINE__);
								turn_angle_ = -45;
								return true;
							case (RconR_HomeR|RconR_HomeL):
								ROS_DEBUG("%s, %d: !position_far, R_R/R_L.", __FUNCTION__, __LINE__);
								turn_angle_ = -50;
								return true;
							case (RconR_HomeL):
								ROS_DEBUG("%s, %d: !position_far, R_L.", __FUNCTION__, __LINE__);
								turn_angle_ = -55;
								return true;
							case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: !position_far, R_R/FR_L/FR2_L.", __FUNCTION__, __LINE__);
								turn_angle_ = -25;
								return true;
							case (RconR_HomeL|RconFR2_HomeL):
								ROS_DEBUG("%s, %d: !position_far, R_L/FR2_L.", __FUNCTION__, __LINE__);
								turn_angle_ = -50;
								return true;
							case (RconFL2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, FL2_R.", __FUNCTION__, __LINE__);
								turn_angle_ = 25;
								return true;
							case (RconFL2_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, FL2_/FL2_R.", __FUNCTION__, __LINE__);
								turn_angle_ = 35;
								return true;
							case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_L/FL2_L/FL2_R.", __FUNCTION__, __LINE__);
								turn_angle_ = 25;
								return true;
							case (RconL_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_L/FL2_R.", __FUNCTION__, __LINE__);
								turn_angle_ = 40;
								return true;
							case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_L/FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
								turn_angle_ = 25;
								return true;
							case (RconL_HomeR|RconL_HomeL|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_R/L_L/FL2_R.", __FUNCTION__, __LINE__);
								turn_angle_ = 45;
								return true;
							case (RconL_HomeR|RconL_HomeL):
								ROS_DEBUG("%s, %d: !position_far, L_R/L_L.", __FUNCTION__, __LINE__);
								turn_angle_ = 50;
								return true;
							case (RconL_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_R.", __FUNCTION__, __LINE__);
								turn_angle_ = 25;
								return true;
							case (RconL_HomeL|RconFL2_HomeR|RconFL_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
								turn_angle_ = 25;
								return true;
							case (RconL_HomeR|RconFL2_HomeR):
								ROS_DEBUG("%s, %d: !position_far, L_R/FL2_R.", __FUNCTION__, __LINE__);
								turn_angle_ = 50;
								return true;
						}
					}
				}
				no_signal_cnt = 0;
			}
			else
			{
				near_counter = 0;
				if(++no_signal_cnt > 1)
				{
					ROS_INFO("%s %d: No signal in by path, switch to check position.", __FUNCTION__, __LINE__);
					check_position_dir = gtc_check_position_left;
					gtc_state_now_ = gtc_check_position_init;
				}
			}
		}
	}
	if (gtc_state_now_ == gtc_turn_for_charger)
	{
		if (turn_connect_dir == gtc_check_position_right && ++turn_connect_cnt > 50)
		{
			turn_connect_dir = gtc_check_position_left;
			turn_connect_cnt = 0;
		}
		if (turn_connect_dir == gtc_check_position_left && ++turn_connect_cnt > 50)
		{
			turn_connect_cnt = 0;
			back_distance_ = 0.3;
			ROS_WARN("%s %d: Turn connect failed, move back for 0.3m.", __FUNCTION__, __LINE__);
			turn_angle_ = 0;
			gtc_state_now_ = gtc_turn_for_charger_signal_init;
			return true;
		}
	}

	return false;
}

bool MovementGoToCharger::_isStop()
{
	if (gtc_state_now_ == gtc_turn_for_charger_signal && gyro_radian_step_ >= 2 * PI)
	{
		ROS_WARN("%s %d: Turn for charger signal failed, no charger signal received.", __FUNCTION__, __LINE__);
		return true;
	}
	return false;
}

void MovementGoToCharger::getTurnBackInfo(double &turn_radian, float &back_distance)
{
	turn_radian = degree_to_radian(turn_angle_);
	back_distance = back_distance_;
}

void MovementGoToCharger::adjustSpeed(int32_t &l_speed, int32_t &r_speed)
{
	/*---check if near charger station---*/
	if (gtc_state_now_ == gtc_check_near_charger_station)
	{
		wheel.setDirectionForward();
		l_speed = r_speed = 0;
	}
	else if (gtc_state_now_ == gtc_away_from_charger_station)
	{
		wheel.setDirectionBackward();
		l_speed = r_speed = 20;
	}
	else if (gtc_state_now_ == gtc_turn_for_charger_signal)
	{
		wheel.setDirectionRight();
		l_speed = r_speed = 10;
	}
	else if (gtc_state_now_ == gtc_around_charger_station_init)
	{
		wheel.setDirectionForward();
		l_speed = r_speed = 9;
		around_move_cnt = 0;
	}
	else if (gtc_state_now_ == gtc_around_charger_station)
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
	else if (gtc_state_now_ == gtc_check_position)
	{
		ROS_DEBUG("%s, %d: Check position dir: %d.", __FUNCTION__, __LINE__, check_position_dir);
		if(check_position_dir == gtc_check_position_left)
			wheel.setDirectionLeft();
		else if(check_position_dir == gtc_check_position_right)
			wheel.setDirectionRight();
		l_speed = r_speed = 10;
	}
	else if (gtc_state_now_ == gtc_by_path)
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
	else if (gtc_state_now_ == gtc_turn_for_charger)
	{
		if (turn_connect_dir == gtc_check_position_right)
			wheel.setDirectionRight();
		else if (turn_connect_dir == gtc_check_position_left)
			wheel.setDirectionLeft();
		l_speed = r_speed = 5;
	}
	left_speed_ = l_speed;
	right_speed_ = r_speed;
}

bool MovementGoToCharger::isFinish()
{
	return charger.getChargeStatus() || _isStop();
}
