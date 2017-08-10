#include <ros/ros.h>
#include <ros/console.h>
#include <stdio.h>
#include <wav.h>

#include "movement.h"
#include "config.h"
#include "go_home.hpp"
#include "robot.hpp"
#include "gyro.h"
#include "random_runing.h"
#include "core_move.h"
#include "event_manager.h"
#include "charger.hpp"
#include "map.h"

extern float saved_pos_x, saved_pos_y;
/*----------------------------------------------------------------GO Home  ----------------*/
#define GO_HOME_INIT 0
#define CHECK_NEAR_CHARGER_STATION 1
#define AWAY_FROM_CHARGER_STATION 2
#define GO_HOME 3
#define AROUND_CHARGER_STATION_INIT 4
#define AROUND_CHARGER_STATION 5
#define CHECK_POSITION_INIT 6
#define CHECK_POSITION 7
#define BY_PATH_INIT 8
#define BY_PATH 9
int8_t g_go_home_state_now = GO_HOME_INIT;
bool g_bumper_left = false, g_bumper_right = false;
bool g_go_to_charger_failed = false;
bool during_cleaning = false;
bool should_check_near_home = true;
uint8_t counter_for_move_away = 0;

void go_home(void)
{
	g_go_to_charger_failed = false;
	should_check_near_home = true;

	set_led_mode(LED_STEADY, LED_ORANGE);
	if (get_clean_mode() == Clean_Mode_GoHome)
	{
		during_cleaning = false;
		if (!is_gyro_on())
		{
			// Restart the gyro.
			set_gyro_off();
			// Wait for 30ms to make sure the off command has been effectived.
			usleep(30000);
			// Set gyro on before wav_play can save the time for opening the gyro.
			set_gyro_on();
			wav_play(WAV_BACK_TO_CHARGER);

			if (!wait_for_gyro_on())
			{
				set_clean_mode(Clean_Mode_Userinterface);
				return;
			}
		}
		else
		{
			wav_play(WAV_BACK_TO_CHARGER);
		}
	}
	else
	{
		ROS_INFO("%s %d: set during_cleaning.", __FUNCTION__, __LINE__);
		during_cleaning = true;
	}

	if (!during_cleaning)
		event_manager_reset_status();

	go_home_register_events();

	while (ros::ok())
	{
		if (g_fatal_quit_event || g_key_clean_pressed || g_go_to_charger_failed)
		{
			if(!during_cleaning && g_cliff_all_triggered)
			{
				disable_motors();
				wav_play(WAV_ERROR_LIFT_UP);
				g_key_clean_pressed = false;
			}
			set_clean_mode(Clean_Mode_Userinterface);
			break;
		}
		if(g_charge_detect)
		{
			if(!during_cleaning)
				set_clean_mode(Clean_Mode_Charging);
			disable_motors();
			break;
		}
		if(g_battery_low)
		{
			set_clean_mode(Clean_Mode_Sleep);
			break;
		}

		go_to_charger();

		if(cm_should_self_check())
			cm_self_check();

	}

	if(!during_cleaning && !g_charge_detect && !g_fatal_quit_event && !g_key_clean_pressed)
	{
		disable_motors();
		wav_play(WAV_BACK_TO_CHARGER_FAILED);
	}

	go_home_unregister_events();
}

void go_to_charger(void)
{
	/*	meaning of entrance_to_check_position	*
	 *	0: around_chargestation					*
	 *	1: by_path								*/
	uint8_t entrance_to_check_position = 0;
	uint32_t receive_code = 0;

	uint8_t around_charger_stub_dir = 0;
#define ROUND_LEFT			0x01
#define ROUND_RIGHT			0x02
	uint8_t check_position_dir = 0;
	bool position_far = true;
	float target_distance = 0;
	/*---variable for around_chargestation---*/
	uint32_t no_signal_counter=0;
	uint8_t cliff_counter = 0;
	/*---variable for by_path---*/
	uint32_t temp_code =0 ;
	uint16_t nosignal_counter=0;
	uint8_t temp_check_position=0;
	uint8_t near_counter=0;
	uint8_t side_counter=0;
	bool eh_status_now=false, eh_status_last=false;
	/*---variable for turn_connect---*/
	int8_t tc_speed = 5;

	reset_rcon_status();
	// This is for calculating the robot turning.
	int16_t go_home_target_angle = 0;
	float current_angle;
	float last_angle;
	float angle_offset;
	uint8_t turn_finished = true;
	// This step is for counting angle change when the robot turns.
	float gyro_step = 0;

	uint8_t go_home_bumper_counter = 0;
	g_go_home_state_now = GO_HOME_INIT;
	g_bumper_left = false;
	g_bumper_right = false;

	g_move_back_finished = true;
	set_side_brush_pwm(30, 30);
	set_main_brush_pwm(30);
	set_bldc_speed(Vac_Speed_NormalL);

	stop_brifly();
	reset_rcon_status();
	// Save the start angle.
	last_angle = robot::instance()->getAngle();
	// Enable the charge function
	set_start_charge();

	while(ros::ok())
	{
		if(event_manager_check_event(&eh_status_now, &eh_status_last) == 1)
			continue;

		if(g_fatal_quit_event)
			break;
		if(g_charge_detect && g_go_home_state_now != CHECK_POSITION)
			break;
		if(g_key_clean_pressed)
			break;
		if(g_battery_low)
			break;
		if (g_slam_error)
		{
			set_wheel_speed(0, 0);
			continue;
		}
		if(cm_should_self_check())
			break;

		if(during_cleaning)
			map_set_cleaned(map_get_curr_cell());

		if(!g_move_back_finished)
		{
			if (!go_home_check_move_back_finish(target_distance))
				continue;
			ROS_WARN("%s %d: Move back finish.", __FUNCTION__, __LINE__);
			g_move_back_finished = true;
			reset_rcon_status();
		}

		if(!turn_finished)
		{
			if (!go_home_check_turn_finish(go_home_target_angle))
			{
				if(g_cliff_triggered)
				{
					ROS_INFO("%s, %d: cliff while turning!", __FUNCTION__, __LINE__);
					target_distance = 0.1;
					g_cliff_cnt++;
					g_move_back_finished = false;
					saved_pos_x = robot::instance()->getOdomPositionX();
					saved_pos_y = robot::instance()->getOdomPositionY();
				}
				continue;
			}

			ROS_WARN("%s %d: Turn finish.", __FUNCTION__, __LINE__);
			turn_finished = true;
			reset_rcon_status();
		}

		/*---go_home initial---*/
		if(g_go_home_state_now == GO_HOME_INIT)
		{
			ROS_INFO("%s %d: Start go to charger.", __FUNCTION__, __LINE__);
			entrance_to_check_position = 0;
			receive_code = 0;
			no_signal_counter=0;
			go_home_bumper_counter=0;
			cliff_counter = 0;
			temp_code =0 ;
			nosignal_counter=0;
			temp_check_position=0;
			near_counter=0;
			side_counter=0;
			eh_status_now=false, eh_status_last=false;
			tc_speed = 5;

			turn_finished = true;
			reset_rcon_status();
			gyro_step = 0;

			g_move_back_finished = true;

			// Save the start angle.
			last_angle = robot::instance()->getAngle();
			// Enable the charge function
			set_start_charge();

			if(should_check_near_home)
			{
				should_check_near_home = false;
				g_go_home_state_now = CHECK_NEAR_CHARGER_STATION;
			}
			else
			{
				g_go_home_state_now = GO_HOME;
			}
		}
		/*---check if near charger station---*/
		else if(g_go_home_state_now == CHECK_NEAR_CHARGER_STATION)
		{
			if(nosignal_counter < 10)
			{
				receive_code = get_rcon_status();
				ROS_INFO("%s, %d: check near home, receive_code: %8x", __FUNCTION__, __LINE__, receive_code);
				if(receive_code&RconAll_Home_T)
				{
					reset_rcon_status();
					ROS_INFO("receive LR");
					if(receive_code&(RconFL_HomeT|RconFR_HomeT|RconFL2_HomeT|RconFR2_HomeT))
					{
						ROS_INFO("turn 180");
						target_distance = 0.1;
						g_move_back_finished = false;

						go_home_target_angle = ranged_angle(gyro_get_angle() + 1800);
						turn_finished = false;
					}
					else if(receive_code&RconR_HomeT)
					{
						ROS_INFO("turn left 90");
						target_distance = 0.1;
						g_move_back_finished = false;

						go_home_target_angle = ranged_angle(gyro_get_angle() + 900);
						turn_finished = false;
					}
					else if(receive_code&RconL_HomeT)
					{
						ROS_INFO("turn right 90");
						target_distance = 0.1;
						g_move_back_finished = false;

						go_home_target_angle = ranged_angle(gyro_get_angle() - 900);
						turn_finished = false;
					}
					saved_pos_x = robot::instance()->getOdomPositionX();
					saved_pos_y = robot::instance()->getOdomPositionY();
					g_go_home_state_now = AWAY_FROM_CHARGER_STATION;
					nosignal_counter = 0;
					continue;
				}
				else
				{
					nosignal_counter++;
				}
			}
			else
			{
				g_go_home_state_now = GO_HOME;
				nosignal_counter = 0;
			}
		}
		/*---move forward to keep away from charger station---*/
		else if(g_go_home_state_now == AWAY_FROM_CHARGER_STATION)
		{
			move_forward(30, 30);
			if(g_bumper_left || g_bumper_right)
			{
				stop_brifly();
				ROS_WARN("%s %d: Get bumper trigered.", __FUNCTION__, __LINE__);
				target_distance = 0.03;
				g_move_back_finished = false;
				g_go_home_state_now = GO_HOME;
				continue;
			}
			if(g_cliff_triggered)
			{
				stop_brifly();
				ROS_WARN("%s %d: Get cliff trigered.", __FUNCTION__, __LINE__);
				g_cliff_cnt++;
				target_distance = 0.03;
				g_move_back_finished = false;
				g_go_home_state_now = GO_HOME_INIT;
				counter_for_move_away = 0;
				continue;
			}
			if(counter_for_move_away++ > 20)
			{
				g_go_home_state_now = GO_HOME;
				counter_for_move_away = 0;
			}
		}
		/*---go_home main while---*/
		else if(g_go_home_state_now == GO_HOME)
		{
			if(gyro_step < 360)
			{
				if(g_bumper_left || g_bumper_right)
				{
					target_distance = 0.03;
					g_move_back_finished = false;
					continue;
				}
				if(g_cliff_triggered)
				{
					ROS_WARN("%s %d: Get cliff trigered.", __FUNCTION__, __LINE__);
					g_cliff_cnt++;
					target_distance = 0.03;
					g_move_back_finished = false;
					g_go_home_state_now = GO_HOME_INIT;
					continue;
				}

				current_angle = robot::instance()->getAngle();
				angle_offset = ranged_angle(current_angle - last_angle);
				ROS_DEBUG("Current_Angle = %f, Last_Angle = %f, Angle_Offset = %f, Gyro_Step = %f.", current_angle, last_angle, angle_offset, gyro_step);
				if (angle_offset < 0)
					gyro_step += (-angle_offset);
				last_angle = current_angle;

				set_dir_right();
				set_wheel_speed(10, 10);

				receive_code = get_rcon_status();
				reset_rcon_status();
				if(receive_code&RconL_HomeL)// L  H_L
				{
					ROS_INFO("Start with L-L.");
					around_charger_stub_dir = 1;
					g_go_home_state_now = AROUND_CHARGER_STATION_INIT;
					continue;
				}
				if(receive_code&RconR_HomeR)// R  H_R
				{
					ROS_INFO("Start with R-R.");
					around_charger_stub_dir = 0;
					g_go_home_state_now = AROUND_CHARGER_STATION_INIT;
					continue;
				}

				if(receive_code&RconFL_HomeR)//FL H_R
				{
					ROS_INFO("Start with FL-R.");
					go_home_target_angle = ranged_angle(gyro_get_angle() + 900);
					turn_finished = false;
					around_charger_stub_dir = 0;
				}
				else if(receive_code&RconFR_HomeL)//FR H_L
				{
					ROS_INFO("Start with FR-L.");
					go_home_target_angle = ranged_angle(gyro_get_angle() - 900);
					turn_finished = false;
					around_charger_stub_dir = 1;
				}
				else if(receive_code&RconFL_HomeL)//FL H_L
				{
					ROS_INFO("Start with FL-L.");
					go_home_target_angle = ranged_angle(gyro_get_angle() - 900);
					turn_finished = false;
					around_charger_stub_dir = 1;
				}
				else if(receive_code&RconFR_HomeR)//FR H_R
				{
					ROS_INFO("Start with FR-R.");
					go_home_target_angle = ranged_angle(gyro_get_angle() + 900);
					turn_finished = false;
					around_charger_stub_dir = 0;
				}
				else if(receive_code&RconFL2_HomeR)//FL2 H_R
				{
					ROS_INFO("Start with FL2-R.");
					go_home_target_angle = ranged_angle(gyro_get_angle() + 850);
					turn_finished = false;
					around_charger_stub_dir = 0;
				}
				else if(receive_code&RconFR2_HomeL)//FR2 H_L
				{
					ROS_INFO("Start with FR2-L.");
					go_home_target_angle = ranged_angle(gyro_get_angle() - 850);
					turn_finished = false;
					around_charger_stub_dir = 1;
				}
				else if(receive_code&RconFL2_HomeL)//FL2 H_L
				{
					ROS_INFO("Start with FL2-L.");
					go_home_target_angle = ranged_angle(gyro_get_angle() - 600);
					turn_finished = false;
					around_charger_stub_dir = 1;
				}
				else if(receive_code&RconFR2_HomeR)//FR2 H_R
				{
					ROS_INFO("Start with FR2-R.");
					go_home_target_angle = ranged_angle(gyro_get_angle() + 600);
					turn_finished = false;
					around_charger_stub_dir = 0;
				}
				else if(receive_code&RconL_HomeR)// L  H_R
				{
					ROS_INFO("Start with L-R.");
					go_home_target_angle = ranged_angle(gyro_get_angle() + 1500);
					turn_finished = false;
					around_charger_stub_dir = 0;
				}
				else if(receive_code&RconR_HomeL)// R  H_L
				{
					ROS_INFO("Start with R-L.");
					go_home_target_angle = ranged_angle(gyro_get_angle() - 1500);
					turn_finished = false;
					around_charger_stub_dir = 1;
				}
		/*--------------------------HomeT-----------------*/
				else if(receive_code&RconFL_HomeT)//FL H_T
				{
					ROS_INFO("Start with FL-T.");
					go_home_target_angle = ranged_angle(gyro_get_angle() - 600);
					turn_finished = false;
					around_charger_stub_dir = 1;
				}
				else if(receive_code&RconFR_HomeT)//FR H_T
				{
					ROS_INFO("Start with FR-T.");
					go_home_target_angle = ranged_angle(gyro_get_angle() - 800);
					turn_finished = false;
					around_charger_stub_dir = 1;
				}
				else if(receive_code&RconFL2_HomeT)//FL2 H_T
				{
					ROS_INFO("Start with FL2-T.");
					go_home_target_angle = ranged_angle(gyro_get_angle() - 600);
					turn_finished = false;
					around_charger_stub_dir = 1;
				}
				else if(receive_code&RconFR2_HomeT)//FR2 H_T
				{
					ROS_INFO("Start with FR2-T.");
					go_home_target_angle = ranged_angle(gyro_get_angle() - 800);
					turn_finished = false;
					around_charger_stub_dir = 1;
				}
				else if(receive_code&RconL_HomeT)// L  H_T
				{
					ROS_INFO("Start with L-T.");
					go_home_target_angle = ranged_angle(gyro_get_angle() - 1200);
					turn_finished = false;
					around_charger_stub_dir = 1;
				}
				else if(receive_code&RconR_HomeT)// R  H_T
				{
					ROS_INFO("Start with R-T.");
					go_home_target_angle = ranged_angle(gyro_get_angle() - 1200);
					turn_finished = false;
					around_charger_stub_dir = 1;
				}
		/*--------------BL BR---------------------*/
				else if((receive_code&RconBL_HomeL))//BL H_L
				{
					ROS_INFO("Start with BL-L.");
					go_home_target_angle = ranged_angle(gyro_get_angle() + 800);
					turn_finished = false;
					around_charger_stub_dir = 1;
				}
				else if((receive_code&RconBR_HomeR))//BR H_L R  //OK
				{
					ROS_INFO("Start with BR-R.");
					go_home_target_angle = ranged_angle(gyro_get_angle() - 800);
					turn_finished = false;
					around_charger_stub_dir = 0;
				}
				else if((receive_code&RconBL_HomeR))//BL H_R
				{
					ROS_INFO("Start with BL-R.");
					go_home_target_angle = ranged_angle(gyro_get_angle() + 800);
					turn_finished = false;
					around_charger_stub_dir = 1;
				}
				else if((receive_code&RconBR_HomeL))//BL H_L R
				{
					ROS_INFO("Start with BR-L.");
					go_home_target_angle = ranged_angle(gyro_get_angle() - 800);
					turn_finished = false;
					around_charger_stub_dir = 0;
				}
				else if((receive_code&RconBL_HomeT))//BL H_T
				{
					ROS_INFO("Start with BL-T.");
					go_home_target_angle = ranged_angle(gyro_get_angle() + 300);
					turn_finished = false;
					around_charger_stub_dir = 1;
				}
				else if((receive_code&RconBR_HomeT))//BR H_T
				{
					ROS_INFO("Start with BR-T.");
					go_home_target_angle = ranged_angle(gyro_get_angle() - 300);
					turn_finished = false;
					around_charger_stub_dir = 0;
				}

				if (!turn_finished)
				{
					g_go_home_state_now = AROUND_CHARGER_STATION_INIT;
					continue;
				}
			}
			else
			{
				g_go_to_charger_failed = true;
				break;
			}
		}
		/*-----around_chargestation init-----*/
		else if(g_go_home_state_now == AROUND_CHARGER_STATION_INIT)
		{
			go_home_bumper_counter = 0;
			move_forward(9, 9);
			reset_rcon_status();
			ROS_INFO("%s, %d: Call Around_ChargerStation with dir = %d.", __FUNCTION__, __LINE__, around_charger_stub_dir);
			g_go_home_state_now = AROUND_CHARGER_STATION;
		}
		/*-------around_chargestation main while-------*/
		else if(g_go_home_state_now == AROUND_CHARGER_STATION)
		{
			if(g_cliff_triggered)
			{
				ROS_WARN("%s %d: Get cliff trigered.", __FUNCTION__, __LINE__);
				g_cliff_cnt++;
				target_distance = 0.03;
				g_move_back_finished = false;
				go_home_target_angle = ranged_angle(gyro_get_angle() + 1750);
				turn_finished = false;
				g_go_home_state_now = GO_HOME_INIT;
				continue;
			}
			if(g_bumper_left || g_bumper_right)
			{
				ROS_WARN("%s %d: Get bumper trigered.", __FUNCTION__, __LINE__);
				around_charger_stub_dir = 1 - around_charger_stub_dir;
				if(++go_home_bumper_counter > 1)
					g_go_home_state_now = GO_HOME_INIT;
				target_distance = 0.03;
				g_move_back_finished = false;
				go_home_target_angle = ranged_angle(gyro_get_angle() + 1800);
				ROS_WARN("%s %d: Set target angle:%d.", __FUNCTION__, __LINE__, go_home_target_angle);
				turn_finished = false;
				continue;
			}
			receive_code = get_rcon_status();
			if(receive_code)
			{
				no_signal_counter=0;
				reset_rcon_status();
			}
			else
			{
				no_signal_counter++;
				if(no_signal_counter>200)
				{
					ROS_WARN("No charger signal received.");
					g_go_home_state_now = GO_HOME_INIT;
					continue;
				}
			}

			ROS_DEBUG("%s %d Check DIR: %d, and do something", __FUNCTION__, __LINE__, around_charger_stub_dir);
			if(around_charger_stub_dir == 1)//10.30
			{
				if(receive_code&(RconFR_HomeR))
				{
					ROS_INFO("%s, %d: Detect FR-R, call By_Path().", __FUNCTION__, __LINE__);
					g_go_home_state_now = BY_PATH_INIT;
					continue;
				}
				if(receive_code&(RconFL_HomeR))
				{
					ROS_INFO("%s, %d: Detect FL-R, call By_Path().", __FUNCTION__, __LINE__);
					g_go_home_state_now = BY_PATH_INIT;
					continue;
				}
				if(receive_code&(RconL_HomeR))
				{
					ROS_INFO("%s, %d: Detect L-R, Check position.", __FUNCTION__, __LINE__);
					check_position_dir = ROUND_LEFT;
					g_go_home_state_now = CHECK_POSITION_INIT;
					continue;
				}

				if(receive_code&RconFL_HomeL)	//FR_HL
				{
					ROS_DEBUG("%s, %d: Detect FL-L.", __FUNCTION__, __LINE__);
					turn_finished = false;
					go_home_target_angle = ranged_angle(gyro_get_angle() - 500);
				}
				else if(receive_code&RconFL_HomeR)	 //FR_HL
				{
					ROS_DEBUG("%s, %d: Detect FL-R.", __FUNCTION__, __LINE__);
					turn_finished = false;
					go_home_target_angle = ranged_angle(gyro_get_angle() + 600);
				}
				else if(receive_code&RconFL_HomeT)	 //FR_HT
				{
					ROS_DEBUG("%s, %d: Detect FL-T.", __FUNCTION__, __LINE__);
					turn_finished = false;
					go_home_target_angle = ranged_angle(gyro_get_angle() - 500);
				}
				else if(receive_code&RconFR_HomeT)	 //R_HT
				{
					ROS_DEBUG("%s, %d: Detect FR-T.", __FUNCTION__, __LINE__);
					turn_finished = false;
					go_home_target_angle = ranged_angle(gyro_get_angle() - 800);
				}
				else if(receive_code&RconFR2_HomeT) //FR2_T //OK
				{
					ROS_DEBUG("%s, %d: Detect FR2-T.", __FUNCTION__, __LINE__);
					turn_finished = false;
					go_home_target_angle = ranged_angle(gyro_get_angle() - 900);
				}
				else if(receive_code&RconR_HomeT)  //OK
				{
					ROS_DEBUG("%s, %d: Detect R-T.", __FUNCTION__, __LINE__);
					turn_finished = false;
					go_home_target_angle = ranged_angle(gyro_get_angle() - 1100);
					around_charger_stub_dir = 0;
				}
				if (!turn_finished)
					continue;

				if(receive_code&RconL_HomeT)  //L_T
				{
					ROS_DEBUG("%s, %d: Detect L-T.", __FUNCTION__, __LINE__);
					move_forward(22, 12);
					usleep(100000);
				}
				else if(receive_code&RconL_HomeL)  //L_L  9 18
				{
					ROS_DEBUG("%s, %d: Detect L-L.", __FUNCTION__, __LINE__);
					move_forward(22, 12);
					usleep(100000);
				}
				else if(receive_code&RconL_HomeR)  //L_R  9 18
				{
					ROS_DEBUG("%s, %d: Detect L-R.", __FUNCTION__, __LINE__);
					move_forward(25, 14);
					usleep(100000);
				}
				else if(receive_code&RconFL2_HomeT)  //FL2_T
				{
					ROS_DEBUG("%s, %d: Detect FL2-T.", __FUNCTION__, __LINE__);
					move_forward(23, 14);
					usleep(100000);
				}
				else if(receive_code&RconFL2_HomeL)  //FL_HL
				{
					ROS_DEBUG("%s, %d: Detect FL2-L.", __FUNCTION__, __LINE__);
					move_forward(20, 14);
					usleep(100000);
				}
				else if(receive_code&RconFL2_HomeR)//FL2_HR
				{
					ROS_DEBUG("%s, %d: Detect FL2-R.", __FUNCTION__, __LINE__);
					move_forward(15, 21);
					usleep(100000);
				}
				else
				{
					ROS_DEBUG("%s, %d: Else.", __FUNCTION__, __LINE__);
					move_forward(14, 21);  //0K (16,35)	  1100
					usleep(100000);
				}
			}
			else //around_charger_stub_dir == 0
			{
				if(receive_code&(RconFL_HomeL))
				{
					ROS_INFO("%s, %d: Detect FL-L, call By_Path().", __FUNCTION__, __LINE__);
					g_go_home_state_now = BY_PATH_INIT;
					continue;
				}
				if(receive_code&(RconFR_HomeL))
				{
					ROS_INFO("%s, %d: Detect FR-L, call By_Path().", __FUNCTION__, __LINE__);
					g_go_home_state_now = BY_PATH_INIT;
					continue;
				}
				if((receive_code&(RconR_HomeL)))
				{
					ROS_INFO("%s, %d: Detect R-L, check position.", __FUNCTION__, __LINE__);
					check_position_dir = ROUND_RIGHT;
					g_go_home_state_now = CHECK_POSITION_INIT;
					continue;
				}

				if(receive_code&RconFR_HomeR)	//OK
				{
					ROS_DEBUG("%s, %d: Detect FR-R.", __FUNCTION__, __LINE__);
					turn_finished = false;
					go_home_target_angle = ranged_angle(gyro_get_angle() + 500);
				}
				else if(receive_code&RconFR_HomeL)	//OK
				{
					ROS_DEBUG("%s, %d: Detect FR-L.", __FUNCTION__, __LINE__);
					turn_finished = false;
					go_home_target_angle = ranged_angle(gyro_get_angle() - 600);
				}
				else if(receive_code&RconFR_HomeT)	//ok
				{
					ROS_DEBUG("%s, %d: Detect FR-T.", __FUNCTION__, __LINE__);
					turn_finished = false;
					go_home_target_angle = ranged_angle(gyro_get_angle() + 500);
				}
				else if(receive_code&RconFL_HomeT)	//OK
				{
					ROS_DEBUG("%s, %d: Detect FL-T.", __FUNCTION__, __LINE__);
					turn_finished = false;
					go_home_target_angle = ranged_angle(gyro_get_angle() + 800);
				}
				else if(receive_code&RconFL2_HomeT)  //OK
				{
					ROS_DEBUG("%s, %d: Detect FL2-T.", __FUNCTION__, __LINE__);
					turn_finished = false;
					go_home_target_angle = ranged_angle(gyro_get_angle() + 900);
				}
				else if(receive_code&RconL_HomeT)  //OK
				{
					ROS_DEBUG("%s, %d: Detect L-T.", __FUNCTION__, __LINE__);
					turn_finished = false;
					go_home_target_angle = ranged_angle(gyro_get_angle() + 1100);
					around_charger_stub_dir = 1;
				}
				if (!turn_finished)
					continue;

				if(receive_code&RconR_HomeT)   // OK ,(10,26)
				{
					ROS_DEBUG("%s %d Detect R-T.", __FUNCTION__, __LINE__);
					move_forward(12, 22);
					usleep(100000);
				}
				else if(receive_code&RconR_HomeR)  //ok 18 13
				{
					ROS_DEBUG("%s %d Detect R-R.", __FUNCTION__, __LINE__);
					move_forward(12, 22);
					usleep(100000);
				}
				else if(receive_code&RconR_HomeL)  //ok 18 13
				{
					ROS_DEBUG("%s %d Detect R-L.", __FUNCTION__, __LINE__);
					move_forward(14, 25);
					usleep(100000);
				}
				else if(receive_code&RconFR2_HomeT)   //turn left
				{
					ROS_DEBUG("%s %d Detect FR2-T.", __FUNCTION__, __LINE__);
					move_forward(14, 23);
					usleep(100000);
				}
				else if(receive_code&RconFR2_HomeR)  //OK
				{
					ROS_DEBUG("%s %d Detect FR2-R.", __FUNCTION__, __LINE__);
					move_forward(14, 20);
					usleep(100000);
				}
				else if(receive_code&RconFR2_HomeL)  //
				{
					ROS_DEBUG("%s, %d: Detect FL2-R.", __FUNCTION__, __LINE__);
					move_forward(21, 15);
					usleep(100000);
				}
				else
				{
					ROS_DEBUG("%s, %d: Else.", __FUNCTION__, __LINE__);
					move_forward(21, 14);
					usleep(100000);
				}
			}
		}
		/*---check_position initial---*/
		else if(g_go_home_state_now == CHECK_POSITION_INIT)
		{
			receive_code = 0;
			gyro_step = 0;
			stop_brifly();
			if(check_position_dir == ROUND_LEFT)
			{
				ROS_INFO("Check position Dir = left");
				set_dir_left();
			}
			else if(check_position_dir == ROUND_RIGHT)
			{
				ROS_INFO("Check position Dir = right");
				set_dir_right();
			}
			set_wheel_speed(10, 10);

			last_angle = robot::instance()->getAngle();
			ROS_DEBUG("Last_Angle = %f.", last_angle);
			g_go_home_state_now = CHECK_POSITION;
		}
		/*---check_position main while---*/
		else if(g_go_home_state_now == CHECK_POSITION)
		{
			if(g_charge_detect)
			{
				if(g_charge_detect_cnt == 0)
					g_charge_detect_cnt++;
				else
					break;
			}
			else if(g_charge_detect_cnt > 0)
			{
				g_charge_detect_cnt = 0;
				if(turn_connect())
					break;
				else
				{
					target_distance = 0.1;
					ROS_WARN("%s %d: Turn connect failed, move back for %fm.", __FUNCTION__, __LINE__, target_distance);
					g_move_back_finished = false;
					saved_pos_x = robot::instance()->getOdomPositionX();
					saved_pos_y = robot::instance()->getOdomPositionY();

					go_home_target_angle = ranged_angle(gyro_get_angle() + 1800);
					turn_finished = false;

					g_go_home_state_now = AWAY_FROM_CHARGER_STATION;
					continue;

				}
			}

			if(g_bumper_left || g_bumper_right)
			{
				ROS_WARN("%s %d: Get bumper trigered.", __FUNCTION__, __LINE__);
				around_charger_stub_dir = 1 - around_charger_stub_dir;
				if(++go_home_bumper_counter > 1)
					g_go_home_state_now = GO_HOME_INIT;
				else
					g_go_home_state_now = AROUND_CHARGER_STATION_INIT;
				target_distance = 0.03;
				g_move_back_finished = false;
				go_home_target_angle = ranged_angle(gyro_get_angle() + 1800);
				ROS_WARN("%s %d: Set target angle:%d.", __FUNCTION__, __LINE__, go_home_target_angle);
				turn_finished = false;
				continue;
			}
			if(g_cliff_triggered)
			{
				ROS_WARN("%s %d: Get cliff trigered.", __FUNCTION__, __LINE__);
				g_cliff_cnt++;
				target_distance = 0.03;
				g_move_back_finished = false;
				g_go_home_state_now = GO_HOME_INIT;
				continue;
			}

			if(gyro_step < 360)
			{
				current_angle = robot::instance()->getAngle();
				angle_offset = ranged_angle(current_angle - last_angle);
				ROS_DEBUG("%s %d: Current_Angle = %f, Last_Angle = %f, Angle_Offset = %f, Gyro_Step = %f.", __FUNCTION__, __LINE__, current_angle, last_angle, angle_offset, gyro_step);
				if (check_position_dir == ROUND_LEFT && angle_offset > 0)
				{
					gyro_step += angle_offset;
				}
				if (check_position_dir == ROUND_RIGHT && angle_offset < 0)
				{
					gyro_step += (-angle_offset);
				}
				last_angle = current_angle;

				receive_code = (get_rcon_status()&(RconL_HomeL|RconL_HomeR|RconFL_HomeL|RconFL_HomeR|RconR_HomeL|RconR_HomeR|RconFR_HomeL|RconFR_HomeR));
				ROS_DEBUG("Check_Position get_rcon_status() == %x, R... == %x, receive code: %x.", get_rcon_status(), (RconL_HomeL|RconL_HomeR|RconFL_HomeL|RconFL_HomeR|RconR_HomeL|RconR_HomeR|RconFR_HomeL|RconFR_HomeR), receive_code);
				if(receive_code)
				{
					reset_rcon_status();
					if (receive_code & RconL_HomeL)ROS_DEBUG("Check_Position get L-L");
					if (receive_code & RconL_HomeR)ROS_DEBUG("Check_Position get L-R");
					if (receive_code & RconFL_HomeL)ROS_DEBUG("Check_Position get FL-L");
					if (receive_code & RconFL_HomeR)ROS_DEBUG("Check_Position get FL-R");
					if (receive_code & RconR_HomeL)ROS_DEBUG("Check_Position get R-L");
					if (receive_code & RconR_HomeR)ROS_DEBUG("Check_Position get R-R");
					if (receive_code & RconFR_HomeL)ROS_DEBUG("Check_Position get FR-L");
					if (receive_code & RconFR_HomeR)ROS_DEBUG("Check_Position get FR-R");
				}

				if(check_position_dir == ROUND_LEFT)
				{
					if(receive_code & (RconFR_HomeL|RconFR_HomeR))
					{
						g_go_home_state_now = BY_PATH_INIT;
						continue;
					}
				}
				if(check_position_dir == ROUND_RIGHT)
				{
					if(receive_code & (RconFL_HomeL|RconFL_HomeR))
					{
						g_go_home_state_now = BY_PATH_INIT;
						continue;
					}
				}
			}
			if(gyro_step >= 360)
			{
				if(entrance_to_check_position == 0)
					g_go_home_state_now = AROUND_CHARGER_STATION;
				else
				{
					ROS_INFO("%s, %d: Robot can't see charger, return to gohome mode.", __FUNCTION__, __LINE__);
					go_home_target_angle = ranged_angle(gyro_get_angle() - 1000);
					turn_finished = false;
					g_go_home_state_now = GO_HOME_INIT;
					continue;
				}
			}
		}
		/*---by_path initial---*/
		else if(g_go_home_state_now == BY_PATH_INIT)
		{
			ROS_INFO("%s %d: Start by path logic.", __FUNCTION__, __LINE__);
			receive_code=0;
			temp_code = 0;
			position_far = true;
			nosignal_counter=0;
			temp_check_position=0;
			near_counter=0;
			side_counter=0;

			reset_stop_event_status();
			set_start_charge();
			move_forward(9, 9);

			g_go_home_state_now = BY_PATH;
			continue;
		}
		/*---by_path main while---*/
		else if(g_go_home_state_now == BY_PATH)
		{
			if(g_charge_detect)
			{
				if(g_charge_detect_cnt == 0)g_charge_detect_cnt++;
				else
				{
					if(!during_cleaning)
						set_clean_mode(Clean_Mode_Charging);
					disable_motors();
					break;
				}
			}
			else if(g_charge_detect_cnt > 0)
			{
				g_charge_detect_cnt = 0;
				if(turn_connect())
					break;
				else
				{
					target_distance = 0.1;
					ROS_WARN("%s %d: Turn connect failed, move back for %fm.", __FUNCTION__, __LINE__, target_distance);
					g_move_back_finished = false;
					saved_pos_x = robot::instance()->getOdomPositionX();
					saved_pos_y = robot::instance()->getOdomPositionY();

					go_home_target_angle = ranged_angle(gyro_get_angle() + 1800);
					turn_finished = false;

					g_go_home_state_now = AWAY_FROM_CHARGER_STATION;
					continue;
				}
			}
			if(g_bumper_left || g_bumper_right)
			{
				ROS_INFO("bumper in by path!");
				g_move_back_finished = false;
				if(!position_far)
				{
					stop_brifly();
					if(turn_connect())
						break;
					ROS_WARN("%d: quick_back in !position_far", __LINE__);

					target_distance = 0.1;
					ROS_WARN("%s %d: Turn connect failed, move back for %fm.", __FUNCTION__, __LINE__, target_distance);
					saved_pos_x = robot::instance()->getOdomPositionX();
					saved_pos_y = robot::instance()->getOdomPositionY();

					go_home_target_angle = ranged_angle(gyro_get_angle() + 1800);
					turn_finished = false;

					g_go_home_state_now = AWAY_FROM_CHARGER_STATION;
					continue;
				}
				else
				{
					if((get_rcon_status()&(RconFL2_HomeL|RconFL2_HomeR|RconFR2_HomeL|RconFR2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR))==0)
						g_go_home_state_now = GO_HOME_INIT;

					target_distance = 0.03;
					if(g_bumper_left)
						go_home_target_angle = ranged_angle(gyro_get_angle() - 1100);
					else
						go_home_target_angle = ranged_angle(gyro_get_angle() + 1100);
					turn_finished = false;
					ROS_WARN("%d: quick_back in position_far", __LINE__);
					continue;
				}
			}
			if(g_cliff_triggered)
			{
				target_distance = 0.03;
				g_move_back_finished = false;
				go_home_target_angle = ranged_angle(gyro_get_angle() + 1750);
				turn_finished = false;
				g_go_home_state_now = GO_HOME_INIT;
				continue;
			}

			if(entrance_to_check_position == 0)
			{
				receive_code = get_rcon_status();
				temp_code = receive_code;
				temp_code &= (	RconL_HomeL|RconL_HomeT|RconL_HomeR| \
								RconFL2_HomeL|RconFL2_HomeT|RconFL2_HomeR| \
								RconFL_HomeL|RconFL_HomeT|RconFL_HomeR| \
								RconFR_HomeL|RconFR_HomeT|RconFR_HomeR| \
								RconFR2_HomeL|RconFR2_HomeT|RconFR2_HomeR| \
								RconR_HomeL|RconR_HomeT|RconR_HomeR \
							 );
				if(receive_code)
				{
					if((receive_code&(RconFR_HomeT|RconFL_HomeT)) == (RconFR_HomeT|RconFL_HomeT))
					{
						position_far = false;
						ROS_DEBUG("%s, %d: Robot face HomeT, position_far = false.", __FUNCTION__, __LINE__);
					}
					if(receive_code&(RconFR2_HomeT|RconFL2_HomeT|RconR_HomeT|RconL_HomeT))
					{
						position_far = false;
						ROS_DEBUG("%s, %d: Robot side face HomeT, position_far = false.", __FUNCTION__, __LINE__);
					}
					if(receive_code&(RconFR_HomeT|RconFL_HomeT|RconFR2_HomeT|RconFL2_HomeT|RconR_HomeT|RconL_HomeT))
					{
						near_counter++;
						if(near_counter > 1)
						{
							position_far = false;
							ROS_DEBUG("%s, %d: Robot near HomeT counter > 1, position_far = false.", __FUNCTION__, __LINE__);
						}
						if((receive_code&(RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR)) == 0)
						{
							side_counter++;
							if(side_counter > 10)
							{
								move_forward(9, 9);
								ROS_INFO("%s, %d: Robot goes far, back to gohome mode.", __FUNCTION__, __LINE__);
								g_go_home_state_now = GO_HOME_INIT;
								continue;
							}
						}
						else
						{
							side_counter = 0;
						}
					}

					if((receive_code&(RconFL_HomeL|RconFR_HomeR))==(RconFL_HomeL|RconFR_HomeR))
					{
						ROS_DEBUG("%s, %d: Robot sees HomeL or HomeR, position_far = false.", __FUNCTION__, __LINE__);
						position_far = false;
					}
					reset_rcon_status();
					nosignal_counter = 0;
				}
				else
				{
					near_counter = 0;
					nosignal_counter++;
					if(nosignal_counter>50)
					{
						ROS_INFO("no signal in by path");
						nosignal_counter = 0;
						stop_brifly();
						check_position_dir = ROUND_LEFT;
						entrance_to_check_position = 1;
						g_go_home_state_now = CHECK_POSITION_INIT;
						continue;
					}
				}
			}

			temp_code &= (RconL_HomeL|RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL|RconR_HomeL| \
						RconL_HomeR|RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR|RconR_HomeR);
			if(position_far)
			{
				switch(temp_code)
				{
					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
						move_forward(12, 12);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR2_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
						move_forward(12, 12);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR2_R/FR_R.", __FUNCTION__, __LINE__);
						move_forward(12, 12);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
						move_forward(13, 11);
						break;

					case (RconFL2_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
						move_forward(13, 9);
						break;

					case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(12, 9);
						break;

					case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
						move_forward(11, 9);
						break;

					case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(12, 8);
						break;

					case (RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_L/FR_R.", __FUNCTION__, __LINE__);
						move_forward(12, 8);
						break;

					case (RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FR_L.", __FUNCTION__, __LINE__);
						move_forward(11, 8);
						break;

					case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(11, 7);
						break;

					case (RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(13, 7);
						break;

					case (RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(12, 7);
						break;

					case (RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR2_R.", __FUNCTION__, __LINE__);
						move_forward(12, 6);
						break;

					case (RconFR2_HomeL):
						ROS_DEBUG("%s, %d: position_far, FR2_L.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() - 250);
						break;

					case (RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() - 350);
						break;

					case (RconFL_HomeL|RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_L.", __FUNCTION__, __LINE__);
						move_forward(11, 10);
						break;

					case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(11, 9);
						break;

					case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(12, 10);
						break;

					case (RconFR_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: position_far, FR_L/FR2_L.", __FUNCTION__, __LINE__);
						move_forward(12, 7);
						break;

					case (RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
						move_forward(12, 11);
						break;

					case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(13, 10);
						break;

					case (RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(13, 8);
						break;

					case (RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FR_R.", __FUNCTION__, __LINE__);
						move_forward(12, 11);
						break;

					case (RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(12, 10);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(12, 10);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(13, 7);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(13, 7);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(12, 11);
						break;

					case (RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(13, 9);
						break;

					case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(13, 8);
						break;

					case (RconFL2_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR_R.", __FUNCTION__, __LINE__);
						move_forward(13, 9);
						break;

					case (RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(12, 11);
						break;

					case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(12, 11);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
						move_forward(12, 11);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
						move_forward(12, 11);
						break;

					case (RconFL_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(12, 11);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR_L.", __FUNCTION__, __LINE__);
						move_forward(13, 12);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR_L.", __FUNCTION__, __LINE__);
						move_forward(13, 12);
						break;

					case (RconR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, R_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() - 250);
						break;

					case (RconR_HomeR|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: position_far, R_R/FR2_L.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() - 400);
						break;

					case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, R_R/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() - 250);
						break;

					case (RconR_HomeR|RconR_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: position_far, R_R/R_L/FR2_L.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() - 450);
						break;

					case (RconR_HomeR|RconR_HomeL):
						ROS_DEBUG("%s, %d: position_far, R_R/R_L.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() - 250);
						break;

					case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: position_far, R_R/FR_L/FR2_L.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() - 550);
						break;

					case (RconR_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: position_far, R_L/FR2_L.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() - 500);
						break;

					case (RconR_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, R_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(9, 2);
						break;

					case (RconR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, R_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(9, 0);
						break;

					case (RconL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, L_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(9, 1);
						break;

					case (RconL_HomeL|RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, L_L/FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(9, 6);
						break;

					case (RconR_HomeR):
						ROS_DEBUG("%s, %d: position_far, R_R.", __FUNCTION__, __LINE__);
						move_forward(10, 0);
						break;

					case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(13, 12);
						break;

					case (RconFL_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR2_L.", __FUNCTION__, __LINE__);
						move_forward(13, 12);
						break;

					case (RconFL2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(14, 4);
						break;

					case (RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(12, 11);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(10, 12);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(8, 12);
						break;

					case (RconFL2_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(11, 13);
						break;

					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
						move_forward(10, 12);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R.", __FUNCTION__, __LINE__);
						move_forward(9, 13);
						break;

					case (RconFL_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FL_R.", __FUNCTION__, __LINE__);
						move_forward(10, 12);
						break;

					case (RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_R.", __FUNCTION__, __LINE__);
						move_forward(7, 11);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FL_L/FL_R.", __FUNCTION__, __LINE__);
						move_forward(7, 12);
						break;

					case (RconFL2_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_R.", __FUNCTION__, __LINE__);
						move_forward(9, 13);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
						move_forward(8, 13);
						break;

					case (RconFL2_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L.", __FUNCTION__, __LINE__);
						move_forward(8, 14);
						break;

					case (RconFL2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 250);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_/FL2_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 350);
						break;

					case (RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_R/FR_R.", __FUNCTION__, __LINE__);
						move_forward(11, 12);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
						move_forward(10, 12);
						break;

					case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
						move_forward(10, 12);
						break;

					case (RconFL2_HomeR|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_R/FL_R.", __FUNCTION__, __LINE__);
						move_forward(7, 12);
						break;

					case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR_L/FR2_L.", __FUNCTION__, __LINE__);
						move_forward(11, 12);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
						move_forward(9, 12);
						break;

					case (RconFL2_HomeL|RconFL_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L.", __FUNCTION__, __LINE__);
						move_forward(7, 12);
						break;

					case (RconFL_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL_L.", __FUNCTION__, __LINE__);
						move_forward(11, 12);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR2_L.", __FUNCTION__, __LINE__);
						move_forward(10, 12);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(10, 12);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(7, 12);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(7, 12);
						break;

					case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(11, 12);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(7, 12);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(7, 12);
						break;

					case (RconFL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(10, 12);
						break;

					case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
						move_forward(11, 12);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FL_R/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(11, 12);
						break;

					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(11, 12);
						break;

					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(11, 12);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL_L/FR_R.", __FUNCTION__, __LINE__);
						move_forward(11, 12);
						break;

					case (RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(11, 12);
						break;

					case (RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(9, 12);
						break;

					case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR):
						ROS_DEBUG("%s, %d: position_far, L_L/FL2_L/FL2_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 250);
						break;

					case (RconL_HomeL|RconFL2_HomeR):
						ROS_DEBUG("%s, %d: position_far, L_L/FL2_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 400);
						break;

					case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, L_L/FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 250);
						break;

					case (RconL_HomeR|RconL_HomeL|RconFL2_HomeR):
						ROS_DEBUG("%s, %d: position_far, L_R/L_L/FL2_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 450);
						break;

					case (RconL_HomeR|RconL_HomeL):
						ROS_DEBUG("%s, %d: position_far, L_R/L_L.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 500);
						break;

					case (RconL_HomeR):
						ROS_DEBUG("%s, %d: position_far, L_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 550);
						break;

					case (RconL_HomeL|RconFL2_HomeR|RconFL_HomeR):
						ROS_DEBUG("%s, %d: position_far, L_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 250);
						break;

					case (RconL_HomeR|RconFL2_HomeR):
						ROS_DEBUG("%s, %d: position_far, L_R/FL2_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 500);
						break;

					case (RconL_HomeL|RconFL2_HomeL|RconFL_HomeL):
						ROS_DEBUG("%s, %d: position_far, L_L/FL2_L/FL_L.", __FUNCTION__, __LINE__);
						move_forward(2, 9);
						break;

					case (RconL_HomeL|RconFL2_HomeL):
						ROS_DEBUG("%s, %d: position_far, L_L/FL2_L.", __FUNCTION__, __LINE__);
						move_forward(0, 9);
						break;

					case (RconR_HomeR|RconFL2_HomeL):
						ROS_DEBUG("%s, %d: position_far, R_R/FL2_L.", __FUNCTION__, __LINE__);
						move_forward(1, 9);
						break;

					case (RconR_HomeR|RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, R_R/FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(6, 9);
						break;

					case (RconL_HomeL):
						ROS_DEBUG("%s, %d: position_far, L_L.", __FUNCTION__, __LINE__);
						move_forward(0, 10);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(11, 12);
						break;

					case (RconFL2_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_R/FR_R.", __FUNCTION__, __LINE__);
						move_forward(11, 12);
						break;

					case (RconFL_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: position_far, FL_R/FR_L.", __FUNCTION__, __LINE__);
						move_forward(7, 12);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: position_far, FL2_L/FL2_R/FR_R.", __FUNCTION__, __LINE__);
						move_forward(11, 12);
						break;

					default:
						ROS_DEBUG("%s, %d: position_far, else:%x.", __FUNCTION__, __LINE__, temp_code);
						move_forward(10, 11);
						break;
				}
				if (!turn_finished)
					continue;
			}
			else
			{
				switch(temp_code)
				{
					case (RconFL_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL-L/FR_R.", __FUNCTION__, __LINE__);
						move_forward(8, 8);
						break;

					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
						move_forward(9, 9);
						break;
					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR2_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
						move_forward(8, 8);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR2_R/FR_R.", __FUNCTION__, __LINE__);
						move_forward(9, 9);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
						move_forward(9, 8);
						break;

					case (RconFL2_HomeL|RconFR2_HomeR|RconFR_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR2_R/FR_R/FR_L.", __FUNCTION__, __LINE__);
						move_forward(10, 8);
						break;

					case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(9, 7);
						break;

					case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
						move_forward(8, 6);
						break;

					case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(9, 6);
						break;

					case (RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_L/FR_R.", __FUNCTION__, __LINE__);
						move_forward(8, 6);
						break;

					case (RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FR_L.", __FUNCTION__, __LINE__);
						move_forward(7, 4);
						break;

					case (RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(7, 3);
						break;

					case (RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(9, 5);
						break;

					case (RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(8, 3);
						break;

					case (RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR2_R.", __FUNCTION__, __LINE__);
						move_forward(9, 3);
						break;

					case (RconFR2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FR2_L.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() - 250);
						break;

					case (RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() - 350);
						break;

					case (RconFL_HomeL|RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_L.", __FUNCTION__, __LINE__);
						move_forward(8, 7);
						break;

					case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(8, 6);
						break;

					case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(9, 7);
						break;

					case (RconFR_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FR_L/FR2_L.", __FUNCTION__, __LINE__);
						move_forward(9, 4);
						break;

					case (RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
						move_forward(8, 7);
						break;

					case (RconFL_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(8, 6);
						break;

					case (RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(9, 7);
						break;

					case (RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FR_R.", __FUNCTION__, __LINE__);
						move_forward(8, 7);
						break;

					case (RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(8, 7);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(8, 6);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(8, 3);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(8, 3);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(8, 7);
						break;

					case (RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(9, 8);
						break;

					case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR_L/FR_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(8, 7);
						break;

					case (RconFL2_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR_R.", __FUNCTION__, __LINE__);
						move_forward(9, 8);
						break;

					case (RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(9, 8);
						break;

					case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(8, 7);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
						move_forward(8, 7);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR_L/FR_R.", __FUNCTION__, __LINE__);
						move_forward(8, 7);
						break;

					case (RconFL_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(8, 7);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR_L.", __FUNCTION__, __LINE__);
						move_forward(9, 8);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR_L.", __FUNCTION__, __LINE__);
						move_forward(9, 6);
						break;

					case (RconR_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, R_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() - 250);
						break;

					case (RconR_HomeR|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, R_R/FR2_L.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() - 400);
						break;

					case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, R_R/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() - 250);
						break;

					case (RconR_HomeR|RconR_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, R_R/R_L/FR2_L.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() - 450);
						break;

					case (RconR_HomeR|RconR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, R_R/R_L.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() - 500);
						break;

					case (RconR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, R_L.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() - 550);
						break;

					case (RconR_HomeR|RconFR_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, R_R/FR_L/FR2_L.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() - 250);
						break;

					case (RconR_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, R_L/FR2_L.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() - 500);
						break;

					case (RconR_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, R_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(9, 2);
						break;

					case (RconR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, R_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(9, 0);
						break;

					case (RconL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, L_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(9, 1);
						break;

					case (RconL_HomeL|RconFL2_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, L_L/FL2_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(9, 6);
						break;

					case (RconR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, R_R.", __FUNCTION__, __LINE__);
						move_forward(10, 0);
						break;

					case (RconFL2_HomeL|RconFR_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(8, 7);
						break;

					case (RconFL_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR2_L.", __FUNCTION__, __LINE__);
						move_forward(9, 8);
						break;

					case (RconFL2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(9, 2);
						break;

					case (RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(9, 8);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(8, 9);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(8, 10);
						break;

					case (RconFL2_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(7, 9);
						break;

					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
						move_forward(6, 8);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R.", __FUNCTION__, __LINE__);
						move_forward(6, 9);
						break;

					case (RconFL_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FL_R.", __FUNCTION__, __LINE__);
						move_forward(6, 8);
						break;

					case (RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_R.", __FUNCTION__, __LINE__);
						move_forward(4, 7);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FL_L/FL_R.", __FUNCTION__, __LINE__);
						move_forward(3, 7);
						break;

					case (RconFL2_HomeL|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_R.", __FUNCTION__, __LINE__);
						move_forward(5, 9);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
						move_forward(3, 8);
						break;

					case (RconFL2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L.", __FUNCTION__, __LINE__);
						move_forward(3, 9);
						break;

					case (RconFL2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 250);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_/FL2_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 350);
						break;

					case (RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_R/FR_R.", __FUNCTION__, __LINE__);
						move_forward(7, 8);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FL_R/FR_R.", __FUNCTION__, __LINE__);
						move_forward(6, 8);
						break;

					case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
						move_forward(7, 9);
						break;

					case (RconFL2_HomeR|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_R/FL_R.", __FUNCTION__, __LINE__);
						move_forward(4, 9);
						break;

					case (RconFL_HomeL|RconFR_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR_L/FR2_L.", __FUNCTION__, __LINE__);
						move_forward(7, 8);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR_R.", __FUNCTION__, __LINE__);
						move_forward(6, 8);
						break;

					case (RconFL2_HomeL|RconFL_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L.", __FUNCTION__, __LINE__);
						move_forward(7, 9);
						break;

						ROS_DEBUG("%s, %d: !position_far, FL_L.", __FUNCTION__, __LINE__);
						move_forward(7, 8);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR2_L.", __FUNCTION__, __LINE__);
						move_forward(7, 8);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(6, 8);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(3, 8);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FL_L/FL_R/FR2_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(3, 8);
						break;

					case (RconFL2_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(7, 8);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(8, 9);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeL|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FL_L/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(7, 8);
						break;

					case (RconFL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(8, 9);
						break;

					case (RconFL2_HomeL|RconFR_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FR_L/FR_R.", __FUNCTION__, __LINE__);
						move_forward(8, 9);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FL_R/FR_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(7, 8);
						break;

					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(7, 8);
						break;

					case (RconFL_HomeL|RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_L/FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(7, 8);
						break;

					case (RconFL2_HomeL|RconFL_HomeL|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL_L/FR_R.", __FUNCTION__, __LINE__);
						move_forward(7, 8);
						break;

					case (RconFL_HomeR|RconFR_HomeL|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_R/FR_L/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(8, 9);
						break;

					case (RconFL_HomeR|RconFR_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL_R/FR_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(6, 9);
						break;

					case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, L_L/FL2_L/FL2_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 250);
						break;

					case (RconL_HomeL|RconFL2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, L_L/FL2_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 400);
						break;

					case (RconL_HomeL|RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, L_L/FL2_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 250);
						break;

					case (RconL_HomeR|RconL_HomeL|RconFL2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, L_R/L_L/FL2_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 450);
						break;

					case (RconL_HomeR|RconL_HomeL):
						ROS_DEBUG("%s, %d: !position_far, L_R/L_L.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 500);
						break;

					case (RconL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, L_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 250);
						break;

					case (RconL_HomeL|RconFL2_HomeR|RconFL_HomeR):
						ROS_DEBUG("%s, %d: !position_far, L_L/FL2_R/FL_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 250);
						break;

					case (RconL_HomeR|RconFL2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, L_R/FL2_R.", __FUNCTION__, __LINE__);
						turn_finished = false;
						go_home_target_angle = ranged_angle(gyro_get_angle() + 500);
						break;

					case (RconL_HomeL|RconFL2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, L_L/FL2_L.", __FUNCTION__, __LINE__);
						move_forward(0, 9);
						break;

					case (RconR_HomeR|RconFL2_HomeL):
						ROS_DEBUG("%s, %d: !position_far, R_R/FL2_L.", __FUNCTION__, __LINE__);
						move_forward(1, 9);
						break;

					case (RconR_HomeR|RconFL2_HomeL|RconFL_HomeL|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, R_R/FL2_L/FL_L/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(6, 9);
						break;

					case (RconL_HomeL):
						ROS_DEBUG("%s, %d: !position_far, L_L.", __FUNCTION__, __LINE__);
						move_forward(0, 10);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFL_HomeR|RconFR2_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FL_R/FR2_R.", __FUNCTION__, __LINE__);
						move_forward(7, 8);
						break;

					case (RconFL2_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_R/FR_R.", __FUNCTION__, __LINE__);
						move_forward(8, 9);
						break;

					case (RconFL_HomeR|RconFR_HomeL):
						ROS_DEBUG("%s, %d: !position_far, FL_R/FR_L.", __FUNCTION__, __LINE__);
						move_forward(2, 9);
						break;

					case (RconFL2_HomeL|RconFL2_HomeR|RconFR_HomeR):
						ROS_DEBUG("%s, %d: !position_far, FL2_L/FL2_R/FR_R.", __FUNCTION__, __LINE__);
						move_forward(8, 9);
						break;

					default:
						ROS_DEBUG("%s, %d: !position_far, else:%x.", __FUNCTION__, __LINE__, temp_code);
						move_forward(7, 8);
						break;
				}
				if (!turn_finished)
					continue;
			}
			usleep(450000);
		}
		usleep(50000);
	}
}

/* turn_connect()
 * return: true - event triggered, including g_charge_detect/g_key_clean_pressed/g_cliff_all_triggered.
 *         false - can't reach the charger stub.
 */
bool turn_connect(void)
{
	ROS_INFO("%s %d: Start turn_connect().", __FUNCTION__, __LINE__);
	// This function is for trying turning left and right to adjust the pose of robot, so that it can charge.
	int16_t target_angle;
	int8_t speed = 5;
	// Enable the switch for charging.
	set_start_charge();
	// Wait for 200ms for charging activated.
	usleep(200000);
	if(g_charge_detect)
	{
		ROS_INFO("Reach charger without turning.");
		return true;
	}
	// Start turning right.
	target_angle = ranged_angle(gyro_get_angle() - 120);
	set_dir_right();
	set_wheel_speed(speed, speed);
	while (abs(ranged_angle(target_angle - gyro_get_angle())) > 20)
	{
		if (g_charge_detect)
		{
			g_charge_detect = 0;
			disable_motors();
			// Wait for a while to decide if it is really on the charger stub.
			usleep(500000);
			if (g_charge_detect)
			{
				ROS_INFO("Turn left reach charger.");
				return true;
			}
			set_wheel_speed(speed, speed);
		}
		if(g_key_clean_pressed || g_fatal_quit_event)
			return true;
	}
	stop_brifly();
	// Start turning left.
	target_angle = ranged_angle(gyro_get_angle() + 240);
	set_dir_left();
	set_wheel_speed(speed, speed);
	while (abs(ranged_angle(target_angle - gyro_get_angle())) > 20)
	{
		if (g_charge_detect)
		{
			g_charge_detect = 0;
			disable_motors();
			// Wait for a while to decide if it is really on the charger stub.
			usleep(500000);
			if (g_charge_detect)
			{
				ROS_INFO("Turn left reach charger.");
				return true;
			}
			set_wheel_speed(speed, speed);
		}
		if(g_key_clean_pressed || g_fatal_quit_event)
			return true;
	}
	stop_brifly();

	return false;
}

bool go_home_check_move_back_finish(float target_distance)
{
	float distance;
	distance = sqrtf(powf(saved_pos_x - robot::instance()->getOdomPositionX(), 2) + powf(saved_pos_y - robot::instance()->getOdomPositionY(), 2));

	set_dir_backward();
	set_wheel_speed(18,18);

	if(distance < target_distance)
		return false;
	else
	{
		if(g_cliff_triggered && get_cliff_status())
		{
			if(++g_cliff_cnt>2)
				g_cliff_jam = true;
			return false;
		}
		else
		{
			if (g_cliff_triggered)
				ROS_WARN("%s %d: reset for cliff.", __FUNCTION__, __LINE__);
			g_cliff_triggered = 0;
			g_cliff_cnt = 0;
		}

		if((g_bumper_left || g_bumper_right) && get_bumper_status())
		{
			if(++g_bumper_cnt>2)
				g_bumper_jam = true;
			return false;
		}
		else
		{
			if (g_bumper_left || g_bumper_right)
				ROS_WARN("%s %d: reset for bumper.", __FUNCTION__, __LINE__);
			g_bumper_cnt = 0;
			g_bumper_left = false;
			g_bumper_right = false;
		}
	}

	return true;
}

bool go_home_check_turn_finish(int16_t target_angle)
{
	static uint8_t turn_speed = 0;
	auto diff = ranged_angle(target_angle - gyro_get_angle());
	if (diff > 0)
		set_dir_left();
	else
		set_dir_right();

	if(g_cliff_triggered)
	{
		ROS_WARN("%s %d: Get cliff trigered.", __FUNCTION__, __LINE__);
		return false;
	}

	if(std::abs(diff) < 10)
	{
		ROS_WARN("%s %d: Turn finish.", __FUNCTION__, __LINE__);
		return true;
	}
	//ROS_WARN("%s %d: Turn not finish yet, target: %d, current: %d, diff: %d.", __FUNCTION__, __LINE__, target_angle, gyro_get_angle(), diff);
	if(std::abs(diff) > 80)
		turn_speed = ROTATE_TOP_SPEED;
	else
	{
		if(turn_speed > ROTATE_LOW_SPEED)
			turn_speed -= 1;
	}
	set_wheel_speed(turn_speed, turn_speed);

	return false;
}

void go_home_register_events(void)
{
	ROS_WARN("%s, %d: Register events.", __FUNCTION__, __LINE__);
	event_manager_set_current_mode(EVT_MODE_HOME);
#define event_manager_register_and_enable_x(name, y, enabled) \
	event_manager_register_handler(y, &go_home_handle_ ##name); \
	event_manager_enable_handler(y, enabled);

	/*---charge detect---*/
	event_manager_register_and_enable_x(charge_detect, EVT_CHARGE_DETECT, true);
	/*---key---*/
	event_manager_register_and_enable_x(key_clean, EVT_KEY_CLEAN, true);
	/*---remote---*/
	event_manager_register_and_enable_x(remote_clean, EVT_REMOTE_CLEAN, true);
	event_manager_enable_handler(EVT_REMOTE_HOME, true);
	event_manager_enable_handler(EVT_REMOTE_DIRECTION_FORWARD, true);
	event_manager_enable_handler(EVT_REMOTE_DIRECTION_LEFT, true);
	event_manager_enable_handler(EVT_REMOTE_DIRECTION_RIGHT, true);
	event_manager_enable_handler(EVT_REMOTE_WALL_FOLLOW, true);
	event_manager_enable_handler(EVT_REMOTE_SPOT, true);
	event_manager_enable_handler(EVT_REMOTE_MAX, true);
	event_manager_enable_handler(EVT_REMOTE_PLAN, true);
	/*---cliff---*/
	event_manager_register_and_enable_x(cliff_all, EVT_CLIFF_ALL, true);
	event_manager_register_and_enable_x(cliff, EVT_CLIFF_LEFT, true);
	event_manager_register_and_enable_x(cliff, EVT_CLIFF_FRONT, true);
	event_manager_register_and_enable_x(cliff, EVT_CLIFF_RIGHT, true);
	/*---bumper---*/
	event_manager_register_and_enable_x(bumper, EVT_BUMPER_RIGHT, true);
	event_manager_register_and_enable_x(bumper, EVT_BUMPER_ALL, true);
	event_manager_register_and_enable_x(bumper, EVT_BUMPER_LEFT, true);
	/*---battery---*/
	event_manager_register_and_enable_x(battery_low, EVT_BATTERY_LOW, true);
	/*---over current---*/
	event_manager_enable_handler(EVT_OVER_CURRENT_BRUSH_LEFT, true);
	event_manager_register_and_enable_x(over_current_brush_main, EVT_OVER_CURRENT_BRUSH_MAIN, true);
	event_manager_enable_handler(EVT_OVER_CURRENT_BRUSH_RIGHT, true);
	event_manager_register_and_enable_x(over_current_wheel_left, EVT_OVER_CURRENT_WHEEL_LEFT, true);
	event_manager_register_and_enable_x(over_current_wheel_right, EVT_OVER_CURRENT_WHEEL_RIGHT, true);
	event_manager_register_and_enable_x(over_current_suction, EVT_OVER_CURRENT_SUCTION, true);
#undef event_manager_register_and_enable_x

	event_manager_set_enable(true);
}

void go_home_unregister_events(void)
{
	ROS_WARN("%s, %d: Unregister events.", __FUNCTION__, __LINE__);
	#define event_manager_register_and_disable_x(x) \
	event_manager_register_handler(x, NULL); \
	event_manager_enable_handler(x, false);

	/*---charge detect---*/
	event_manager_register_and_disable_x(EVT_CHARGE_DETECT);
	/*---key---*/
	event_manager_register_and_disable_x(EVT_KEY_CLEAN);
	/*---remote---*/
	event_manager_register_and_disable_x(EVT_REMOTE_CLEAN);
	event_manager_enable_handler(EVT_REMOTE_HOME, false);
	event_manager_enable_handler(EVT_REMOTE_DIRECTION_FORWARD, false);
	event_manager_enable_handler(EVT_REMOTE_DIRECTION_LEFT, false);
	event_manager_enable_handler(EVT_REMOTE_DIRECTION_RIGHT, false);
	event_manager_enable_handler(EVT_REMOTE_WALL_FOLLOW, false);
	event_manager_enable_handler(EVT_REMOTE_SPOT, false);
	event_manager_enable_handler(EVT_REMOTE_MAX, false);
	event_manager_enable_handler(EVT_REMOTE_PLAN, false);
	/*---cliff---*/
	event_manager_register_and_disable_x(EVT_CLIFF_ALL);
	event_manager_register_and_disable_x(EVT_CLIFF_LEFT);
	event_manager_register_and_disable_x(EVT_CLIFF_FRONT);
	event_manager_register_and_disable_x(EVT_CLIFF_RIGHT);
	/*---bumper---*/
	event_manager_register_and_disable_x(EVT_BUMPER_RIGHT);
	event_manager_register_and_disable_x(EVT_BUMPER_ALL);
	event_manager_register_and_disable_x(EVT_BUMPER_LEFT);
	/*---battery---*/
	event_manager_register_and_disable_x(EVT_BATTERY_LOW);
	/* Over Current */
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_BRUSH_LEFT);
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_BRUSH_MAIN);
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_BRUSH_RIGHT);
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_WHEEL_LEFT);
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_WHEEL_RIGHT);
	event_manager_register_and_disable_x(EVT_OVER_CURRENT_SUCTION);
#undef event_manager_register_and_disable_x

	event_manager_set_enable(false);
}

void go_home_handle_charge_detect(bool state_now, bool state_last)
{
	if(g_go_home_state_now == CHECK_POSITION)
	{
		g_charge_detect = 1;
	}
	else
	{
		if(state_now == true && state_last == true)
		{
			g_charge_detect_cnt++;
			if(g_charge_detect_cnt > 2)
			{
				g_charge_detect = 1;
			}
		}
		else
		{
			g_charge_detect_cnt = 0;
		}
	}
}

void go_home_handle_key_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Key clean is pressed.", __FUNCTION__, __LINE__);
	time_t start_time;
	bool reset_manual_pause = false;
	beep_for_command(VALID);
	set_wheel_speed(0, 0);
	g_key_clean_pressed = true;
	start_time = time(NULL);

	if (during_cleaning && get_clean_mode() == Clean_Mode_Navigation)
		robot::instance()->setManualPause();

	ROS_WARN("%s %d: Key clean is not released.", __FUNCTION__, __LINE__);
	while (get_key_press() & KEY_CLEAN)
	{
		if (during_cleaning && get_clean_mode() == Clean_Mode_Navigation && (time(NULL) - start_time > 3))
		{
			if (!reset_manual_pause)
			{
				beep_for_command(VALID);
				reset_manual_pause = true;
			}
			robot::instance()->resetManualPause();
			ROS_WARN("%s %d: Key clean is not released and manual pause has been reset.", __FUNCTION__, __LINE__);
		}
		usleep(20000);
	}
	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);

	reset_touch();
}

void go_home_handle_remote_clean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Remote clean is pressed.", __FUNCTION__, __LINE__);
	beep_for_command(VALID);
	g_key_clean_pressed = true;
	if (during_cleaning && get_clean_mode() == Clean_Mode_Navigation)
		robot::instance()->setManualPause();
	reset_rcon_remote();
}

void go_home_handle_cliff_all(bool state_now, bool state_last)
{
	g_cliff_all_cnt++;
	if (g_cliff_all_cnt++ > 2)
	{
		g_cliff_all_triggered = true;
		g_fatal_quit_event = true;
	}
	g_cliff_triggered = Status_Cliff_All;
	if (during_cleaning)
		map_set_cliff();
	if (g_move_back_finished && !g_cliff_jam && !state_last)
		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void go_home_handle_cliff(bool state_now, bool state_last)
{
	if (state_now == true && state_last == false)
	{
		ROS_WARN("%s %d: Cliff triggered.", __FUNCTION__, __LINE__);
		saved_pos_x = robot::instance()->getOdomPositionX();
		saved_pos_y = robot::instance()->getOdomPositionY();
		g_cliff_triggered = Status_Cliff_All;
		if (during_cleaning)
			map_set_cliff();
	}
}

void go_home_handle_bumper(bool state_now, bool state_last)
{
	if (state_now == true && state_last == false)
	{
		saved_pos_x = robot::instance()->getOdomPositionX();
		saved_pos_y = robot::instance()->getOdomPositionY();
		if (get_bumper_status() & LeftBumperTrig)
		{
			ROS_WARN("%s %d: Left bumper triggered.", __FUNCTION__, __LINE__);
			g_bumper_left = true;
		}
		if (get_bumper_status() & RightBumperTrig)
		{
			ROS_WARN("%s %d: Left bumper triggered.", __FUNCTION__, __LINE__);
			g_bumper_right = true;
		}
		if (during_cleaning)
			map_set_bumper();
	}
}

void go_home_handle_battery_low(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: Battery too low (< LOW_BATTERY_STOP_VOLTAGE)", __FUNCTION__, __LINE__);
	g_battery_low = true;
}

void go_home_handle_over_current_brush_main(bool state_now, bool state_last)
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

void go_home_handle_over_current_wheel_left(bool state_now, bool state_last)
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

void go_home_handle_over_current_wheel_right(bool state_now, bool state_last)
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

void go_home_handle_over_current_suction(bool state_now, bool state_last)
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

