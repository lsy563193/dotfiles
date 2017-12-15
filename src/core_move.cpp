#include <battery.h>
#include <brush.h>
#include <bumper.h>
#include <remote.hpp>
#include <beep.h>
#include <charger.h>
#include <wall_follow.h>
#include <pp.h>
#include "pp.h"
#include "wheel.hpp"
#include "error.h"
#include "odom.h"

PPTargetType g_plan_path;
std::deque<Cell_t> g_passed_path;

//Cell_t g_next_cell;
//Cell_t g_target_cell;

//uint8_t	g_remote_go_home = 0;
//bool	cs.is_going_home() = false;
bool	g_from_charger = false;

// This flag is indicating robot is resuming from low battery go home.
bool g_resume_cleaning = false;

// This flag is for checking whether clean_map boundary is created.

bool g_have_seen_charger = false;
bool g_start_point_seen_charger = false;

extern int16_t g_x_min, g_x_max, g_y_min, g_y_max;

// Saved position for move back.
float saved_pos_x, saved_pos_y;

// Flag for indicating whether move back has finished.
bool g_move_back_finished = true;

// Flag for indicating whether motion instance is initialized successfully.
bool g_motion_init_succeeded = false;

bool g_go_home_by_remote = false;

//Flag for judge if keep on wall follow
bool g_keep_on_wf;

time_t last_time_remote_spot = time(NULL);
CM_EventHandle eh;

void cm_self_check_with_handle(void)
{
	// Can not set handler state inside cm_self_check(), because it is actually a universal function.
	cm_self_check();
	if(cm_is_follow_wall()) {
		g_keep_on_wf = true;
	}
}

void cm_cleaning() {
//	Cell_t curr = nav_map.update_position();
	Cell_t curr = {0, 0, 0};

	CleanMode* p_cm;
	if(cm_is_follow_wall())
		p_cm = new WallFollowClean(curr, g_plan_path.front(), g_plan_path);
	else if(cm_is_exploration())
		p_cm = new Exploration(curr, g_plan_path.front(), g_plan_path);
	else if(cm_is_spot())
		p_cm = new SpotClean(curr, g_plan_path.front(), g_plan_path);
	else
		p_cm = new NavigationClean(curr, g_plan_path.front(), g_plan_path);

	MotionManage motion(p_cm);
	if (!motion.initSucceeded())
		return;

	g_motion_init_succeeded = true;
	cs.init();

	g_check_path_in_advance = false;

	p_cm->run();
/*
	while (ros::ok()) {

		if (p_cm->isExit()) {
			break;
		}

		if (ev.slam_error) {
			wheel.stop();
			continue;
		}

		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}

		curr = p_cm->updatePosition({nav_map.get_x_count(), nav_map.get_y_count()});

		if (p_cm->isReach() || p_cm->isStop())
		{
			if(!p_cm->csm_next(curr))
				return;
		}


	if (cm_should_self_check())
		cm_self_check_with_handle();
	}*/
}

void cm_apply_cs(int next) {
	if (next == CS_OPEN_GYRO)
	{
		// Set for LEDs.
		if (ev.remote_home || g_go_home_by_remote)
			led.set_mode(LED_FLASH, LED_ORANGE, 1000);
		else
			led.set_mode(LED_FLASH, LED_GREEN, 1000);

		// Operate on gyro.
		gyro.setOff();
		usleep(30000);
		gyro.reOpen();

		// Reset for keys.
		key.resetTriggerStatus();

		// Playing wavs.
		// Can't register until the status has been checked. because if register too early, the handler may affect the pause status, so it will play the wrong wav.
		cm_register_events();
		if (g_resume_cleaning)
		{
			ROS_WARN("Restore from low battery pause");
			speaker.play(SPEAKER_CLEANING_CONTINUE);
		}
		else if (cs_is_paused())
		{
			ROS_WARN("Restore from manual pause");
			speaker.play(SPEAKER_CLEANING_CONTINUE);
			if (cs.is_going_home())
			{
				speaker.play(SPEAKER_BACK_TO_CHARGER);
			}
		}
		else if(g_plan_activated == true)
		{
			g_plan_activated = false;
		}
		else{
			speaker.play(SPEAKER_CLEANING_START);
		}
	}
	else if (next == CS_BACK_FROM_CHARGER)
	{
		path_set_home(nav_map.getCurrCell());
		cs_work_motor();
		wheel.setDirBackward();
	}
	else if (next == CS_OPEN_LIDAR)
	{
		cs_work_motor();
		lidar.motorCtrl(ON);
		lidar.setScanOriginalReady(0);
	}
	else if (next == CS_ALIGN)
	{
		lidar.startAlign();
	}
	else if (next == CS_OPEN_SLAM)
	{
		if (!(g_is_manual_pause || g_resume_cleaning))
		{
			robot::instance()->setTfReady(false);
			robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle);
			slam.start();
		}
		else
			robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle);
	}
	else if (next == CS_CLEAN) {
		g_wf_reach_count = 0;
		led.set_mode(LED_STEADY, LED_GREEN);
	}
	if (next == CS_GO_HOME_POINT) {
		cs_work_motor();
		wheel.setPidTargetSpeed(0, 0, REG_TYPE_LINEAR);
		if (ev.remote_home || cm_is_go_charger())
			led.set_mode(LED_STEADY, LED_ORANGE);

		// Special handling for wall follow mode_.
		if (cm_is_follow_wall()) {
			robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle); //For wall follow mode_.
			nav_map.updatePosition();
			//wf_mark_home_point();
			nav_map.reset(CLEAN_MAP);
			nav_map.mergeFromSlamGridMap(slam_grid_map, false, false, true, false, false);
			nav_map.markRobot(CLEAN_MAP);//note: To clear the obstacles before go home, please don't remove it!
		}
		// Play wavs.
		if (ev.battrey_home)
			speaker.play(SPEAKER_BATTERY_LOW);
		if (!cm_is_go_charger())
			speaker.play(SPEAKER_BACK_TO_CHARGER);

		if (ev.remote_home)
			g_go_home_by_remote = true;
		ev.remote_home = false;
		ev.battrey_home = false;
		mt.set(MT_LINEARMOVE);
	}
	if (next == CS_TMP_SPOT)
	{
		if( SpotMovement::instance() -> getSpotType() == NO_SPOT){
			ROS_INFO("%s %d: Entering temp spot during navigation.", __FUNCTION__, __LINE__);
			Cell_t curr_cell = nav_map.getCurrCell();
			ROS_WARN("%s %d: current cell(%d, %d).", __FUNCTION__, __LINE__, curr_cell.X, curr_cell.Y);
			SpotMovement::instance() ->setSpotType(CLEAN_SPOT);
			wheel.stop();
		}
		else if(SpotMovement::instance()->getSpotType() == CLEAN_SPOT){
			ROS_INFO("%s %d: Exiting temp spot.", __FUNCTION__, __LINE__);
			SpotMovement::instance()->spotDeinit();
			wheel.stop();
			speaker.play(SPEAKER_CLEANING_CONTINUE);
		}
		ev.remote_spot = false;
	}
	if (next == CS_TRAPPED)
	{
		g_wf_start_timer = time(NULL);
		g_wf_diff_timer = ESCAPE_TRAPPED_TIME;
		led.set_mode(LED_FLASH, LED_GREEN, 300);
		mt.set(MT_FOLLOW_LEFT_WALL);
	}
	if (next == CS_EXPLORATION) {
		mt.set(MT_LINEARMOVE);
		g_wf_reach_count = 0;
		led.set_mode(LED_STEADY, LED_ORANGE);
	}
	if (next == CS_GO_CHANGER)
	{
		gyro.TiltCheckingEnable(false); //disable tilt detect
		led.set_mode(LED_STEADY, LED_ORANGE);
	}
	if (next == CS_SELF_CHECK)
	{
		led.set_mode(LED_STEADY, LED_GREEN);
	}
}

void cm_reset_go_home(void)
{
	ROS_DEBUG("%s %d: Reset go home flags here.", __FUNCTION__, __LINE__);
	cs.setNext(CS_CLEAN);
	g_go_home_by_remote = false;
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

	if (ev.bumper_jam || ev.cliff_jam || ev.lidar_stuck)
	{
		// Save current position for moving back detection.
		saved_pos_x = odom.getX();
		saved_pos_y = odom.getY();
	}

	if (ev.oc_wheel_left || ev.oc_wheel_right)
		cs_disable_motors();

	if (ev.oc_suction)
	{
		ROS_WARN("%s, %d: Vacuum Self checking start", __FUNCTION__, __LINE__);
		cs_disable_motors();
		vacuum.startSelfCheck();
	}

	SelfCheckRegulator regulator;

	robot::instance()->obsAdjustCount(50);
	while (ros::ok()) {
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}

		if (ev.fatal_quit || ev.key_clean_pressed || ev.charge_detect)
			break;

		if (ev.slam_error)
		{
			wheel.stop();
			continue;
		}

		// Check for right wheel.
		if (ev.oc_wheel_left || ev.oc_wheel_right)
		{
			if (time(NULL) - start_time >= 1)
			{
				wheel_current_sum /= wheel_current_sum_cnt;
				if (wheel_current_sum > Wheel_Stall_Limit)
				{
					if (resume_cnt >= 3)
					{
						cs_disable_motors();
						if (ev.oc_wheel_left)
						{
							ROS_WARN("%s,%d Left wheel stall maybe, please check!!\n", __FUNCTION__, __LINE__);
							error.set(ERROR_CODE_LEFTWHEEL);
						}
						else
						{
							ROS_WARN("%s,%d Right wheel stall maybe, please check!!\n", __FUNCTION__, __LINE__);
							error.set(ERROR_CODE_RIGHTWHEEL);
						}
						ev.fatal_quit = true;
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
					if (ev.oc_wheel_left)
					{
						ROS_WARN("%s %d: Left wheel resume succeeded.", __FUNCTION__, __LINE__);
						ev.oc_wheel_left = false;
						cs_work_motor();
					}
					else
					{
						ROS_WARN("%s %d: Left wheel resume succeeded.", __FUNCTION__, __LINE__);
						ev.oc_wheel_right = false;
						cs_work_motor();
					}
				}
			}
			else
			{
				if (ev.oc_wheel_left)
					wheel_current_sum += (uint32_t) wheel.getLeftWheelCurrent();
				else
					wheel_current_sum += (uint32_t) wheel.getRightWheelCurrent();
				wheel_current_sum_cnt++;
			}
		}
		else if (ev.cliff_jam)
		{
			if (!cliff.get_status())
			{
				ROS_WARN("%s %d: Cliff resume succeeded.", __FUNCTION__, __LINE__);
				ev.cliff_triggered = 0;
				ev.cliff_all_triggered = false;
				g_cliff_cnt = 0;
				g_cliff_all_cnt = 0;
				ev.cliff_jam = false;
			}
			float distance;
			distance = sqrtf(powf(saved_pos_x - odom.getX(), 2) + powf(saved_pos_y - odom.getY(), 2));
			if (fabsf(distance) > 0.05f)
			{
				if (++resume_cnt >= 3)
				{
					ROS_WARN("%s %d: Cliff jamed.", __FUNCTION__, __LINE__);
					ev.fatal_quit = true;
					error.set(ERROR_CODE_CLIFF);
				}
				else
				{
					wheel.stop();
					saved_pos_x = odom.getX();
					saved_pos_y = odom.getY();
				}
			}
		}
		else if (ev.bumper_jam)
		{
			if (!bumper.get_status())
			{
				ROS_WARN("%s %d: Bumper resume succeeded.", __FUNCTION__, __LINE__);
				ev.bumper_jam = false;
				ev.bumper_triggered = false;
				g_bumper_cnt = 0;
			}

			switch (bumper_jam_state)
			{
				case 1: // Move back for the first time.
				case 2: // Move back for the second time.
				{
					float distance;
					distance = sqrtf(powf(saved_pos_x - odom.getX(), 2) + powf(saved_pos_y - odom.getY(), 2));
					if (fabsf(distance) > 0.05f)
					{
						wheel.stop();
						// If cliff jam during bumper self resume.
						if (cliff.get_status() && ++g_cliff_cnt > 2)
						{
							ev.cliff_jam = true;
							resume_cnt = 0;
						}
						else
						{
							bumper_jam_state++;
							ROS_WARN("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state);
						}
						saved_pos_x = odom.getX();
						saved_pos_y = odom.getY();
					}
					break;
				}
				case 3: // Move back for the third time.
				{
					float distance;
					distance = sqrtf(powf(saved_pos_x - odom.getX(), 2) + powf(saved_pos_y - odom.getY(), 2));
					if (fabsf(distance) > 0.05f)
					{
						// If cliff jam during bumper self resume.
						if (cliff.get_status() && ++g_cliff_cnt > 2)
						{
							ev.cliff_jam = true;
							resume_cnt = 0;
						}
						else
						{
							bumper_jam_state++;
							ROS_WARN("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state);
							target_angle = ranged_angle(robot::instance()->getPoseAngle() - 900);
							ROS_WARN("%s %d: target_angle:%d.", __FUNCTION__, __LINE__, target_angle);
						}
					}
					break;
				}
				case 4:
				{
					ROS_DEBUG("%s %d: robot::instance()->getPoseAngle(): %d", __FUNCTION__, __LINE__, robot::instance()->getPoseAngle());
					// If cliff jam during bumper self resume.
					if (cliff.get_status() && ++g_cliff_cnt > 2)
					{
						ev.cliff_jam = true;
						resume_cnt = 0;
					}
					else if (abs(ranged_angle(robot::instance()->getPoseAngle() - target_angle)) < 50)
					{
						bumper_jam_state++;
						ROS_WARN("%s %d: Try bumper resume state %d.", __FUNCTION__, __LINE__, bumper_jam_state);
						target_angle = ranged_angle(robot::instance()->getPoseAngle() + 900);
						ROS_WARN("%s %d: target_angle:%d.", __FUNCTION__, __LINE__, target_angle);
					}
					break;
				}
				case 5:
				{
					// If cliff jam during bumper self resume.
					if (cliff.get_status() && ++g_cliff_cnt > 2)
					{
						ev.cliff_jam = true;
						resume_cnt = 0;
					}
					else if (abs(ranged_angle(robot::instance()->getPoseAngle() - target_angle)) < 50)
					{
						ROS_WARN("%s %d: Bumper jamed.", __FUNCTION__, __LINE__);
						ev.fatal_quit = true;
						error.set(ERROR_CODE_BUMPER);
					}
					break;
				}
			}
		}
		else if (ev.oc_suction)
		{
			if(vacuum_oc_state == 1)
			{
				ROS_DEBUG("%s %d: Wait for suction self check begin.", __FUNCTION__, __LINE__);
				if (vacuum.getSelfCheckStatus() == 0x10)
				{
					ROS_WARN("%s %d: Suction self check begin.", __FUNCTION__, __LINE__);
					vacuum.resetSelfCheck();
					vacuum_oc_state = 2;
				}
				continue;
			}
			else if (vacuum_oc_state == 2)
			{
				ROS_DEBUG("%s %d: Wait for suction self check result.", __FUNCTION__, __LINE__);
				if (vacuum.getSelfCheckStatus() == 0x20)
				{
					ROS_WARN("%s %d: Resume suction failed.", __FUNCTION__, __LINE__);
					error.set(ERROR_CODE_FAN_H);
					ev.fatal_quit = true;
					break;
				}
				else if (vacuum.getSelfCheckStatus() == 0x00)
				{
					ROS_WARN("%s %d: Resume suction succeeded.", __FUNCTION__, __LINE__);
					ev.oc_suction = false;
					g_oc_suction_cnt = 0;
					break;
				}
			}
		}
		else if (g_slip_cnt >= 2)
		{
			if(g_slip_cnt < 3 && g_robot_slip){
				g_robot_slip = false;
				target_angle = ranged_angle(robot::instance()->getPoseAngle() + 900);
				ROS_INFO("%s,%d,\033[32mrobot slip again slip count %d\033[0m",__FUNCTION__,__LINE__,g_slip_cnt);
			}
			else if(g_slip_cnt <4 && g_robot_slip){
				g_robot_slip = false;
				target_angle = ranged_angle(robot::instance()->getPoseAngle() - 900);
				ROS_INFO("%s,%d,\033[32mrobot slip again slip count %d\033[0m",__FUNCTION__,__LINE__,g_slip_cnt);
			}
			/*
			if(g_robot_slip){
				g_robot_slip = false;
				ROS_INFO("%s,%d,robot slip again",__FUNCTION__,__LINE__);
			}
			*/
			if( abs(robot::instance()->getPoseAngle() - target_angle) <= 50 ){
				g_slip_cnt = 0;
				g_robot_slip = false;
				ROS_INFO("\033[32m%s,%d,reach target angle\033[0m ",__FUNCTION__,__LINE__);
				break;
			}
			if(g_slip_cnt>=4){
				ROS_INFO("%s,%d,robot stuck ,slip count\033[32m %d \033[0m",__FUNCTION__,__LINE__,g_slip_cnt);
				g_slip_cnt = 0;
				g_robot_stuck = true;
				error.set(ERROR_CODE_STUCK);
				//ev.fatal_quit = true;
				break;
			}
		}
		else if (ev.lidar_stuck)
		{
			if (!lidar_is_stuck())
			{
				ROS_WARN("%s %d: Restore from lidar stuck.", __FUNCTION__, __LINE__);
				ev.lidar_stuck = false;
			}

			float distance = sqrtf(powf(saved_pos_x - odom.getX(), 2) + powf(saved_pos_y - odom.getY(), 2));
			if (fabsf(distance) > 0.35f)
			{
				ROS_WARN("%s %d: Lidar stuck.", __FUNCTION__, __LINE__);
				ev.fatal_quit = true;
				error.set(ERROR_CODE_LIDAR);
			}
		}
		else
			break;

		regulator.adjustSpeed(bumper_jam_state);
	}
}

bool cm_should_self_check(void)
{
	return (ev.oc_wheel_left || ev.oc_wheel_right || ev.bumper_jam || ev.cliff_jam || ev.oc_suction || g_slip_cnt >= 2 || ev.lidar_stuck);
}

/* Event handler functions. */
void cm_register_events()
{
	event_manager_register_handler(&eh);
	event_manager_set_enable(true);
}

void cm_unregister_events()
{
	event_manager_set_enable(true);
}

void CM_EventHandle::bumperAll(bool state_now, bool state_last)
{
//	ev.bumper_triggered = BLOCK_ALL;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		wheel.stop();
//	}

//	if (g_move_back_finished && !ev.bumper_jam)
//		ROS_WARN("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, bumper.get_status(), state_now ? "true" : "false", state_last ? "true" : "false");

}

void CM_EventHandle::bumperLeft(bool state_now, bool state_last)
{
//	if(ev.bumper_triggered != 0)
//		return;
//	ev.bumper_triggered = BLOCK_LEFT;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		wheel.stop();
//	}

//	if (g_move_back_finished && !ev.bumper_jam)
//		ROS_WARN("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, bumper.get_status(), state_now ? "true" : "false", state_last ? "true" : "false");
}

void CM_EventHandle::bumperRight(bool state_now, bool state_last)
{
//	if(ev.bumper_triggered != 0)
//		return;
//
//	ev.bumper_triggered = BLOCK_RIGHT;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		wheel.stop();
//	}

//	if (g_move_back_finished && !ev.bumper_jam)
//		ROS_WARN("%s %d: is called, bumper: %d\tstate now: %s\tstate last: %s", __FUNCTION__, __LINE__, bumper.get_status(), state_now ? "true" : "false", state_last ? "true" : "false");
}

/* OBS */
void CM_EventHandle::obsFront(bool state_now, bool state_last)
{
//	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
//	ev.obs_triggered = Status_Front_OBS;
}

void CM_EventHandle::obsLeft(bool state_now, bool state_last)
{
//	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
//	ev.obs_triggered = Status_Left_OBS;
}

void CM_EventHandle::obsRight(bool state_now, bool state_last)
{
//	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);
//	ev.obs_triggered = Status_Right_OBS;
}

/* Cliff */
void CM_EventHandle::cliffAll(bool state_now, bool state_last)
{
	g_cliff_all_cnt++;
	if (g_cliff_all_cnt++ > 2)
	{
		ev.cliff_all_triggered = true;
		ev.fatal_quit = true;
	}
	ev.cliff_triggered = BLOCK_ALL;
	if (g_move_back_finished && !ev.cliff_jam && !state_last)
		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void CM_EventHandle::cliffFrontLeft(bool state_now, bool state_last)
{
//	ev.cliff_all_triggered = false;
//	ev.cliff_triggered = Status_Cliff_LF;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		wheel.stop();
//	}

//	if (g_move_back_finished && !ev.cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");

}

void CM_EventHandle::cliffFrontRight(bool state_now, bool state_last)
{
//	ev.cliff_all_triggered = false;
//	ev.cliff_triggered = Status_Cliff_RF;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		wheel.stop();
//	}

//	if (g_move_back_finished && !ev.cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");

}

void CM_EventHandle::cliffLeftRight(bool state_now, bool state_last)
{
//	ev.cliff_all_triggered = false;
//	ev.cliff_triggered = Status_Cliff_LR;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		wheel.stop();
//	}
//
//	if (g_move_back_finished && !ev.cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void CM_EventHandle::cliffFront(bool state_now, bool state_last)
{
//	ev.cliff_all_triggered = false;
//	ev.cliff_triggered = Status_Cliff_Front;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		wheel.stop();
//	}
//
//	if (g_move_back_finished && !ev.cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void CM_EventHandle::cliffLeft(bool state_now, bool state_last)
{
//	ev.cliff_all_triggered = false;
//	ev.cliff_triggered = Status_Cliff_Left;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		wheel.stop();
//	}

//	if (g_move_back_finished && !ev.cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

void CM_EventHandle::cliffRight(bool state_now, bool state_last)
{
//	ev.cliff_all_triggered = false;
//	ev.cliff_triggered = Status_Cliff_Right;

//	if (!state_last && g_move_back_finished)
//	{
//		saved_pos_x = robot::instance()->getOdomPositionX();
//		saved_pos_y = robot::instance()->getOdomPositionY();
//		wheel.stop();
//	}

//	if (g_move_back_finished && !ev.cliff_jam && !state_last)
//		ROS_WARN("%s %d: is called, state now: %s\tstate last: %s", __FUNCTION__, __LINE__, state_now ? "true" : "false", state_last ? "true" : "false");
}

/* RCON */
void CM_EventHandle::rcon(bool state_now, bool state_last)
{
/*
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (cs.is_going_home()) {
		ROS_DEBUG("%s %d: is called. Skip while going home.", __FUNCTION__, __LINE__);
		return;
	}
	if(mt.is_follow_wall()){
		if (!(get_rcon_status() & (RconL_HomeT | RconR_HomeT | RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT)))
			return;
	}
	else if (mt.is_linear())
		// Since we have front left 2 and front right 2 rcon receiver, seems it is not necessary to handle left or right rcon receives home signal.
		if (!(c_rcon.getStatus() & (RconFL_HomeT | RconFR_HomeT | RconFL2_HomeT | RconFR2_HomeT)))
		return;

	ev.rcon_triggered = c_rcon.get_trig_();
	if(ev.rcon_triggered != 0){
		nav_map.set_rcon();
	}
	c_rcon.resetStatus();*/
}

/* Over Current */
//void CM_EventHandle::over_current_brush.left(bool state_now, bool state_last)
//{
//	static uint8_t stop_cnt = 0;
//
//	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
//	if(!robot::instance()->getLbrushOc()) {
//		g_oc_brush.left_cnt = 0;
//		if (stop_cnt++ > 250) {
//			brush.setLeftPwm(30);
//		}
//		return;
//	}
//
//	stop_cnt = 0;
//	if (g_oc_brush.left_cnt++ > 40) {
//		g_oc_brush.left_cnt = 0;
//		brush.setLeftPwm(0);
//		ROS_WARN("%s %d: left brush over current", __FUNCTION__, __LINE__);
//	}
//}

void CM_EventHandle::overCurrentBrushMain(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!brush.getMainOc()){
		brush.oc_main_cnt_ = 0;
		return;
	}

	if (brush.oc_main_cnt_++ > 40) {
		brush.oc_main_cnt_ = 0;
		ROS_WARN("%s %d: main brush over current", __FUNCTION__, __LINE__);

		if (cs_self_check(Check_Main_Brush) == 1) {
			ev.oc_brush_main = true;
			ev.fatal_quit = true;
		}
    }
}

//void CM_EventHandle::over_current_brush.right(bool state_now, bool state_last)
//{
//	static uint8_t stop_cnt = 0;
//
//	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);
//
//	if(!robot::instance()->getRbrushOc()) {
//		g_oc_brush.right_cnt = 0;
//		if (stop_cnt++ > 250) {
//			brush.setRightPwm(30);
//		}
//		return;
//	}
//
//	stop_cnt = 0;
//	if (g_oc_brush.right_cnt++ > 40) {
//		g_oc_brush.right_cnt = 0;
//		brush.setRightPwm(0);
//		ROS_WARN("%s %d: reft brush over current", __FUNCTION__, __LINE__);
//	}
//}

void CM_EventHandle::overCurrentWheelLeft(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if ((uint32_t) wheel.getLeftWheelCurrent() < Wheel_Stall_Limit) {
        g_oc_wheel_left_cnt = 0;
		return;
	}

	if (g_oc_wheel_left_cnt++ > 40){
		g_oc_wheel_left_cnt = 0;
		ROS_WARN("%s %d: left wheel over current, \033[1m%u mA\033[0m", __FUNCTION__, __LINE__, (uint32_t) wheel.getLeftWheelCurrent());

		ev.oc_wheel_left = true;
	}
}

void CM_EventHandle::overCurrentWheelRight(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if ((uint32_t) wheel.getRightWheelCurrent() < Wheel_Stall_Limit) {
		g_oc_wheel_right_cnt = 0;
		return;
	}

	if (g_oc_wheel_right_cnt++ > 40){
		g_oc_wheel_right_cnt = 0;
		ROS_WARN("%s %d: right wheel over current, \033[1m%u mA\033[0m", __FUNCTION__, __LINE__, (uint32_t) wheel.getRightWheelCurrent());

		ev.oc_wheel_right = true;
	}
}

void CM_EventHandle::overCurrentSuction(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!vacuum.getOc()) {
		g_oc_suction_cnt = 0;
		return;
	}

	if (g_oc_suction_cnt++ > 40) {
		g_oc_suction_cnt = 0;
		ROS_WARN("%s %d: vacuum over current", __FUNCTION__, __LINE__);

		ev.oc_suction = true;
	}
}

/* Key */
void CM_EventHandle::keyClean(bool state_now, bool state_last)
{
	time_t start_time;
	bool reset_manual_pause = false;

	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (ev.slam_error)
	{
		beeper.play_for_command(INVALID);
		while (key.getPressStatus())
		{
			usleep(20000);
		}
		ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);
		key.resetTriggerStatus();
		return;
	}

	beeper.play_for_command(VALID);
	wheel.stop();
	ev.key_clean_pressed = true;

	if(cm_is_navigation())
		g_is_manual_pause = true;

	start_time = time(NULL);
	while (key.getPressStatus())
	{
		if (cm_is_navigation() && time(NULL) - start_time > 3) {
			if (!reset_manual_pause)
			{
				beeper.play_for_command(VALID);
				reset_manual_pause = true;
				g_is_manual_pause = false;
				ROS_WARN("%s %d: Manual pause has been reset.", __FUNCTION__, __LINE__);
			}
		}
		else
			ROS_DEBUG("%s %d: Key clean is not released.", __FUNCTION__, __LINE__);
		usleep(20000);
	}

	ROS_WARN("%s %d: Key clean is released.", __FUNCTION__, __LINE__);
	key.resetTriggerStatus();
}

/* Remote */

void CM_EventHandle::remoteClean(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (ev.slam_error)
	{
		beeper.play_for_command(INVALID);
		remote.reset();
		return;
	}
	beeper.play_for_command(VALID);
	ev.key_clean_pressed = true;
	if(cm_is_navigation()){
		g_is_manual_pause = true;
	}
	remote.reset();
}

void CM_EventHandle::remoteHome(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_motion_init_succeeded && !cs.is_going_home() && !cm_should_self_check() && !ev.slam_error && !g_is_manual_pause) {

		if( SpotMovement::instance()->getSpotType()  == NORMAL_SPOT){
			beeper.play_for_command(INVALID);
		}
		else{
			ev.remote_home = true;
			beeper.play_for_command(VALID);
			if (SpotMovement::instance()->getSpotType() == CLEAN_SPOT){
				SpotMovement::instance()->spotDeinit();
			}
		}
		ROS_INFO("ev.remoteHome = %d", ev.remote_home);
	}
	else {
		beeper.play_for_command(INVALID);
		ROS_INFO("ev.remoteHome = %d", ev.remote_home);
	}
	remote.reset();
}

void CM_EventHandle::remoteSpot(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (!g_motion_init_succeeded || !cm_is_navigation()
		|| cs.is_going_home() || cm_should_self_check() || ev.slam_error || g_is_manual_pause
		|| time(NULL) - last_time_remote_spot < 3)
		beeper.play_for_command(INVALID);
	else
	{
		ev.remote_spot = true;
		last_time_remote_spot = time(NULL);
		beeper.play_for_command(VALID);
	}

	remote.reset();
}

void CM_EventHandle::remoteMax(bool state_now, bool state_last)
{
	ROS_WARN("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_motion_init_succeeded && !cs.is_going_home() && !cm_should_self_check() && SpotMovement::instance()->getSpotType() == NO_SPOT && !ev.slam_error && !g_is_manual_pause)
	{
		beeper.play_for_command(VALID);
		vacuum.switchToNext(true);
	}
	else
		beeper.play_for_command(INVALID);
	remote.reset();
}

/*
void CM_EventHandle::remote_direction(bool state_now,bool state_last)
{
	ROS_WARN("%s,%d: is called.",__FUNCTION__,__LINE__);
	// For Debug
	// ev.battrey_home = true;
	// path_set_continue_cell(nav_map.get_curr_cell());
	// robot::instance()->setLowBatPause();
	beeper.play_for_command(INVALID);
	remote.reset();
}

void CM_EventHandle::remoteDirectionLeft(bool state_now, bool state_last)
{
	remote_direction(state_now,state_last);
}
void CM_EventHandle::remoteDirectionRight(bool state_now, bool state_last){
	remote_direction(state_now,state_last);
}

void CM_EventHandle::remoteDirectionForward(bool state_now, bool state_last){
	remote_direction(state_now,state_last);
}
*/
/* Battery */
void CM_EventHandle::batteryHome(bool state_now, bool state_last)
{
	if (g_motion_init_succeeded && !cs.is_going_home()) {
		ROS_INFO("%s %d: low battery, battery =\033[33m %dmv \033[0m", __FUNCTION__, __LINE__,
						 battery.getVoltage());
		ev.battrey_home = true;

		if (vacuum.getMode() == Vac_Max) {
			vacuum.switchToNext(false);
		}
#if CONTINUE_CLEANING_AFTER_CHARGE
		if (SpotMovement::instance()->getSpotType() != NORMAL_SPOT ){
			path_set_continue_cell(nav_map.getCurrCell());
			g_is_low_bat_pause = true;
		}
#endif
    }
}

void CM_EventHandle::batteryLow(bool state_now, bool state_last)
{
    uint8_t         v_pwr, s_pwr, m_pwr;
    uint16_t        t_vol;

    ROS_DEBUG("%s %d: is called.", __FUNCTION__, __LINE__);

	if (g_battery_low_cnt++ > 50) {
		t_vol = battery.getVoltage();
		ROS_WARN("%s %d: low battery, battery < %umv is detected.", __FUNCTION__, __LINE__,t_vol);

		if (cs.is_going_home()) {
			v_pwr = Home_Vac_Power / t_vol;
			s_pwr = Home_SideBrush_Power / t_vol;
			m_pwr = Home_MainBrush_Power / t_vol;
		} else {
			v_pwr = Clean_Vac_Power / t_vol;
			s_pwr = Clean_SideBrush_Power / t_vol;
			m_pwr = Clean_MainBrush_Power / t_vol;
		}

		g_battery_low_cnt = 0;
		vacuum.bldcSpeed(v_pwr);
		brush.setSidePwm(s_pwr, s_pwr);
		brush.setMainPwm(m_pwr);

		ev.fatal_quit = true;
		ev.battery_low = true;
	}
}

void CM_EventHandle::chargeDetect(bool state_now, bool state_last)
{
	ROS_DEBUG("%s %d: Detect charger: %d, g_charge_detect_cnt: %d.", __FUNCTION__, __LINE__, charger.getChargeStatus(), g_charge_detect_cnt);
	if (((cm_is_exploration() || cm_is_go_charger() || cs.is_going_home()) && charger.getChargeStatus()) ||
		(!cm_is_exploration() && charger.getChargeStatus() == 3))
	{
		if (g_charge_detect_cnt++ > 2)
		{
			ev.charge_detect = charger.getChargeStatus();
			if (!cm_is_exploration() && charger.getChargeStatus() == 3)
				ev.fatal_quit = true;
			ROS_WARN("%s %d: ev.chargeDetect has been set to %d.", __FUNCTION__, __LINE__, ev.charge_detect);
			g_charge_detect_cnt = 0;
		}
	}
}
