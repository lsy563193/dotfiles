//
// Created by lsy563193 on 4/25/17.
//

#include <fcntl.h>
#include "motion_manage.h"
#include "movement.h"
#include "gyro.h"
#include "key.h"
#include "robot.hpp"
#include "wav.h"
#include "config.h"
#include "laser.hpp"
#include "slam.h"
#include "error.h"
#include "move_type.h"
#include <clean_state.h>
#include <vacuum.h>
#include <brush.h>
#include <remote.h>
#include <accelerator.h>
#include <tilt.h>
#include <led.h>
#include <charger.h>
#include "path_planning.h"
#include "core_move.h"
#include "event_manager.h"
#include "spot.h"
#include "move_type.h"
#include "robotbase.h"
#include "map.h"
#include "regulator.h"
#include "clean_mode.h"

//Segment_set segmentss;

uint32_t g_saved_work_time = 0;//temporary work time

/*
int g_enable_angle_offset = 0;
boost::mutex g_angle_offset_mt;
boost::mutex g_wait_mt;

int get_angle_offset(void)
{
	boost::mutex::scoped_lock lock(g_angle_offset_mt);
	return g_enable_angle_offset;
}
void set_angle_offset(int status)
{
	boost::mutex::scoped_lock lock(g_angle_offset_mt);
	g_enable_angle_offset = status;
}

boost::condition_variable g_cond_var;
*/
/*
void MotionManage::robot_obstacles_cb(const obstacle_detector::Obstacles::ConstPtr &msg)
{
//	ROS_WARN("robot_obstacles_cb");
	line_align_ = start;

	if (msg->segments.size() != 0)
	{
//			ROS_INFO("size = %d", msg->segments.size());
		for (auto s : msg->segments)
		{
			Point first(s.first_point.x, s.first_point.y);
			Point last(s.last_point.x, s.last_point.y);
			Segment seg(first, last);
//				std::cout << "seg: " << seg << std::endl;

			auto dist = seg.length();

//				if (dist < 1)
//					ROS_INFO("dist = %f", dist);
//				else
//					ROS_INFO("dist = %f >1", dist);

			if (dist < 1)
				continue;

			segmentss.classify(seg);
		}
	}

}
*/
/*
bool MotionManage::get_align_angle(float &line_angle)
{
	time_t start_time;
	bool eh_status_now=false, eh_status_last=false;

	segmentss.clear();
	ROS_INFO("Start subscribe to /obstacles");
	auto obstacles_sub = nh_.subscribe("/obstacles", 1, &MotionManage::robot_obstacles_cb, this);

	//wait for start obstacle_detector
	start_time = time(NULL);
	while (time(NULL) - start_time <= 10 && line_align_ != start)
	{
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			continue;
		}

		if (ev.fatal_quit || ev.key_clean_pressed || ev.cliff_all_triggered)
		{
			ROS_WARN("%s %d: Launch obstacle detector interrupted.", __FUNCTION__, __LINE__);
			return false;
		}
	}
	if (line_align_ != start)
	{
		ROS_WARN("%s %d: Obstacle detector launch timeout,align fail.", __FUNCTION__, __LINE__);
		line_angle = 0;
		return true;
	}

	ROS_DEBUG("%s %d: Obstacle detector launch finishd.", __FUNCTION__, __LINE__);

	//wait for detecting line
	start_time = time(NULL);
	while (time(NULL) - start_time <= 2)
	{
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			continue;
		}

		if (ev.fatal_quit || ev.key_clean_pressed || ev.cliff_all_triggered)
		{
			ROS_WARN("%s %d: Detecting line interrupted.", __FUNCTION__, __LINE__);
			return false;
		}
	}

	ROS_DEBUG("%s %d: Get the line", __FUNCTION__, __LINE__);
//	auto line_angle = static_cast<int16_t>(segmentss.min_distant_segment_angle() *10);
	line_angle = segmentss.min_distant_segment_angle();
	// If get line_angle from the scan data, turn 180 degrees.
	// Else, the line_angle should be 0(Actually there is very little chance that the line_angle from scan data is exactly 0).
	if (line_angle > 0)
	{
		line_angle -= 180;
	} else if (line_angle < 0)
	{
		line_angle += 180;
	}
//	return line_angle;
	return true;
}
*/

Slam* MotionManage::s_slam = nullptr/*new Slam()*/;

void init_nav_before_gyro()
{
	if (ev.remote_home || g_go_home_by_remote)
		led.set_mode(LED_FLASH, LED_ORANGE, 1000);
	else
		led.set_mode(LED_FLASH, LED_GREEN, 1000);

	// Initialize motors and map.
	if (!cs_is_paused() && !g_is_low_bat_pause && !g_resume_cleaning )
	{
		g_saved_work_time = 0;
		ROS_INFO("%s ,%d ,set g_saved_work_time to zero ", __FUNCTION__, __LINE__);
		// Push the start point into the home point list
		ROS_INFO("map_init-----------------------------");

		path_planning_initialize();

		robot::instance()->initOdomPosition();

		// If it it the first time cleaning, initialize the g_continue_point.
		extern Cell_t g_continue_cell;
		g_continue_cell.X = g_continue_cell.Y = 0;
		g_have_seen_charger = false;
		g_start_point_seen_charger = false;

		g_homes.resize(1,g_zero_home);
		g_home_gen_rosmap = true;
		g_home_way_list.clear();
	}

	key.reset();

	gyro.setOff();
	usleep(30000);
	gyro.setOn();

	c_rcon.reset_status();
	key.reset();
	// Can't register until the status has been checked. because if register too early, the handler may affect the pause status, so it will play the wrong wav.
	if (g_resume_cleaning)
	{
		ROS_WARN("Restore from low battery pause");
		cm_register_events();
		wav.play(WAV_CLEANING_CONTINUE);
	}
	else if (cs_is_paused())
	{
		ROS_WARN("Restore from manual pause");
		cm_register_events();
		wav.play(WAV_CLEANING_CONTINUE);
		if (cs.is_going_home())
		{
			wav.play(WAV_BACK_TO_CHARGER);
		}
	}
	else if(g_plan_activated == true)
	{
		cm_register_events();
		wav.play(WAV_PLAN_CLEANING_START);
		g_plan_activated = false;
	}
	else{
		cm_register_events();
		wav.play(WAV_CLEANING_START);
	}
}
void init_nav_gyro_charge()
{
	if (cs_is_paused() || g_resume_cleaning )
	{
		robot::instance()->offsetAngle(robot::instance()->savedOffsetAngle());
		ROS_WARN("%s %d: Restore the gyro angle(%f).", __FUNCTION__, __LINE__, -robot::instance()->savedOffsetAngle());
		if (!cs.is_going_home())
			if(ev.remote_home || ev.battery_home)
				cs.set(CS_GO_HOME_POINT);
	}
}
void init_nav_after_charge()
{
acc.setAccInitData();//about 200ms delay
	tilt.enable(true);
	ROS_INFO("\033[35m" "%s,%d,enable tilt detect" "\033[0m",__FUNCTION__,__LINE__);

	cs_work_motor();

	ROS_INFO("%s %d: Init cs.is_going_home()(%d), lowbat(%d), manualpaused(%d), g_resume_cleaning(%d),g_robot_stuck(%d)", __FUNCTION__, __LINE__,
					 cs.is_going_home(), g_is_low_bat_pause, g_is_manual_pause, g_resume_cleaning,g_robot_stuck);
}

void init_exp_before_gyro()
{

	if (ev.remote_home || g_go_home_by_remote)
		led.set_mode(LED_FLASH, LED_ORANGE, 1000);
	else
		led.set_mode(LED_FLASH, LED_GREEN, 1000);

	// Initialize motors and map.
	g_saved_work_time = 0;
	ROS_INFO("%s ,%d ,set g_saved_work_time to zero ", __FUNCTION__, __LINE__);
	// Push the start point into the home point list
	path_planning_initialize();

	robot::instance()->initOdomPosition();

	// If it it the first time cleaning, initialize the g_continue_point.
	g_have_seen_charger = false;
	g_start_point_seen_charger = false;

	g_homes.resize(1,g_zero_home);
	g_home_gen_rosmap = true;
	g_home_way_list.clear();

	key.reset();

	gyro.setOff();
	usleep(30000);
	gyro.setOn();

	c_rcon.reset_status();
	key.reset();
	// Can't register until the status has been checked. because if register too early, the handler may affect the pause status, so it will play the wrong wav.
	cm_register_events();
	wav.play(WAV_EXPLORATION_START);

}
void init_exp_after_gyro()
{
	acc.setAccInitData();//about 200ms delay
	tilt.enable(true);
	ROS_INFO("\033[47;35m" "%s,%d,enable tilt detect" "\033[0m",__FUNCTION__,__LINE__);

	cs_work_motor();
}

void init_wf_before_gyro()
{
	cm_register_events();
	led.set_mode(LED_FLASH, LED_GREEN, 1000);

	g_wf_start_timer = time(NULL);
	g_wf_diff_timer = WALL_FOLLOW_TIME;
	remote.reset_move_with();
	c_rcon.reset_status();
	key.reset();
	key.reset();
	gyro.setOff();
	usleep(30000);
	gyro.setOn();

	wav.play(WAV_CLEANING_WALL_FOLLOW);
}

void init_wf_after_gyro()
{
		// enable titlt detct
	acc.setAccInitData();//about 200ms delay
	tilt.enable(true);
	ROS_INFO("\033[47;35m" "%s,%d,enable tilt detect" "\033[0m",__FUNCTION__,__LINE__);

	g_saved_work_time = 0;
	ROS_INFO("%s ,%d ,set g_saved_work_time to zero ", __FUNCTION__, __LINE__);
	wf_path_planning_initialize();
	ROS_WARN("%s %d: path planning initialized", __FUNCTION__, __LINE__);
	//pthread_t	escape_thread_id;
	robot::instance()->initOdomPosition();// for reset odom position to zero.

	g_homes.resize(1,g_zero_home);
	g_home_gen_rosmap = true;
	g_home_way_list.clear();
	g_have_seen_charger = false;
	cs_work_motor();

}

void init_spot_before_gyro()
{
		cm_register_events();
	led.set_mode(LED_FLASH, LED_GREEN, 1000);

	c_rcon.reset_status();
	remote.reset_move_with();
	key.reset();

	gyro.setOff();
	usleep(30000);
	gyro.setOn();

	wav.play(WAV_CLEANING_SPOT);
}

void init_spot_after_gyro()
{
		// enable titlt detct
	acc.setAccInitData();//about 200ms delay
	tilt.enable(true);
	ROS_INFO("\033[33m" "%s,%d,enable tilt detect" "\033[0m",__FUNCTION__,__LINE__);

	g_saved_work_time = 0;
	ROS_INFO("%s ,%d ,set g_saved_work_time to zero ", __FUNCTION__, __LINE__);

	robot::instance()->initOdomPosition();// for reset odom position to zero.

	g_homes.resize(1,g_zero_home);
	g_home_gen_rosmap = true;
	g_home_way_list.clear();

	vacuum.mode(Vac_Max);
	brush.set_main_pwm(80);
	brush.set_side_pwm(60, 60);

}

void init_go_home_before_gyro()
{
	led.set_mode(LED_FLASH, LED_ORANGE, 1000);
	gyro.setOff();
	usleep(30000);
	gyro.setOn();
	key.reset();
	cm_register_events();
	wav.play(WAV_BACK_TO_CHARGER);
}
void init_go_home_after_gyro()
{
	cs_work_motor();
	c_rcon.reset_status();

}
bool wait_for_back_from_charge()
{
	ROS_INFO("%s %d: calling moving back", __FUNCTION__, __LINE__);
		auto curr = cost_map.get_curr_cell();
		path_set_home(curr);
		extern bool g_from_station;
		g_from_station = 1;

	brush.set_side_pwm(30, 30);
		int back_segment = MOVE_BACK_FROM_STUB_DIST/SIGMENT_LEN;
		for (int i = 0; i < back_segment; i++) {
			quick_back(20,SIGMENT_LEN);
			if (ev.fatal_quit || ev.key_clean_pressed || charger.is_on_stub() || ev.cliff_all_triggered) {
				cs_disable_motors();
				if (ev.fatal_quit)
				{
					g_is_manual_pause = false;
					g_resume_cleaning = false;
				}
				else if (ev.key_clean_pressed && !g_resume_cleaning)
					// Reset the odom position so when continue cleaning, the position robot stopped at will be the home point (0, 0).
					robot::instance()->initOdomPosition();
				else if (!ev.fatal_quit && !ev.key_clean_pressed)
				{
					ROS_WARN("%s %d: Fail to leave charger stub.", __FUNCTION__, __LINE__);
					g_is_manual_pause = false;
					g_resume_cleaning = false;
				}
				return false;
			}
		}
	wheel.stop();
		robot::instance()->initOdomPosition();
	return true;
}

void init_before_gyro()
{
	mt.set(cm_is_follow_wall() ? MT_FOLLOW_LEFT_WALL : MT_LINEARMOVE);
	if(!cs_is_paused())
		g_from_station = 0;
	g_motion_init_succeeded = false;

	bool remote_home_during_pause = false;
	if (cs_is_paused() && ev.remote_home)
		remote_home_during_pause = true;
	event_manager_reset_status();
	if (remote_home_during_pause)
	{
		ev.remote_home = true;
		ROS_INFO("%s %d: Resume remote home.", __FUNCTION__, __LINE__);
	}

	reset_work_time();
	if (!cs_is_paused() && !g_is_low_bat_pause && !g_resume_cleaning )
		cost_map.reset(MAP);

	fw_map.reset(MAP);
	ros_map.reset(MAP);
	ros2_map.reset(MAP);
	switch (cm_get())
	{
		case Clean_Mode_Navigation:
			init_nav_before_gyro();
			break;
		case Clean_Mode_Exploration:
			init_exp_before_gyro();
			break;
		case Clean_Mode_WallFollow:
			init_wf_before_gyro();
			break;
		case Clean_Mode_Spot:
			init_spot_before_gyro();
			break;
		case Clean_Mode_Go_Charger:
			init_go_home_before_gyro();
			break;
		default:
			ROS_ERROR("This mode_ (%d) should not use MotionManage.", cm_get());
			break;
	}
}

bool MotionManage::laser_init()
{
	s_laser = new Laser();
	s_laser->laserMotorCtrl(ON);
	if (s_laser->isScanReady() == -1)
	{
		ROS_ERROR("%s %d: Laser opening failed.", __FUNCTION__, __LINE__);
		error.set(Error_Code_Laser);
		initSucceeded(false);
		return false;
	}
	else if (s_laser->isScanReady() == 0)
	{
		initSucceeded(false);
		return false;
	}
}

void MotionManage::get_aligment_angle()
{
	/*--- get aligment angle-----*/
	if( !(cs_is_paused() || g_resume_cleaning ))
	{
		nh_.param<bool>("is_active_align", is_align_active_, false);
		if (cm_is_navigation() && is_align_active_)
		{
			//ObstacleDetector od;
			std::vector<LineABC> lines;
			time_t time_findline = time(NULL);
			ROS_INFO("%s,%d,ready to find lines ",__FUNCTION__,__LINE__);
			float align_angle = 0.0;
			while(1){
				if(s_laser->findLines(&lines)){
					if(s_laser->getAlignAngle(&lines,&align_angle))
						break;
				}
				if(difftime(time(NULL) ,time_findline) >= 2){
					ROS_INFO("%s,%d,find lines timeout",__FUNCTION__,__LINE__);
					break;
				}
			}

			align_angle += (float)(LIDAR_THETA / 10);
			robot::instance()->offsetAngle(align_angle);
			ROS_INFO("%s %d: align_angle angle (%f).", __FUNCTION__, __LINE__,align_angle);
			g_homes[0].TH = -(int16_t)(align_angle);
			ROS_INFO("%s %d: g_homes[0].TH (%d).", __FUNCTION__, __LINE__, g_homes[0].TH);
		}
//		robot::instance()->startAngle(0);
//		g_homes[0].TH=0;
	}
}

bool MotionManage::slam_init()
{
		s_slam = new Slam();
	//4 call start slam
	while (ev.slam_error)
	{
		// Wait for slam launch.
		usleep(20000);
	}

	robot::instance()->setTfReady(false);
	if (cm_is_navigation() || cm_get() == Clean_Mode_Spot || cm_is_exploration())
		robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle);
	else if (cm_is_follow_wall())
		robot::instance()->setBaselinkFrameType(Map_Position_Odom_Angle);
	s_slam->enableMapUpdate();
	auto count_n_10ms = 500;

	bool eh_status_now=false, eh_status_last=false;
	while (ros::ok() && !(s_slam->isMapReady() && robot::instance()->isTfReady()) && --count_n_10ms != 0)
	{
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			continue;
		}

		if (ev.fatal_quit || ev.key_clean_pressed || ev.cliff_all_triggered)
		{
			ROS_WARN("%s %d: Waiting for slam interrupted.", __FUNCTION__, __LINE__);
			break;
		}

		usleep(20000);
	}
	if (count_n_10ms == 0)
	{
		ROS_ERROR("%s %d: Map or tf framework is still not ready after 10s, timeout and return.", __FUNCTION__, __LINE__);
		error.set(Error_Code_Slam);
		wav.play(WAV_TEST_LIDAR);
		initSucceeded(false);
		return false;
	}
	return true;
}

void MotionManage::init_after_slam()
{
	ev.rcon_triggered = ev.bumper_triggered =  ev.obs_triggered  = 0;
	/*--- slam end ---*/

	if (g_go_home_by_remote || (cm_is_exploration()))
		led.set_mode(LED_STEADY, LED_ORANGE);
	else
		led.set_mode(LED_STEADY, LED_GREEN);

		g_robot_slip_enable = true;
	g_robot_stuck = false;
	g_robot_slip = false;
}

MotionManage::MotionManage():nh_("~"),is_align_active_(false)
{
	init_before_gyro();
	initSucceeded(true);

	if (!gyro.waitForOn())
		return;

	switch (cm_get())
	{
		case Clean_Mode_Navigation:
			init_nav_gyro_charge();
			if(charger.is_on_stub() && !wait_for_back_from_charge())
				return;
			init_nav_after_charge();
			break;
		case Clean_Mode_Exploration:
			init_exp_after_gyro();
			break;
		case Clean_Mode_WallFollow:
			init_wf_after_gyro();
			break;
		case Clean_Mode_Spot:
			init_spot_after_gyro();
			break;
		case Clean_Mode_Go_Charger:
			init_go_home_after_gyro();
			break;
		default:
			ROS_ERROR("This mode_ (%d) should not use MotionManage.", cm_get());
			break;
	}
	// No need to start laser or slam if it is go home mode_.
	if (cm_get() == Clean_Mode_Go_Charger)
		return;

	/*--- laser init ---*/
	if(!laser_init())
		return;

	if (g_is_low_bat_pause || g_resume_cleaning)
	{
		robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle);
		if (g_go_home_by_remote)
			led.set_mode(LED_STEADY, LED_ORANGE);
		else
			led.set_mode(LED_STEADY, LED_GREEN);

		return;
	}
	if (cs_is_paused())
	{
		g_is_manual_pause = false;
		g_robot_stuck = false;
		if (s_slam != nullptr)
		{
			robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle);
			if (g_go_home_by_remote)
				led.set_mode(LED_STEADY, LED_ORANGE);
			else
				led.set_mode(LED_STEADY, LED_GREEN);
			return;
		}
	}

	if(g_from_station)
	{
		robot::instance()->offsetAngle(180);
		ROS_INFO("%s,%d,\033[32m charge stub postion estiamate on(%d,%d)\033[0m",__FUNCTION__,__LINE__,(-1)*(int)MOVE_BACK_FROM_STUB_DIST/CELL_SIZE,0);
		Cell_t home_point((-1)*(int)MOVE_BACK_FROM_STUB_DIST/CELL_SIZE,0);
		cost_map.set_charge_position(home_point);
		g_homes[0].TH = 180;
	}

	get_aligment_angle();

	usleep(600000);// wait for tf ready

	/*----slam init----*/
	if(!slam_init())
		return;
	init_after_slam();
}

MotionManage::~MotionManage()
{
	if (cm_get() != Clean_Mode_Go_Charger)
	{
		cost_map.print(MAP, cost_map.get_x_cell(), cost_map.get_y_cell());
		//if (cm_is_follow_wall())
		g_wf_reach_count = 0;
		if (SpotMovement::instance()->getSpotType() != NO_SPOT)
		//if (cm_get() == Clean_Mode_Spot)
		{
			SpotMovement::instance()->spotDeinit();// clear the variables.
		}
	}
	// Disable motor here because there ie a cs_work_motor() in spotDeinit().
	cs_disable_motors();
	tilt.enable(false);
	g_robot_slip_enable =false;
	ROS_INFO("\033[35m" "disable tilt detect & robot stuck detect" "\033[0m");

	robot::instance()->setBaselinkFrameType(Odom_Position_Odom_Angle);

	if (s_laser != nullptr)
	{
		delete s_laser; // It takes about 1s.
		s_laser = nullptr;
	}
	if (!ev.fatal_quit && ( ( ev.key_clean_pressed && cs_is_paused() ) || g_robot_stuck ) )
	{
		wav.play(WAV_CLEANING_PAUSE);
		if (!ev.cliff_all_triggered)
		{
			if (cs.is_going_home())
			{
				// The current home cell is still valid, so push it back to the home point list.
				path_set_home(g_home_point);
			}
			cm_set(Clean_Mode_Idle);
			robot::instance()->savedOffsetAngle(robot::instance()->getPoseAngle());
			ROS_INFO("%s %d: Save the gyro angle(\033[32m%f\033[0m) before pause.", __FUNCTION__, __LINE__, robot::instance()->getPoseAngle());
			if (cs.is_going_home())
#if MANUAL_PAUSE_CLEANING
//				ROS_WARN("%s %d: Pause going home, g_homes list size: %u, g_new_homes list size: %u.", __FUNCTION__, __LINE__, (uint)g_homes.size(), (uint)g_new_homes.size());
				ROS_WARN("%s %d: Pause going home", __FUNCTION__, __LINE__);
#else
				ROS_WARN("%s %d: Clean key pressed. Finish cleaning.", __FUNCTION__, __LINE__);
#endif
			else
				ROS_INFO("%s %d: Pause cleanning.", __FUNCTION__, __LINE__);
			g_saved_work_time += get_work_time();
			ROS_INFO("%s %d: Cleaning time: \033[32m%d(s)\033[0m", __FUNCTION__, __LINE__, g_saved_work_time);
			cm_unregister_events();
			return;
		}
		else
			ROS_WARN("%s %d: Robot lifted up.", __FUNCTION__, __LINE__);
	}
	else{
		g_from_station = 0;
	}

	if (!ev.charge_detect)
		// It means robot can not go to charger stub.
		g_is_low_bat_pause = false;

	if (!ev.fatal_quit && g_is_low_bat_pause)
	{
		wav.play(WAV_CLEANING_PAUSE);
		if (!ev.cliff_all_triggered)
		{
			g_resume_cleaning = true;
			g_is_low_bat_pause = false;
			cm_set(Clean_Mode_Charging);
			robot::instance()->savedOffsetAngle(robot::instance()->getPoseAngle());
			ROS_WARN("%s %d: Save the gyro angle(%f) before pause.", __FUNCTION__, __LINE__, robot::instance()->getPoseAngle());
			ROS_WARN("%s %d: Pause cleaning for low battery, will continue cleaning when charge finished.", __FUNCTION__, __LINE__);
			g_saved_work_time += get_work_time();
			ROS_INFO("%s %d: Cleaning time:\033[32m%d(s)\033[0m", __FUNCTION__, __LINE__, g_saved_work_time);
			cm_unregister_events();

			cm_reset_go_home();
			return;
		}
		else
			ROS_WARN("%s %d: Robot lifted up.", __FUNCTION__, __LINE__);
	}

	// Unregister here because robot may be lifted up during the wav playing for cleaning pause.
	cm_unregister_events();
	if (ev.fatal_quit) // Also handles for ev.battery_low/ev.charge_detect/ev.cliff_all_triggered.
	{
		g_is_manual_pause = false;
		g_is_low_bat_pause = false;
		g_resume_cleaning = false;
		if (ev.cliff_all_triggered)
			wav.play(WAV_ERROR_LIFT_UP);
		wav.play(WAV_CLEANING_STOP);
	}
	else // Normal finish.
	{
		if(cs.is_going_home() && !ev.charge_detect && g_have_seen_charger)
			wav.play(WAV_BACK_TO_CHARGER_FAILED);
		if (cm_get() != Clean_Mode_Go_Charger && !cm_is_exploration())
			wav.play(WAV_CLEANING_FINISHED);
	}
	cm_reset_go_home();

	if (s_slam != nullptr)
	{
		delete s_slam;
		s_slam = nullptr;
	}

	robot::instance()->savedOffsetAngle(0);

	if (ev.fatal_quit)
		if (ev.cliff_all_triggered)
			ROS_WARN("%s %d: All Cliff are triggered. Finish cleaning.", __FUNCTION__, __LINE__);
		else
			ROS_WARN("%s %d: Fatal quit and finish cleanning.", __FUNCTION__, __LINE__);
	else if (ev.key_clean_pressed)
		ROS_WARN("%s %d: Key clean pressed. Finish cleaning.", __FUNCTION__, __LINE__);
	else if (ev.charge_detect)
		ROS_WARN("%s %d: Finish cleaning and stop in charger stub.", __FUNCTION__, __LINE__);
	else if (ev.battery_low)
		ROS_WARN("%s %d: Battery too low. Finish cleaning.", __FUNCTION__, __LINE__);
	else
		if (cm_get() == Clean_Mode_Spot)
			ROS_WARN("%s %d: Finish cleaning.", __FUNCTION__, __LINE__);
		else if (cm_get() == Clean_Mode_Go_Charger)
			ROS_WARN("%s %d: Could not go to charger stub.", __FUNCTION__, __LINE__);
		else
			ROS_WARN("%s %d: Can not go to charger stub after going to all home cells. Finish cleaning.", __FUNCTION__, __LINE__);

	if (cm_get() != Clean_Mode_Go_Charger)
	{
		g_saved_work_time += get_work_time();
		auto cleaned_count = cost_map.get_cleaned_area();
		auto map_area = cleaned_count * (CELL_SIZE * 0.001) * (CELL_SIZE * 0.001);
		ROS_INFO("%s %d: Cleaned area = \033[32m%.2fm2\033[0m, cleaning time: \033[32m%d(s) %.2f(min)\033[0m, cleaning speed: \033[32m%.2f(m2/min)\033[0m.", __FUNCTION__, __LINE__, map_area, g_saved_work_time, double(g_saved_work_time) / 60, map_area / (double(g_saved_work_time) / 60));
	}
	if (ev.battery_low)
		cm_set(Clean_Mode_Sleep);
	else if (ev.charge_detect)
		cm_set(Clean_Mode_Charging);
	else
		cm_set(Clean_Mode_Idle);
}

bool MotionManage::initCleaning(uint8_t cleaning_mode)
{

	switch (cleaning_mode)
	{
		case Clean_Mode_Navigation:
			return initNavigationCleaning();
		case Clean_Mode_Exploration:
			return initExplorationCleaning();
		case Clean_Mode_WallFollow:
			return initWallFollowCleaning();
		case Clean_Mode_Spot:
			return initSpotCleaning();
		case Clean_Mode_Go_Charger:
			return initGoHome();
		default:
			ROS_ERROR("This mode_ (%d) should not use MotionManage.", cleaning_mode);
			return false;
	}
}

bool MotionManage::initNavigationCleaning(void)
{
	init_nav_before_gyro();

	if (!gyro.waitForOn())
		return false;

	init_nav_gyro_charge();

	/*Move back from charge station*/
	if (charger.is_on_stub()) {
		if (!wait_for_back_from_charge())
			return false;
	}

	init_nav_after_charge();

	return true;
}

bool MotionManage::initExplorationCleaning(void)
{
	init_exp_before_gyro();

	if (!gyro.waitForOn())
		return false;

	init_exp_after_gyro();

	return true;
}

bool MotionManage::initWallFollowCleaning(void)
{
	init_wf_before_gyro();

	if (!gyro.waitForOn())
	{
		return false;
	}
	init_wf_after_gyro();

	return true;
}

bool MotionManage::initSpotCleaning(void)
{

	init_spot_before_gyro();

	if (!gyro.waitForOn())
	{
		return false;
	}
	init_spot_after_gyro();

	return true;
}

bool MotionManage::initGoHome(void)
{
	init_go_home_before_gyro();
	if (!gyro.waitForOn())
		return false;
	init_go_home_after_gyro();
	return true;
}

void MotionManage::pubCleanMapMarkers(CostMap& map, const std::deque<Cell_t>& path, Cell_t* cell_p)
{
	// temp_target is valid if only path is not empty.
	if (path.empty())
		return;

	int16_t x, y, x_min, x_max, y_min, y_max;
	CellState cell_state;
	Cell_t next = path.front();
	Cell_t target = path.back();
	map.path_get_range(MAP, &x_min, &x_max, &y_min, &y_max);

	if (next.X == SHRT_MIN )
		next.X = x_min;
	else if (next.X == SHRT_MAX)
		next.X = x_max;

	for (x = x_min; x <= x_max; x++)
	{
		for (y = y_min; y <= y_max; y++)
		{
			if (x == target.X && y == target.Y)
				robot::instance()->setCleanMapMarkers(x, y, TARGET_CLEAN);
			else if (x == next.X && y == next.Y)
				robot::instance()->setCleanMapMarkers(x, y, TARGET);
			else if (cell_p != nullptr && x == (*cell_p).X && y == (*cell_p).Y)
				robot::instance()->setCleanMapMarkers(x, y, TARGET_CLEAN);
			else
			{
				cell_state = cost_map.get_cell(MAP, x, y);
				if (cell_state > UNCLEAN && cell_state < BLOCKED_BOUNDARY )
					robot::instance()->setCleanMapMarkers(x, y, cell_state);
			}
		}
	}
#if LINEAR_MOVE_WITH_PATH
	if (!path.empty())
	{
		for (auto it = path.begin(); it->X != path.back().X || it->Y != path.back().Y; it++)
			robot::instance()->setCleanMapMarkers(it->X, it->Y, TARGET);

		robot::instance()->setCleanMapMarkers(path.back().X, path.back().Y, TARGET_CLEAN);
	}
#endif

	robot::instance()->pubCleanMapMarkers();
}

