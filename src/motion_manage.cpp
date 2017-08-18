//
// Created by lsy563193 on 4/25/17.
//

#include <movement.h>
#include <gyro.h>
#include <robot.hpp>
#include <wav.h>
#include <config.h>
#include <laser.hpp>
#include <fcntl.h>

#include "obstacle_detector.cpp"

#include "motion_manage.h"
#include <segment_set.h>
#include <slam.h>
#include <move_type.h>
#include "path_planning.h"
#include "core_move.h"
#include "event_manager.h"
#include "spot.h"
#include "move_type.h"
#include "wall_follow_slam.h"
#include "robotbase.h"

Segment_set segmentss;

extern std::list <Cell_t> g_home_point_old_path;
extern std::list <Cell_t> g_home_point_new_path;

uint32_t g_saved_work_time = 0;//temporary work time

bool g_is_main_switch_off = false;
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

		if (g_fatal_quit_event || g_key_clean_pressed || g_cliff_all_triggered)
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

		if (g_fatal_quit_event || g_key_clean_pressed || g_cliff_all_triggered)
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

Laser* MotionManage::s_laser = nullptr/*new Laser()*/;
Slam* MotionManage::s_slam = nullptr/*new Slam()*/;

MotionManage::MotionManage():nh_("~"),is_align_active_(false)
{
	mt_set(get_clean_mode() == Clean_Mode_WallFollow ? CM_FOLLOW_LEFT_WALL : CM_LINEARMOVE);
	g_from_station = 0;
	g_trapped_mode = 0;
	g_finish_cleaning_go_home = false;
	g_motion_init_succeeded = false;
	bool remote_home_during_pause = false;
	if (robot::instance()->isManualPaused() && g_remote_home)
		remote_home_during_pause = true;
	event_manager_reset_status();
	if (remote_home_during_pause)
	{
		g_remote_home = true;
		ROS_INFO("%s %d: Resume remote home.", __FUNCTION__, __LINE__);
	}
	g_turn_angle = 0;
	bool eh_status_now=false, eh_status_last=false;

	initSucceeded(true);

	
	//1 Initialize for different mode.
	if (!initCleaning(get_clean_mode()))
	{
		initSucceeded(false);
		return;
	}

	//2 start laser
	s_laser = new Laser();
	if (s_laser->isScanReady() == -1)
	{
		ROS_ERROR("%s %d: Laser opening failed.", __FUNCTION__, __LINE__);
		set_error_code(Error_Code_Laser);
		initSucceeded(false);
		return;
	}
	else if (s_laser->isScanReady() == 0)
	{
		initSucceeded(false);
		return;
	}

	if (robot::instance()->isLowBatPaused() || g_resume_cleaning)
	{
		robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle);
		s_laser->lidarShieldDetect(ON);
		if (g_go_home_by_remote)
			set_led_mode(LED_STEADY, LED_ORANGE);
		else
			set_led_mode(LED_STEADY, LED_GREEN);
		return;
	}
	if (robot::instance()->isManualPaused())
	{
		robot::instance()->resetManualPause();
		if (s_slam != nullptr)
		{
			robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle);
			s_laser->lidarShieldDetect(ON);
			if (g_go_home_by_remote)
				set_led_mode(LED_STEADY, LED_ORANGE);
			else
				set_led_mode(LED_STEADY, LED_GREEN);
			return;
		}
	}

	//3 calculate offsetAngle
	if(g_from_station)
	{
		robot::instance()->offsetAngle(180);
	}
	else
	{
		nh_.param<bool>("is_active_align", is_align_active_, false);
		if (get_clean_mode() == Clean_Mode_Navigation && is_align_active_)
		{
			ObstacleDetector od;
			float align_angle = 0;
			if (!get_align_angle(align_angle))
			{
				initSucceeded(false);
				return;
			}
			robot::instance()->offsetAngle(align_angle);
		}
	}

	ROS_INFO("waiting 1s for translation odom_to_robotbase work");
	sleep(1); //wait for odom_pub send translation(odom->robotbase) to slam_karto,

	//4 call start slam
	while (g_slam_error)
	{
		// Wait for slam launch.
		usleep(20000);
	}

	s_slam = new Slam();

	robot::instance()->setTfReady(false);
	if (get_clean_mode() == Clean_Mode_Navigation || get_clean_mode() == Clean_Mode_Spot)
		robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle);
	else if (get_clean_mode() == Clean_Mode_WallFollow)
		robot::instance()->setBaselinkFrameType(Map_Position_Odom_Angle);
	s_slam->enableMapUpdate();
	auto count_n_10ms = 500;

	while (ros::ok() && !(s_slam->isMapReady() && robot::instance()->isTfReady()) && --count_n_10ms != 0)
	{
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			continue;
		}

		if (g_fatal_quit_event || g_key_clean_pressed || g_cliff_all_triggered)
		{
			ROS_WARN("%s %d: Waiting for slam interrupted.", __FUNCTION__, __LINE__);
			break;
		}

		usleep(20000);
	}
	if (count_n_10ms == 0)
	{
		ROS_ERROR("%s %d: Map or tf framework is still not ready after 10s, timeout and return.", __FUNCTION__, __LINE__);
		set_error_code(Error_Code_Slam);
		wav_play(WAV_TEST_LIDAR);
		initSucceeded(false);
		return;
	}
	//s_laser->startShield();
	s_laser->lidarShieldDetect(ON);
	g_rcon_triggered = g_bumper_triggered =  g_obs_triggered  = 0;


	if (g_go_home_by_remote)
		set_led_mode(LED_STEADY, LED_ORANGE);
	else
		set_led_mode(LED_STEADY, LED_GREEN);

}

MotionManage::~MotionManage()
{
	ROS_WARN("cleaned area = %.2fm2", map_get_area());
	//if (get_clean_mode() == Clean_Mode_WallFollow)
	wf_clear();
	if (SpotMovement::instance()->getSpotType() != NO_SPOT)
	//if (get_clean_mode() == Clean_Mode_Spot)
	{
		SpotMovement::instance()->spotDeinit();// clear the variables.
		//sleep(1);
	}
	// Disable motor here because there is a work_motor_configure() in spotDeinit().
	disable_motors();

	g_tilt_enable = false;
	ROS_INFO("\033[47;35m" "disable tilt detect" "\033[0m");

	robot::instance()->setBaselinkFrameType(Odom_Position_Odom_Angle);

	if (s_laser != nullptr)
	{
		delete s_laser; // It takes about 1s.
		s_laser = nullptr;
	}

	if (!g_fatal_quit_event && g_key_clean_pressed && robot::instance()->isManualPaused())
	{
		wav_play(WAV_CLEANING_PAUSE);
		if (!g_cliff_all_triggered)
		{
			extern bool g_go_home;
			if (g_go_home)
			{
				extern Cell_t g_current_home_cell;
				// The current home cell is still valid, so push it back to the home point list.
				path_set_home(g_current_home_cell);
			}
			set_clean_mode(Clean_Mode_Userinterface);
			robot::instance()->savedOffsetAngle(robot::instance()->getAngle());
			ROS_INFO("%s %d: Save the gyro angle(%f) before pause.", __FUNCTION__, __LINE__, robot::instance()->getAngle());
			if (g_go_home)
#if MANUAL_PAUSE_CLEANING
				ROS_WARN("%s %d: Pause going home, g_home_point_old_path list size: %u, g_home_point_new_path list size: %u.", __FUNCTION__, __LINE__, (uint)g_home_point_old_path.size(), (uint)g_home_point_new_path.size());
#else
				ROS_WARN("%s %d: Clean key pressed. Finish cleaning.", __FUNCTION__, __LINE__);
#endif
			else
				ROS_INFO("%s %d: Pause cleanning.", __FUNCTION__, __LINE__);
			g_saved_work_time += get_work_time();
			ROS_INFO("%s %d: Cleaning time: %d(s)", __FUNCTION__, __LINE__, g_saved_work_time);
			cm_unregister_events();
			return;
		}
		else
			ROS_WARN("%s %d: Robot lifted up.", __FUNCTION__, __LINE__);
	}


	if (!g_fatal_quit_event && robot::instance()->isLowBatPaused())
	{
		wav_play(WAV_CLEANING_PAUSE);
		if (!g_cliff_all_triggered)
		{
			g_resume_cleaning = true;
			robot::instance()->resetLowBatPause();
			set_clean_mode(Clean_Mode_Charging);
			robot::instance()->savedOffsetAngle(robot::instance()->getAngle());
			ROS_WARN("%s %d: Save the gyro angle(%f) before pause.", __FUNCTION__, __LINE__, robot::instance()->getAngle());
			ROS_WARN("%s %d: Pause cleaning for low battery, will continue cleaning when charge finished.", __FUNCTION__, __LINE__);
			g_saved_work_time += get_work_time();
			ROS_WARN("%s %d: Cleaning time: %d(s)", __FUNCTION__, __LINE__, g_saved_work_time);
			cm_unregister_events();

			cm_reset_go_home();
			return;
		}
		else
			ROS_WARN("%s %d: Robot lifted up.", __FUNCTION__, __LINE__);
	}

	// Unregister here because robot may be lifted up during the wav playing for cleaning pause.
	cm_unregister_events();
	if (g_fatal_quit_event) // Also handles for g_battery_low/g_charge_detect/g_cliff_all_triggered.
	{
		robot::instance()->resetManualPause();
		robot::instance()->resetLowBatPause();
		g_resume_cleaning = false;
		if (g_cliff_all_triggered)
		{
			if(g_is_main_switch_off)
				wav_play(WAV_CHECK_SWITCH);
			else
				wav_play(WAV_ERROR_LIFT_UP);
		}
		wav_play(WAV_CLEANING_STOP);
	}
	else // Normal finish.
	{
		extern bool g_have_seen_charge_stub;
		if(g_go_home && !g_charge_detect && g_have_seen_charge_stub)
			wav_play(WAV_BACK_TO_CHARGER_FAILED);
		wav_play(WAV_CLEANING_FINISHED);
	}
	cm_reset_go_home();

	if (s_slam != nullptr)
	{
		delete s_slam;
		s_slam = nullptr;
	}


	g_home_point_old_path.clear();
	g_home_point_new_path.clear();
	robot::instance()->savedOffsetAngle(0);

	if (g_fatal_quit_event)
		if (g_cliff_all_triggered)
			ROS_WARN("%s %d: All Cliff are triggered. Finish cleaning.", __FUNCTION__, __LINE__);
		else
			ROS_WARN("%s %d: Fatal quit and finish cleanning.", __FUNCTION__, __LINE__);
	else if (g_key_clean_pressed)
		ROS_WARN("%s %d: Key clean pressed. Finish cleaning.", __FUNCTION__, __LINE__);
	else if (g_charge_detect)
		ROS_WARN("%s %d: Finish cleaning and stop in charger stub.", __FUNCTION__, __LINE__);
	else if (g_battery_low)
		ROS_WARN("%s %d: Battery too low. Finish cleaning.", __FUNCTION__, __LINE__);
	else
		if (get_clean_mode() == Clean_Mode_Spot)
			ROS_WARN("%s %d: Finish cleaning.", __FUNCTION__, __LINE__);
		else
			ROS_WARN("%s %d: Can not go to charger stub after going to all home cells. Finish cleaning.", __FUNCTION__, __LINE__);

	g_saved_work_time += get_work_time();
	ROS_WARN("%s %d: Cleaning time: %d(s)", __FUNCTION__, __LINE__, g_saved_work_time);

	if (g_battery_low)
		set_clean_mode(Clean_Mode_Sleep);
	else if (g_charge_detect)
		set_clean_mode(Clean_Mode_Charging);
	else
		set_clean_mode(Clean_Mode_Userinterface);
}

bool MotionManage::initCleaning(uint8_t cleaning_mode)
{
	switch (cleaning_mode)
	{
		case Clean_Mode_Navigation:
			return initNavigationCleaning();
		case Clean_Mode_WallFollow:
			return initWallFollowCleaning();
		case Clean_Mode_Spot:
			return initSpotCleaning();
		default:
			ROS_ERROR("This mode (%d) should not use MotionManage.", cleaning_mode);
			return false;
	}
}

bool MotionManage::initNavigationCleaning(void)
{

	reset_work_time();
	if (g_remote_home || g_go_home_by_remote)
		set_led_mode(LED_FLASH, LED_ORANGE, 1000);
	else
		set_led_mode(LED_FLASH, LED_GREEN, 1000);

	// Initialize motors and map.
	if (!robot::instance()->isManualPaused() && !robot::instance()->isLowBatPaused() && !g_resume_cleaning)
	{
		g_saved_work_time = 0;
		ROS_INFO("%s ,%d ,set g_saved_work_time to zero ", __FUNCTION__, __LINE__);
		//Initital home point
		g_home_point_old_path.clear();
		g_home_point_new_path.clear();

		// Push the start point into the home point list
		Cell_t cell{0, 0};
		g_home_point_old_path.push_front(cell);

		// Mark all the trapped reference cells as (0, 0).
		Cell_t tmp_pnt{0, 0};
		auto g_temp_trapped_cell = path_escape_get_trapped_cell();
		for (int i = 0; i < ESCAPE_TRAPPED_REF_CELL_SIZE; ++i)
		{
			g_temp_trapped_cell[i] = tmp_pnt;
		}
		path_escape_set_trapped_cell(g_temp_trapped_cell, ESCAPE_TRAPPED_REF_CELL_SIZE);

		ROS_INFO("map_init-----------------------------");
		map_init(MAP);
		map_init(WFMAP);
		path_planning_initialize(g_home_point_old_path.front());

		robot::instance()->initOdomPosition();

		// If it it the first time cleaning, initialize the g_continue_point.
		extern Cell_t g_continue_cell;
		g_continue_cell.X = g_continue_cell.Y = 0;
		extern bool g_have_seen_charge_stub, g_start_point_seen_charger;
		g_have_seen_charge_stub = false;
		g_start_point_seen_charger = false;

		extern bool g_switch_home_cell;
		g_switch_home_cell = true;
	}

	reset_touch();

	set_gyro_off();
	usleep(30000);
	set_gyro_on();

	reset_rcon_status();
	reset_touch();
	// Can't register until the status has been checked. because if register too early, the handler may affect the pause status, so it will play the wrong wav.
	if (g_resume_cleaning)
	{
		ROS_WARN("Restore from low battery pause");
		cm_register_events();
		wav_play(WAV_CLEANING_CONTINUE);
	}
	else if (robot::instance()->isManualPaused())
	{
		ROS_WARN("Restore from manual pause");
		cm_register_events();
		wav_play(WAV_CLEANING_CONTINUE);
		if (g_go_home)
		{
			wav_play(WAV_BACK_TO_CHARGER);
		}
	}
	else if(g_plan_activated == true)
	{
		cm_register_events();
		wav_play(WAV_PLAN_CLEANING_START);
		g_plan_activated = false;
	}
	else{
		cm_register_events();
		wav_play(WAV_CLEANING_START);
	}

	if (!wait_for_gyro_on())
		return false;

	if (robot::instance()->isManualPaused() || g_resume_cleaning)
	{
		robot::instance()->offsetAngle(robot::instance()->savedOffsetAngle());
		ROS_WARN("%s %d: Restore the gyro angle(%f).", __FUNCTION__, __LINE__, -robot::instance()->savedOffsetAngle());
		if (!g_go_home)
			cm_check_should_go_home();
	}

	/*Move back from charge station*/
	if (is_on_charger_stub()) {
		ROS_WARN("%s %d: calling moving back", __FUNCTION__, __LINE__);
		set_side_brush_pwm(30, 30);
		// Set i < 7 for robot to move back for approximately 500mm.
		for (int i = 0; i < 7; i++) {
			// Move back for distance of 72mm, it takes approximately 0.5s.
			quick_back(20, 72);
			if (g_fatal_quit_event || g_key_clean_pressed || is_on_charger_stub() || g_cliff_all_triggered) {
				disable_motors();
				if (g_fatal_quit_event)
				{
					robot::instance()->resetManualPause();
					g_resume_cleaning = false;
				}
				else if (g_key_clean_pressed && !g_resume_cleaning)
					// Reset the odom position so when continue cleaning, the position robot stopped at will be the home point (0, 0).
					robot::instance()->initOdomPosition();
				else if (!g_fatal_quit_event && !g_key_clean_pressed)
				{
					ROS_WARN("%s %d: Fail to leave charger stub.", __FUNCTION__, __LINE__);
					robot::instance()->resetManualPause();
					g_resume_cleaning = false;
				}
				return false;
			}
		}
		auto cell = cm_update_position();
		path_set_home(cell);
		stop_brifly();
		extern bool g_from_station;
		g_from_station = 1;
	}

	// enable titlt detct
	set_acc_init_data();//about 200ms delay
	g_tilt_enable = true;
	ROS_INFO("\033[47;35m" "%s,%d,enable tilt detect" "\033[0m",__FUNCTION__,__LINE__);

	work_motor_configure();

	ROS_INFO("%s %d: Init g_go_home(%d), lowbat(%d), manualpaused(%d), g_resume_cleaning(%d).", __FUNCTION__, __LINE__, g_go_home, robot::instance()->isLowBatPaused(), robot::instance()->isManualPaused(), g_resume_cleaning);
	return true;
}

bool MotionManage::initWallFollowCleaning(void)
{
	cm_register_events();
	set_led_mode(LED_FLASH, LED_GREEN, 1000);

	extern std::vector<Pose16_t> g_wf_cell;
	reset_work_time();
	reset_move_with_remote();
	reset_rcon_status();
	reset_stop_event_status();
	reset_touch();
	set_gyro_off();
	usleep(30000);
	set_gyro_on();
	
	wav_play(WAV_CLEANING_WALL_FOLLOW);
	if (!wait_for_gyro_on())
	{
		return false;
	}
	// enable titlt detct
	set_acc_init_data();//about 200ms delay
	g_tilt_enable = true;
	ROS_INFO("\033[47;35m" "%s,%d,enable tilt detect" "\033[0m",__FUNCTION__,__LINE__);

	g_saved_work_time = 0;
	ROS_INFO("%s ,%d ,set g_saved_work_time to zero ", __FUNCTION__, __LINE__);
	//Initital home point
	g_home_point_old_path.clear();
	g_home_point_new_path.clear();
	g_wf_cell.clear();
	// Push the start point into the home point list
	Cell_t cell{0, 0};
	g_home_point_old_path.push_front(cell);

	map_init(MAP);
	ROS_WARN("%s %d: map initialized", __FUNCTION__, __LINE__);
	map_init(WFMAP);
	ROS_WARN("%s %d: wf map initialized", __FUNCTION__, __LINE__);
	debug_map(MAP, 0, 0);
	wf_path_planning_initialize(g_home_point_old_path.front());
	ROS_WARN("%s %d: path planning initialized", __FUNCTION__, __LINE__);
	//pthread_t	escape_thread_id;
	robot::instance()->initOdomPosition();// for reset odom position to zero.

	extern bool g_have_seen_charge_stub;
	g_have_seen_charge_stub = false;

	extern bool g_switch_home_cell;
	g_switch_home_cell = true;

	work_motor_configure();

	return true;
}

bool MotionManage::initSpotCleaning(void)
{
	cm_register_events();
	set_led_mode(LED_FLASH, LED_GREEN, 1000);

	reset_work_time();
	reset_rcon_status();
	reset_move_with_remote();
	reset_stop_event_status();
	reset_touch();

	set_gyro_off();
	usleep(30000);
	set_gyro_on();
	
	wav_play(WAV_CLEANING_SPOT);
	if (!wait_for_gyro_on())
	{
		return false;
	}
	// enable titlt detct
	set_acc_init_data();//about 200ms delay
	g_tilt_enable = true;
	ROS_INFO("\033[47;35m" "%s,%d,enable tilt detect" "\033[0m",__FUNCTION__,__LINE__);

	g_saved_work_time = 0;
	ROS_INFO("%s ,%d ,set g_saved_work_time to zero ", __FUNCTION__, __LINE__);
	g_home_point_old_path.clear();
	g_home_point_new_path.clear();
	Cell_t cell{0, 0};
	g_home_point_old_path.push_front(cell);//init home point
	map_init(MAP);//init map
	path_planning_initialize(g_home_point_old_path.front());//init pathplan

	robot::instance()->initOdomPosition();// for reset odom position to zero.

	set_vac_mode(Vac_Max);
	set_vac_speed();
	set_main_brush_pwm(80);
	set_side_brush_pwm(60, 60);

	return true;
}

void MotionManage::pubCleanMapMarkers(uint8_t id, Cell_t &next, Cell_t &target, const std::list<Cell_t>& path)
{
	int16_t x, y, x_min, x_max, y_min, y_max;
	CellState cell_state;
	path_get_range(id, &x_min, &x_max, &y_min, &y_max);

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
			else
			{
				cell_state = map_get_cell(id, x, y);
				if (cell_state > UNCLEAN && cell_state < BLOCKED_BOUNDARY )
					robot::instance()->setCleanMapMarkers(x, y, cell_state);
			}
		}
	}
#if LINEAR_MOVE_WITH_PATH
	if (!path.empty())
	{
		for (list<Cell_t>::const_iterator it = path.begin(); it->X != path.back().X || it->Y != path.back().Y; it++)
			robot::instance()->setCleanMapMarkers(it->X, it->Y, TARGET);

		robot::instance()->setCleanMapMarkers(path.back().X, path.back().Y, TARGET_CLEAN);
	}
#endif

	robot::instance()->pubCleanMapMarkers();
}
