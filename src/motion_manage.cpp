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

Segment_set segmentss;

extern std::list <Point32_t> g_home_point_old_path;
extern std::list <Point32_t> g_home_point_new_path;

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

		if (g_fatal_quit_event || g_key_clean_pressed || get_cliff_trig() == Status_Cliff_All)
		{
			ROS_WARN("%s %d: Launch obstacle detector interrupted.", __FUNCTION__, __LINE__);
			return false;
		}
	}

	if (line_align_ != start)
	{
		ROS_WARN("%s %d: Obstacle detector launch timeout, do not align.", __FUNCTION__, __LINE__);
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

		if (g_fatal_quit_event || g_key_clean_pressed || get_cliff_trig()== Status_Cliff_All)
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
	g_motion_init_succeeded = false;
	event_manager_reset_status();
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
	if (s_laser->isReady() == -1)
	{
		ROS_ERROR("%s %d: Laser opening failed.", __FUNCTION__, __LINE__);
		set_error_code(Error_Code_Laser);
		wav_play(WAV_TEST_LIDAR);
		initSucceeded(false);
		return;
	}
	else if (s_laser->isReady() == 0)
	{
		initSucceeded(false);
		return;
	}

	if (robot::instance()->isLowBatPaused())
	{
		robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle);
		s_laser->startShield();
		return;
	}
	if (robot::instance()->isManualPaused() && s_slam != nullptr)
	{
		robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle);
		robot::instance()->resetManualPause();
		s_laser->startShield();
		return;
	}

	//3 calculate offsetAngle
	nh_.param<bool>("is_active_align", is_align_active_, false);
	if (get_clean_mode() == Clean_Mode_Navigation && is_align_active_)
	{
		ObstacleDetector od;
		float align_angle=0;
		if(!get_align_angle(align_angle))
		{
			initSucceeded(false);
			return;
		}
		robot::instance()->offsetAngle(align_angle);
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

		if (g_fatal_quit_event || g_key_clean_pressed || get_cliff_trig() == Status_Cliff_All)
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
	s_laser->startShield();
}

MotionManage::~MotionManage()
{

	reset_stop_event_status();
	disable_motors();

	if (get_cliff_trig() == Status_Cliff_All)
		wav_play(WAV_ERROR_LIFT_UP);

	if (s_laser != nullptr)
	{
		delete s_laser; // It takes about 1s.
		s_laser = nullptr;
	}

	robot::instance()->setBaselinkFrameType(Odom_Position_Odom_Angle);
	// Wait for 0.15s to make sure last tf waiting for map frame has finished and Odom_Position_Odom_Angle has been activated.
	usleep(150000);

	cm_unregister_events();

	if (SpotMovement::instance()->getSpotType() != NO_SPOT)
	//if (get_clean_mode() == Clean_Mode_Spot)
	{
		SpotMovement::instance()->spotDeinit();// clear the variables.
		sleep(1);
	}

	if (robot::instance()->isManualPaused())
	{
		set_clean_mode(Clean_Mode_Userinterface);
		wav_play(WAV_PAUSE_CLEANING);
		robot::instance()->savedOffsetAngle(robot::instance()->getAngle());
		ROS_INFO("%s %d: Save the gyro angle(%f) before pause.", __FUNCTION__, __LINE__, robot::instance()->getAngle());
		extern bool g_go_home;
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
		return;
	}
	if (robot::instance()->isLowBatPaused())
	{
		set_clean_mode(Clean_Mode_Charging);
		wav_play(WAV_PAUSE_CLEANING);
		robot::instance()->savedOffsetAngle(robot::instance()->getAngle());
		ROS_WARN("%s %d: Save the gyro angle(%f) before pause.", __FUNCTION__, __LINE__, robot::instance()->getAngle());
		ROS_WARN("%s %d: Pause cleaning for low battery, will continue cleaning when charge finished.", __FUNCTION__, __LINE__);
		g_saved_work_time += get_work_time();
		ROS_WARN("%s %d: Cleaning time: %d(s)", __FUNCTION__, __LINE__, g_saved_work_time);
		return;
	}

	if (s_slam != nullptr)
	{
		delete s_slam;
		s_slam = nullptr;
	}

	robot::instance()->savedOffsetAngle(0);

	wav_play(WAV_CLEANING_FINISHED);

	g_home_point_old_path.clear();
	g_home_point_new_path.clear();

	if (g_fatal_quit_event)
		if (get_cliff_trig() == Status_Cliff_All)
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
			ROS_WARN("%s %d: Can not go to charger stub after going to all home points. Finish cleaning.", __FUNCTION__, __LINE__);

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
	// Wait for 20ms to make sure the event manager has start working.
	usleep(20000);

	reset_work_time();
	set_led(100, 0);

	// Initialize motors and map.
	if (robot::instance()->isLowBatPaused())
	{
		if (get_rcon_status())
		{
			Point32_t new_home_point;
			// Save the current coordinate as a new home point.
			new_home_point.X = map_get_x_count();
			new_home_point.Y = map_get_y_count();

			// Push the start point into the home point list.
			g_home_point_old_path.push_front(new_home_point);
		}

		reset_rcon_status();
	}
	else if (!robot::instance()->isManualPaused())
	{
		g_saved_work_time = 0;
		ROS_INFO("%s ,%d ,set g_saved_work_time to zero ", __FUNCTION__, __LINE__);
		//Initital home point
		g_home_point_old_path.clear();
		g_home_point_new_path.clear();

		// Push the start point into the home point list
		Point32_t new_home_point;
		new_home_point.X = new_home_point.Y = 0;
		g_home_point_old_path.push_front(new_home_point);

		// Mark all the trapped reference points as (0, 0).
		Cell_t tmp_pnt;
		tmp_pnt.X = 0;
		tmp_pnt.Y = 0;
		extern Cell_t g_pnt16_ar_tmp[3];
		for (int i = 0; i < ESCAPE_TRAPPED_REF_CELL_SIZE; ++i)
		{
			g_pnt16_ar_tmp[i] = tmp_pnt;
		}
		path_escape_set_trapped_cell(g_pnt16_ar_tmp, ESCAPE_TRAPPED_REF_CELL_SIZE);

		ROS_INFO("map_init-----------------------------");
		map_init();
		path_planning_initialize(&g_home_point_old_path.front().X, &g_home_point_old_path.front().Y);

		robot::instance()->initOdomPosition();

		// If it it the first time cleaning, initialize the g_continue_point.
		extern Point32_t g_continue_point;
		g_continue_point.X = g_continue_point.Y = 0;
	}

	reset_touch();

	// Restart the gyro.
	set_gyro_off();
	// Wait for 30ms to make sure the off command has been effectived.
	usleep(30000);
	// Set gyro on before wav_play can save the time for opening the gyro.
	set_gyro_on();

	if (robot::instance()->isLowBatPaused())
		wav_play(WAV_CLEANING_CONTINUE);
	else if (robot::instance()->isManualPaused())
	{
		ROS_WARN("Restore from manual pause");
		wav_play(WAV_CLEANING_CONTINUE);
	}
	else{
		wav_play(WAV_CLEANING_START);
	}

	reset_rcon_status();
	reset_touch();
	// Can't register until now because if register too early, the handler may affect the pause status, so it will play the wrong wav.
	cm_register_events();

	if (!wait_for_gyro_on())
		return false;

	if (robot::instance()->isManualPaused() || robot::instance()->isLowBatPaused())
	{
		robot::instance()->offsetAngle(robot::instance()->savedOffsetAngle());
		ROS_WARN("%s %d: Restore the gyro angle(%f).", __FUNCTION__, __LINE__, -robot::instance()->savedOffsetAngle());
	}

	/*Move back from charge station*/
	if (is_on_charger_stub()) {
		ROS_WARN("%s %d: calling moving back", __FUNCTION__, __LINE__);
		set_side_brush_pwm(30, 30);
		// Set i < 7 for robot to move back for approximately 500mm.
		for (int i = 0; i < 7; i++) {
			// Move back for distance of 72mm, it takes approximately 0.5s.
			quick_back(20, 72);
			if (g_fatal_quit_event || g_key_clean_pressed || is_on_charger_stub() || get_cliff_trig() == Status_Cliff_All) {
				disable_motors();
				if (g_fatal_quit_event)
				{
					robot::instance()->resetManualPause();
					robot::instance()->resetLowBatPause();
				}
				else if (g_key_clean_pressed && !robot::instance()->isLowBatPaused())
					// Reset the odom position so when continue cleaning, the position robot stopped at will be the home point (0, 0).
					robot::instance()->initOdomPosition();
				else if (!g_fatal_quit_event && !g_key_clean_pressed)
				{
					ROS_WARN("%s %d: Fail to leave charger stub.", __FUNCTION__, __LINE__);
					robot::instance()->resetManualPause();
					robot::instance()->resetLowBatPause();
				}
				return false;
			}
		}
		auto cell = cm_update_position();
		cm_set_home(map_get_x_count(), map_get_y_count());
		stop_brifly();
		extern bool g_from_station;
		g_from_station = 1;
	}

	work_motor_configure();

	extern bool g_go_home;
	ROS_INFO("init g_go_home(%d), lowbat(%d), manualpaused(%d)", g_go_home, robot::instance()->isLowBatPaused(), robot::instance()->isManualPaused());
	return true;
}

bool MotionManage::initWallFollowCleaning(void)
{
	cm_register_events();

	extern std::vector<Pose16_t> g_wf_cell;
	reset_work_time();
	reset_move_with_remote();
	reset_rcon_status();
	reset_stop_event_status();
	reset_touch();
	// Restart the gyro.
	set_gyro_off();
	// Wait for 30ms to make sure the off command has been effectived.
	usleep(30000);
	// Set gyro on before wav_play can save the time for opening the gyro.
	set_gyro_on();
	set_led(100, 0);
	//wav_play(WAV_SYSTEM_INITIALIZING);
	wav_play(WAV_CLEANING_WALL_FOLLOW);
	if (!wait_for_gyro_on())
	{
		return false;
	}

	g_saved_work_time = 0;
	ROS_INFO("%s ,%d ,set g_saved_work_time to zero ", __FUNCTION__, __LINE__);
	//Initital home point
	g_home_point_old_path.clear();
	g_home_point_new_path.clear();
	g_wf_cell.clear();
	Point32_t new_home_point;
	new_home_point.X = new_home_point.Y = 0;
	// Push the start point into the home point list
	g_home_point_old_path.push_front(new_home_point);

	map_init();
	ROS_WARN("%s %d: grid map initialized", __FUNCTION__, __LINE__);
	debug_map(MAP, 0, 0);
	wf_path_planning_initialize(&g_home_point_old_path.front().X, &g_home_point_old_path.front().Y);
	ROS_WARN("%s %d: path planning initialized", __FUNCTION__, __LINE__);
	//pthread_t	escape_thread_id;
	robot::instance()->initOdomPosition();// for reset odom position to zero.

	work_motor_configure();

	return true;
}

bool MotionManage::initSpotCleaning(void)
{
	cm_register_events();

	reset_work_time();
	reset_rcon_status();
	reset_move_with_remote();
	reset_stop_event_status();
	reset_touch();

	// Restart the gyro.
	set_gyro_off();
	// Wait for 30ms to make sure the off command has been effectived.
	usleep(30000);
	// Set gyro on before wav_play can save the time for opening the gyro.
	set_gyro_on();
	set_led(100, 0);
	//wav_play(WAV_SYSTEM_INITIALIZING);
	wav_play(WAV_CLEANING_SPOT);
	if (!wait_for_gyro_on())
	{
		return false;
	}

	g_saved_work_time = 0;
	ROS_INFO("%s ,%d ,set g_saved_work_time to zero ", __FUNCTION__, __LINE__);
	Point32_t t_point;
	t_point.X = 0;
	t_point.Y = 0;
	g_home_point_old_path.clear();
	g_home_point_new_path.clear();
	g_home_point_old_path.push_front(t_point);//init home point
	map_init();//init map
	path_planning_initialize(&g_home_point_old_path.front().X, &g_home_point_old_path.front().Y);//init pathplan

	robot::instance()->initOdomPosition();// for reset odom position to zero.

	set_vac_mode(Vac_Max);
	set_vac_speed();
	set_main_brush_pwm(80);
	set_side_brush_pwm(60, 60);

	return true;
}

void MotionManage::pubCleanMapMarkers(uint8_t id, Point32_t next_point, Point32_t target_point)
{
	int16_t i, j, x_min, x_max, y_min, y_max, next_point_x, next_point_y, target_point_x, target_point_y;
	CellState cell_state;
	path_get_range(&x_min, &x_max, &y_min, &y_max);

	next_point_x = count_to_cell(next_point.X);
	if (next_point_x == SHRT_MIN )
		next_point_x = x_min;
	else if (next_point_x == SHRT_MAX)
		next_point_x = x_max;

	next_point_y = count_to_cell(next_point.Y);
	target_point_x = count_to_cell(target_point.X);
	target_point_y = count_to_cell(target_point.Y);

	for (i = x_min; i <= x_max; i++)
	{
		for (j = y_min; j <= y_max; j++)
		{
			if (i == target_point_x && j == target_point_y)
				robot::instance()->setCleanMapMarkers(i, j, TARGET_CLEAN);
			else if (i == next_point_x && j == next_point_y)
				robot::instance()->setCleanMapMarkers(i, j, TARGET);
			else
			{
				cell_state = map_get_cell(id, i, j);
				if (cell_state == CLEANED || cell_state == BLOCKED_OBS || cell_state == BLOCKED_BUMPER)
					robot::instance()->setCleanMapMarkers(i, j, cell_state);
			}
		}
	}
	robot::instance()->pubCleanMapMarkers();
}
