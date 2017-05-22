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
#include <path_planning.h>
Segment_set segmentss;

int8_t g_enable_slam_offset = 0;

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
	if (!s_laser->is_ready())
		return;

//	ROS_WARN("2robot_obstacles_cb");
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
	robot::instance()->setOdomReady(false);
	segmentss.clear();
//	ROS_INFO("Start subscribe to /obstacles");
	auto obstacles_sub = nh_.subscribe("/obstacles", 1, &MotionManage::robot_obstacles_cb, this);

	//wait for start obstacle_detector
	auto count_n_10ms = 1000;
	while (line_align_ != start && --count_n_10ms > 0 && !Stop_Event()){
		if (count_n_10ms % 100 == 0)
			ROS_WARN(" start obstacle_detector remain %d s\n", count_n_10ms / 100);
		usleep(10000);
	}
	if(Stop_Event() || count_n_10ms == 0)
		return false;

	count_n_10ms = 200;
	ROS_INFO("Obstacle detector launch finishd.");

	//wait for detecting line
	while (--count_n_10ms > 0 && !Stop_Event())
	{
		if (count_n_10ms % 100 == 0)
			ROS_INFO("detecting line time remain %d s", count_n_10ms / 100);
		usleep(10000);
	}
//	obstacles_sub.shutdown();
	if(Stop_Event())
		return false;

	ROS_INFO("Get the line");
//	auto line_angle = static_cast<int16_t>(segmentss.min_distant_segment_angle() *10);
	line_angle = segmentss.min_distant_segment_angle();

	//todo testing to turn 180 degrees.
	if (line_angle > 0)
	{
		line_angle -= 180;
	} else if (line_angle <= 0)
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
	initSucceeded(true);

	//1 Initialize for different mode.
	if (!initCleaning(Get_Clean_Mode()))
	{
		initSucceeded(false);
		return;
	}

	//2 start motor
	Work_Motor_Configure();

	//3 start laser
	s_laser = new Laser();
	if (!s_laser->is_ready())
	{
		if (!Stop_Event())
		{
			ROS_ERROR("%s %d: Laser opening failed.", __FUNCTION__, __LINE__);
			Set_Error_Code(Error_Code_Slam);
			wav_play(WAV_TEST_LIDAR);
		}
		initSucceeded(false);
		return;
	}

#if CONTINUE_CLEANING_AFTER_CHARGE
	if (robot::instance()->isCleaningLowBatPaused())
		return;
#endif
#if MANUAL_PAUSE_CLEANING
	if (robot::instance()->isCleaningManualPaused())
	{
		robot::instance()->resetCleaningManualPause();
		return;
	}
#endif

	//4 calculate offsetAngle
	nh_.param<bool>("is_active_align", is_align_active_, false);
	if (Get_Clean_Mode() == Clean_Mode_Navigation && is_align_active_)
	{
		ObstacleDetector od;
		float align_angle=0;
		if(! get_align_angle(align_angle) )
			return ;

		robot::instance()->offsetAngle(align_angle);
	}


	ROS_INFO("waiting 1s for translation odom_to_robotbase work");
	sleep(1); //wait for odom_pub send translation(odom->robotbase) to slam_karto,
	//5 call start slam
	s_slam = new Slam();

	g_enable_slam_offset = 1;
	s_slam->enable_map_update();
	auto count_n_10ms = 1000;
	while (!s_slam->is_map_ready() && --count_n_10ms != 0)
	{
//		ROS_WARN("%s %d: Map is still not ready after 10s, timeout and return.", __FUNCTION__, __LINE__);
		usleep(10000);
	}
	if (count_n_10ms == 0)
	{
		ROS_ERROR("%s %d: Map is still not ready after 10s, timeout and return.", __FUNCTION__, __LINE__);
		Set_Error_Code(Error_Code_Slam);
		wav_play(WAV_TEST_LIDAR);
		initSucceeded(false);
		return;
	}
}

MotionManage::~MotionManage()
{
	Disable_Motors();

	g_enable_slam_offset = 0;

	if (s_laser != nullptr)
	{
		delete s_laser;
		s_laser = nullptr;
	}

#if CONTINUE_CLEANING_AFTER_CHARGE
	if (robot::instance()->isCleaningLowBatPaused())
		return;
#endif
#if MANUAL_PAUSE_CLEANING
	if (robot::instance()->isCleaningManualPaused())
	{
		wav_play(WAV_PAUSE_CLEANING);
		return;
	}
#endif

	if (s_slam != nullptr)
	{
		delete s_slam;
		s_slam = nullptr;
	}

	robot::instance()->offsetAngle(0);

	if (Get_Cliff_Trig())
		wav_play(WAV_ERROR_LIFT_UP);

	wav_play(WAV_CLEANING_FINISHED);

	nh_.shutdown();
}

bool MotionManage::initCleaning(uint8_t cleaning_mode)
{
	switch (cleaning_mode)
	{
		case Clean_Mode_Navigation:
			return initNavigationCleaning();
		case Clean_Mode_WallFollow:
			// TODO: return initWallFollowCleaning();
			return true;
		case Clean_Mode_Spot:
			// TODO: return initSpotCleaning();
			return true;
		default:
			ROS_ERROR("This mode (%d) should not use MotionManage.", cleaning_mode);
			return false;
	}
}

bool MotionManage::initNavigationCleaning(void)
{
	extern void CM_reset_cleaning_low_bat_pause();
	extern uint32_t g_cur_wtime;
	extern std::list <Point32_t> g_home_point;

	Set_LED(100,0);
	Reset_Rcon_Status();
	Reset_MoveWithRemote();
	Reset_Stop_Event_Status();

	// Opening the gyro.
#if CONTINUE_CLEANING_AFTER_CHARGE
	if (robot::instance()->isCleaningLowBatPaused())
	{
		wav_play(WAV_CLEANING_CONTINUE);
	}
	else
#endif
	{
#if MANUAL_PAUSE_CLEANING
		if (robot::instance()->isCleaningManualPaused())
		{
			ROS_WARN("Restore from manual pause");
			wav_play(WAV_CLEANING_CONTINUE);
		}
		else
#endif
		{
			// Restart the gyro.
			Set_Gyro_Off();
			// Wait for 30ms to make sure the off command has been effectived.
			usleep(30000);
			// Set gyro on before wav_play can save the time for opening the gyro.
			Set_Gyro_On();
			wav_play(WAV_CLEANING_START);

			if (!Wait_For_Gyro_On())
			{
				Set_Clean_Mode(Clean_Mode_Userinterface);
#if CONTINUE_CLEANING_AFTER_CHARGE
				if (robot::instance()->isCleaningLowBatPaused())
				{
					ROS_WARN("%s %d: fail to leave charger stub when continue to clean.", __FUNCTION__, __LINE__);
					// Quit continue cleaning.
					CM_reset_cleaning_low_bat_pause();
				}
#endif
#if MANUAL_PAUSE_CLEANING
				if (robot::instance()->isCleaningManualPaused())
				{
					robot::instance()->resetCleaningManualPause();
				}
#endif
				return false;
			}
		}
	}

	/*Move back from charge station*/
	if (Is_AtHomeBase()) {
		// Key release detection, if user has not release the key, don't do anything.
		while (Get_Key_Press() & KEY_CLEAN)
		{
			ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
			usleep(20000);
		}
		// Key relaesed, then the touch status and stop event status should be cleared.
		Reset_Stop_Event_Status();
		ROS_WARN("%s %d: calling moving back", __FUNCTION__, __LINE__);
		Set_SideBrush_PWM(30, 30);
		// Reset the robot to non charge mode.
		set_stop_charge();
		while (Is_ChargerOn())
		{
			ROS_INFO("Robot Still charging.");
			usleep(20000);
		}
		// Sleep for 30ms to make sure it has sent at least one control message to stop charging.
		usleep(30000);
		if (Is_ChargerOn()){
			ROS_WARN("[core_move.cpp] Still charging.");
		}
		// Set i < 7 for robot to move back for approximately 500mm.
		for (int i = 0; i < 7; i++) {
			// Move back for distance of 72mm, it takes approximately 0.5s.
			Quick_Back(20, 72);
			if (Stop_Event() || Is_AtHomeBase()) {
				Set_Clean_Mode(Clean_Mode_Userinterface);
				Disable_Motors();
				if (Is_AtHomeBase())
				{
					ROS_WARN("%s %d: move back 100mm and still detect charger! Or touch event. return 0", __FUNCTION__, __LINE__);
				}
				if (Get_Key_Press() & KEY_CLEAN)
				{
					ROS_WARN("%s %d: touch event! return 0", __FUNCTION__, __LINE__);
					Stop_Brifly();
					// Key release detection, if user has not release the key, don't do anything.
					while (Get_Key_Press() & KEY_CLEAN)
					{
						ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
						usleep(20000);
					}
					Reset_Stop_Event_Status();
				}
#if CONTINUE_CLEANING_AFTER_CHARGE
				if (robot::instance()->isCleaningLowBatPaused())
				{
					ROS_WARN("%s %d: fail to leave charger stub when continue to clean.", __FUNCTION__, __LINE__);
					// Quit continue cleaning.
					CM_reset_cleaning_low_bat_pause();
					g_cur_wtime = 0;
					ROS_INFO("%s ,%d ,set g_cur_wtime to zero",__FUNCTION__,__LINE__);
				}
#endif
#if MANUAL_PAUSE_CLEANING
				if (robot::instance()->isCleaningManualPaused())
				{
					robot::instance()->resetCleaningManualPause();
				}
#endif
				return false;
			}
		}
		Deceleration();
		Stop_Brifly();
		extern uint8_t g_from_station;
		g_from_station = 1;
	}
	else
	{
		// Key release detection, if user has not release the key, don't do anything.
		while (Get_Key_Press() & KEY_CLEAN)
		{
			ROS_INFO("%s %d: User hasn't release key or still cliff detected.", __FUNCTION__, __LINE__);
			usleep(20000);
		}
		// Key relaesed, then the touch status and stop event status should be cleared.
		Reset_Stop_Event_Status();
	}

	// Initialize motors and map.
#if CONTINUE_CLEANING_AFTER_CHARGE
	if (robot::instance()->isCleaningLowBatPaused())
	{
		if (Get_Rcon_Status())
		{
			Point32_t new_g_home_point;
			// Save the current coordinate as a new home point.
			new_g_home_point.X = Map_GetXCount();
			new_g_home_point.Y = Map_GetYCount();

			// Push the start point into the home point list.
			g_home_point.push_front(new_g_home_point);
		}

		Reset_Rcon_Status();
	}
	else
#endif
	{
#if MANUAL_PAUSE_CLEANING
		if (robot::instance()->isCleaningManualPaused())
		{
			Reset_Work_Time();
		}
		else
#endif
		{
			// Set the Work_Timer_Start as current time
			Reset_Work_Time();
			g_cur_wtime = 0;
			ROS_INFO("%s ,%d ,set g_cur_wtime to zero ",__FUNCTION__,__LINE__);
			//Initital home point
			g_home_point.clear();

			// Push the start point into the home point list
			Point32_t new_g_home_point;
			new_g_home_point.X = new_g_home_point.Y = 0;
			g_home_point.push_front(new_g_home_point);

			// Mark all the trapped reference points as (0, 0).
			Point16_t tmp_pnt;
			tmp_pnt.X = 0;
			tmp_pnt.Y = 0;
			extern Point16_t g_pnt16_ar_tmp[3];
			for ( int i = 0; i < ESCAPE_TRAPPED_REF_CELL_SIZE; ++i ) {
				g_pnt16_ar_tmp[i] = tmp_pnt;
			}
			path_escape_set_trapped_cell(g_pnt16_ar_tmp, ESCAPE_TRAPPED_REF_CELL_SIZE);

			ROS_INFO("Map_Initialize-----------------------------");
			Map_Initialize();
			PathPlanning_Initialize(&g_home_point.front().X, &g_home_point.front().Y);

			robot::instance()->initOdomPosition();

#if CONTINUE_CLEANING_AFTER_CHARGE
			// If it it the first time cleaning, initialize the g_continue_point.
			extern Point32_t g_continue_point;
			g_continue_point.X = g_continue_point.Y = 0;
#endif
		}
	}

	return true;
}
