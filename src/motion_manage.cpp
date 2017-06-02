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
#include <core_move.h>

Segment_set segmentss;

/* Events variables */
/* The fatal quit event includes any of the following case:
 *  g_bumper_jam
 * 	g_cliff_all_triggered
 * 	g_oc_brush_main
 * 	g_oc_wheel_left
 * 	g_oc_wheel_right
 * 	g_oc_suction
 * 	g_battery_low
 */
extern bool g_fatal_quit_event;
extern bool g_bumper_jam;
extern bool g_bumper_hitted;
extern bool g_obs_triggered;
extern bool g_cliff_all_triggered;
extern bool g_cliff_jam;
extern bool g_cliff_triggered;
extern bool g_rcon_triggered;
extern bool g_oc_brush_main;
extern bool g_oc_wheel_left;
extern bool g_oc_wheel_right;
extern bool g_oc_suction;
extern bool g_key_clean_pressed;
extern bool g_remote_home;
extern bool g_battery_home;
extern bool g_battery_low;
extern bool g_from_station;
extern bool g_go_home;
extern uint8_t g_oc_brush_left_cnt;
extern uint8_t g_oc_brush_main_cnt;
extern uint8_t g_oc_brush_right_cnt;
extern uint8_t g_oc_wheel_left_cnt;
extern uint8_t g_oc_wheel_right_cnt;
extern uint8_t g_oc_suction_cnt;
extern uint8_t g_cliff_cnt;
extern uint8_t	g_remote_go_home;
extern uint16_t g_press_time;
extern int g_bumper_cnt;

extern std::list <Point32_t> g_home_point;

extern uint32_t g_cur_wtime;//temporary work time

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
	if (!s_laser->isReady())
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

	//2 start laser
	s_laser = new Laser();
	if (!s_laser->isReady())
	{
		if (!Stop_Event())
		{
			ROS_ERROR("%s %d: Laser opening failed.", __FUNCTION__, __LINE__);
			Set_Error_Code(Error_Code_Laser);
			wav_play(WAV_TEST_LIDAR);
		}
		initSucceeded(false);
		return;
	}

	if (robot::instance()->isLowBatPaused())
		return;
	if (robot::instance()->isManualPaused())
	{
		robot::instance()->resetManualPause();
		return;
	}

	//3 calculate offsetAngle
	nh_.param<bool>("is_active_align", is_align_active_, false);
	if (Get_Clean_Mode() == Clean_Mode_Navigation && is_align_active_)
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
	s_slam = new Slam();

	if (Get_Clean_Mode() == Clean_Mode_Navigation || Get_Clean_Mode() == Clean_Mode_Spot)
		robot::instance()->setBaselinkFrameType(Map_Position_Map_Angle);
	else if (Get_Clean_Mode() == Clean_Mode_WallFollow)
		robot::instance()->setBaselinkFrameType(Map_Position_Odom_Angle);
	s_slam->enableMapUpdate();
	auto count_n_10ms = 1000;
	robot::instance()->setTfReady(false);
	while (!(s_slam->isMapReady() && robot::instance()->isTfReady()) && --count_n_10ms != 0)
	{
		usleep(20000);
	}
	if (count_n_10ms == 0)
	{
		ROS_ERROR("%s %d: Map or tf framework is still not ready after 10s, timeout and return.", __FUNCTION__, __LINE__);
		Set_Error_Code(Error_Code_Slam);
		wav_play(WAV_TEST_LIDAR);
		initSucceeded(false);
		return;
	}
}

MotionManage::~MotionManage()
{

	Reset_Stop_Event_Status();

	Disable_Motors();

	if (s_laser != nullptr)
	{
		delete s_laser;
		s_laser = nullptr;
	}

	if (robot::instance()->isManualPaused()){
		Set_Clean_Mode(Clean_Mode_Userinterface);
		wav_play(WAV_PAUSE_CLEANING);
		robot::instance()->savedOffsetAngle((float)Gyro_GetAngle() / 10);
		ROS_INFO("%s %d: Save the gyro angle(%f) before pause.", __FUNCTION__, __LINE__, (float)Gyro_GetAngle() / 10);
		return;
	}

	if (robot::instance()->isLowBatPaused())
	{
		robot::instance()->savedOffsetAngle((float)Gyro_GetAngle() / 10);
		ROS_INFO("%s %d: Save the gyro angle(%f) before pause.", __FUNCTION__, __LINE__, (float)Gyro_GetAngle() / 10);
		return;
	}

	if (s_slam != nullptr)
	{
		delete s_slam;
		s_slam = nullptr;
	}

	robot::instance()->savedOffsetAngle(0);

	if(g_bumper_cnt >=3 && g_bumper_hitted)
		wav_play(WAV_ERROR_BUMPER);
	if (Get_Cliff_Trig())
		wav_play(WAV_ERROR_LIFT_UP);

	wav_play(WAV_CLEANING_FINISHED);

	g_go_home =0;
	g_home_point.clear();
	g_cur_wtime = 0;

//	if (g_key_clean_pressed == true)
	Set_Clean_Mode(Clean_Mode_Userinterface);


	if(g_battery_low)
		Set_Clean_Mode(Clean_Mode_Sleep);

	if(g_fatal_quit_event)
		Set_Clean_Mode(Clean_Mode_Sleep);

	if(g_from_station)
		Set_Clean_Mode(Clean_Mode_GoHome);

	if (g_battery_low == true) {
		ROS_WARN("%s %d: Battery too low, cleaning time: %d(s)", __FUNCTION__, __LINE__, Get_Work_Time());
	} else if (g_cliff_all_triggered == true) {
		ROS_WARN("%s %d: All Cliff are triggered, cleaning time: %d(s)", __FUNCTION__, __LINE__, Get_Work_Time());
	}
	ROS_INFO("%s %d: Finish cleanning, cleaning time: %d(s)", __FUNCTION__, __LINE__, Get_Work_Time());

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
	extern uint32_t g_cur_wtime;

	Set_LED(100,0);
	Reset_Rcon_Status();
	Reset_MoveWithRemote();
	Reset_Stop_Event_Status();

	// Restart the gyro.
	Set_Gyro_Off();
	// Wait for 30ms to make sure the off command has been effectived.
	usleep(30000);
	// Set gyro on before wav_play can save the time for opening the gyro.
	Set_Gyro_On();

	if (robot::instance()->isLowBatPaused())
	{
		wav_play(WAV_CLEANING_CONTINUE);
	}
	else if (robot::instance()->isManualPaused())
	{
		ROS_WARN("Restore from manual pause");
		wav_play(WAV_CLEANING_CONTINUE);
	}
	else
		wav_play(WAV_CLEANING_START);

	if (!Wait_For_Gyro_On())
	{
		return false;
	}

	if (robot::instance()->isManualPaused() || robot::instance()->isLowBatPaused())
	{
		robot::instance()->offsetAngle(robot::instance()->savedOffsetAngle());
		ROS_INFO("%s %d: Restore the gyro angle(%f).", __FUNCTION__, __LINE__, -robot::instance()->savedOffsetAngle());
	}

	/*Move back from charge station*/
	if (Is_AtHomeBase()) {
		ROS_WARN("%s %d: calling moving back", __FUNCTION__, __LINE__);
		Set_SideBrush_PWM(30, 30);
		// Reset the robot to non charge mode.
		set_stop_charge();
		// Sleep for 30ms to make sure it has sent at least one control message to stop charging.
		usleep(30000);
		while (Is_ChargerOn())
		{
			ROS_INFO("Robot Still charging.");
			usleep(20000);
		}
		if (Is_ChargerOn()){
			ROS_WARN("[core_move.cpp] Still charging.");
		}
		// Set i < 7 for robot to move back for approximately 500mm.
		for (int i = 0; i < 7; i++) {
			// Move back for distance of 72mm, it takes approximately 0.5s.
			Quick_Back(20, 72);
			if (Stop_Event() || Is_AtHomeBase()) {
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
				if (robot::instance()->isLowBatPaused())
				{
					ROS_WARN("%s %d: fail to leave charger stub when continue to clean.", __FUNCTION__, __LINE__);
					// Quit continue cleaning.
					robot::instance()->resetLowBatPause();
					g_cur_wtime = 0;
					ROS_INFO("%s ,%d ,set g_cur_wtime to zero",__FUNCTION__,__LINE__);
				}
				if (robot::instance()->isManualPaused())
				{
					robot::instance()->resetManualPause();
				}
				return false;
			}
		}
		Deceleration();
		Stop_Brifly();
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
	if (robot::instance()->isLowBatPaused())
	{
		if (Get_Rcon_Status())
		{
			Point32_t new_home_point;
			// Save the current coordinate as a new home point.
			new_home_point.X = Map_GetXCount();
			new_home_point.Y = Map_GetYCount();

			// Push the start point into the home point list.
			g_home_point.push_front(new_home_point);
		}

		Reset_Rcon_Status();
	} else
	if (robot::instance()->isManualPaused())
		Reset_Work_Time();
	else
	{
		// Set the Work_Timer_Start as current time
		Reset_Work_Time();
		g_cur_wtime = 0;
		ROS_INFO("%s ,%d ,set g_cur_wtime to zero ", __FUNCTION__, __LINE__);
		//Initital home point
		g_home_point.clear();

		// Push the start point into the home point list
		Point32_t new_home_point;
		new_home_point.X = new_home_point.Y = 0;
		g_home_point.push_front(new_home_point);

		// Mark all the trapped reference points as (0, 0).
		Point16_t tmp_pnt;
		tmp_pnt.X = 0;
		tmp_pnt.Y = 0;
		extern Point16_t g_pnt16_ar_tmp[3];
		for (int i = 0; i < ESCAPE_TRAPPED_REF_CELL_SIZE; ++i)
		{
			g_pnt16_ar_tmp[i] = tmp_pnt;
		}
		path_escape_set_trapped_cell(g_pnt16_ar_tmp, ESCAPE_TRAPPED_REF_CELL_SIZE);

		ROS_INFO("Map_Initialize-----------------------------");
		Map_Initialize();
		PathPlanning_Initialize(&g_home_point.front().X, &g_home_point.front().Y);

		robot::instance()->initOdomPosition();

		// If it it the first time cleaning, initialize the g_continue_point.
		extern Point32_t g_continue_point;
		g_continue_point.X = g_continue_point.Y = 0;
	}

	Work_Motor_Configure();

	ROS_INFO("init g_go_home(%d), lowbat(%d), manualpaused(%d)", g_go_home, robot::instance()->isLowBatPaused(), robot::instance()->isManualPaused());
	return true;
}

bool MotionManage::initWallFollowCleaning(void)
{
	extern std::vector<Pose32_t> WF_Point;

	Reset_MoveWithRemote();
	// Restart the gyro.
	Set_Gyro_Off();
	// Wait for 30ms to make sure the off command has been effectived.
	usleep(30000);
	// Set gyro on before wav_play can save the time for opening the gyro.
	Set_Gyro_On();
	Set_LED(100, 0);
	//wav_play(WAV_SYSTEM_INITIALIZING);
	wav_play(WAV_CLEANING_WALL_FOLLOW);
	if (!Wait_For_Gyro_On())
	{
		return false;
	}

	//Initital home point
	g_home_point.clear();
	WF_Point.clear();
	Point32_t new_home_point;
	new_home_point.X = new_home_point.Y = 0;
	// Push the start point into the home point list
	g_home_point.push_front(new_home_point);

	Map_Initialize();
	ROS_WARN("%s %d: grid map initialized", __FUNCTION__, __LINE__);
	debug_map(MAP, 0, 0);
	WF_PathPlanning_Initialize(&g_home_point.front().X, &g_home_point.front().Y);
	ROS_WARN("%s %d: path planning initialized", __FUNCTION__, __LINE__);
	//pthread_t	escape_thread_id;
	robot::instance()->initOdomPosition();// for reset odom position to zero.

	Work_Motor_Configure();

	return true;
}

bool MotionManage::initSpotCleaning(void)
{
	// Restart the gyro.
	Set_Gyro_Off();
	// Wait for 30ms to make sure the off command has been effectived.
	usleep(30000);
	// Set gyro on before wav_play can save the time for opening the gyro.
	Set_Gyro_On();
	Set_LED(100, 0);
	//wav_play(WAV_SYSTEM_INITIALIZING);
	wav_play(WAV_CLEANING_SPOT);
	if (!Wait_For_Gyro_On())
	{
		return false;
	}

	std::list<Point32_t> homepoint;
	Point32_t t_point;
	t_point.X = 0;
	t_point.Y = 0;
	homepoint.clear();
	homepoint.push_front(t_point);
	Map_Initialize();//init map
	PathPlanning_Initialize(&homepoint.front().X,&homepoint.front().Y);//init pathplan

	robot::instance()->initOdomPosition();// for reset odom position to zero.

	Switch_VacMode(false);
	Set_MainBrush_PWM(80);
	Set_SideBrush_PWM(60,60);

	return true;
}

void MotionManage::pubCleanMapMarkers(uint8_t id, Point32_t next_point, Point32_t target_point)
{
	int16_t i, j, x_min, x_max, y_min, y_max, next_point_x, next_point_y, target_point_x, target_point_y;
	CellState cell_state;
	path_get_range(&x_min, &x_max, &y_min, &y_max);

	next_point_x = countToCell(next_point.X);
	if (next_point_x == SHRT_MIN )
		next_point_x = x_min;
	else if (next_point_x == SHRT_MAX)
		next_point_x = x_max;

	next_point_y = countToCell(next_point.Y);
	target_point_x = countToCell(target_point.X);
	target_point_y = countToCell(target_point.Y);

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
				cell_state = Map_GetCell(id, i, j);
				if (cell_state == CLEANED || cell_state == BLOCKED_OBS || cell_state == BLOCKED_BUMPER)
					robot::instance()->setCleanMapMarkers(i, j, cell_state);
			}
		}
	}
	robot::instance()->pubCleanMapMarkers();
}
