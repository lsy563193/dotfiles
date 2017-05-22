//
// Created by lsy563193 on 4/25/17.
//

#include <movement.h>
#include <robot.hpp>
#include <wav.h>
#include <config.h>
#include <laser.hpp>
#include <fcntl.h>

#include "obstacle_detector.cpp"

#include "motion_manage.h"
#include <segment_set.h>
#include <slam.h>
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
			ROS_INFO("detecting line time remain %d s\n", count_n_10ms / 100);
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
	//1 start motor
	Work_Motor_Configure();

	//2 start laser
	s_laser = new Laser();
	if (!s_laser->is_ready())
		return;

#if CONTINUE_CLEANING_AFTER_CHARGE
	if (robot::instance()->isCleaningLowBatPaused())
		return;
#endif
#if MANUAL_PAUSE_CLEANING
	if (robot::instance()->isCleaningManualPaused())
		return;
#endif

	//3 calculate offsetAngle
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
		//call start slam
#if CONTINUE_CLEANING_AFTER_CHARGE
	if (!robot::instance()->isCleaningLowBatPaused())
#endif
#if MANUAL_PAUSE_CLEANING
		if (!robot::instance()->isCleaningManualPaused())
#endif
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
	}
}

MotionManage::~MotionManage()
{
	Disable_Motors();

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
