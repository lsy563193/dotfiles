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

#include "motion_controler.h"
#include <segment_set.h>
#include <slam.h>
Segment_set segmentss;
extern bool g_is_line_angle_offset;

int8_t g_enable_slam_offset = 0;

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
//		ROS_INFO("+++++++++++++++++++++++++++++");
	}
	/*else {//(is_obstacles_ready == true)
		static int count = 0;
		if(count++%300==0) {
			for (auto &s : msg->segments) {
				std::cout << "first " << s.first_point << std::endl;
				std::cout << "last " << s.last_point << std::endl;
//			查找直线经过的格子
				Point32_t p1, p2;
				p1 = PointToCount(s.first_point);
				p2 = PointToCount(s.last_point);
				cout << "$$$$$$$$$p1:" << p1.X << "," << p1.Y << endl;
				cout << "$$$$$$$$$p2:" << p2.X << "," << p2.Y << endl;
				std::vector<Point16_t> cells = greds_of_line_pass(p1, p2);
				for (auto &cell : cells) {
					std::cout << "cell:" << cell.X << "," << cell.Y << "\t";
					Point32_t p = Map_CellToPoint(cell);
					Map_SetCell(MAP, p.X, p.Y, BLOCKED_OBS);
				}
				cout << endl;
			}
			debug_map(MAP,0,0);
		}
	}*/
}

bool MotionManage::turn_to_align(void)
{

	//todo align odom
//	is_odom_ready = false;
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
	obstacles_sub.shutdown();
	if(Stop_Event())
		return false;

	ROS_INFO("Get the line");
	line_angle_ = static_cast<int16_t>(segmentss.min_distant_segment_angle() *10);
	auto angle = static_cast<int16_t>(std::abs(line_angle_));
	ROS_INFO("line detect: rotating line_angle_(%d)", line_angle_);

	if (line_angle_ > 0)
	{
		ROS_INFO("Turn_Left %d", angle);
		Turn_Left(13, angle);
	} else if (line_angle_ < 0)
	{
		ROS_INFO("Turn_Right %d", angle);
		Turn_Right(13, angle);
	}
	if(Stop_Event())
		return false;
/*
	auto count = 2;
	while (count-- != 0)
	{
		std::cout << robot::angle << std::endl;
		sleep(1);
	}*/
	g_is_line_angle_offset = true;

	return true;
}


Laser* MotionManage::s_laser = nullptr/*new Laser()*/;
Slam* MotionManage::s_slam = nullptr/*new Slam()*/;

MotionManage::MotionManage():nh_("~"),is_align_active_(false)
{
#if CONTINUE_CLEANING_AFTER_CHARGE
	if (robot::instance()->Is_Cleaning_Low_Bat_Paused())
	{
		Work_Motor_Configure();
		s_laser = new Laser();//start laser
		if (!s_laser->is_ready())
			return;
		g_enable_slam_offset = 1;
	} else
#endif
#if MANUAL_PAUSE_CLEANING
	{
		if (robot::instance()->Is_Cleaning_Manual_Paused())
		{
			Work_Motor_Configure();

			s_laser = new Laser();//start laser
			if (!s_laser->is_ready())
				return;
			g_enable_slam_offset = 1;
		} else
#endif
		{
			s_slam = new Slam();

			Work_Motor_Configure();

			s_laser = new Laser();//start laser
			if (!s_laser->is_ready())
				return;

			nh_.param<bool>("is_active_align",is_align_active_,false);
			if (Get_Clean_Mode() == Clean_Mode_Navigation && is_align_active_)
			{
				ObstacleDetector od;
				if (turn_to_align()) return;
			}

			//call start slam
			g_enable_slam_offset = 1;
			s_slam->enable_map_update();
			auto count_n_10ms = 1000;
			while (! s_slam->is_map_ready() && --count_n_10ms != 0)
			{
//		ROS_WARN("%s %d: Map is still not ready after 10s, timeout and return.", __FUNCTION__, __LINE__);
				usleep(10000);
			}
			if (count_n_10ms == 0)
			{
				ROS_INFO("%s %d: Map is still not ready after 10s, timeout and return.", __FUNCTION__, __LINE__);
			}
		}
#if MANUAL_PAUSE_CLEANING
	}
#endif
}

MotionManage::~MotionManage()
{
#if CONTINUE_CLEANING_AFTER_CHARGE
	if (robot::instance()->Is_Cleaning_Low_Bat_Paused())
	{
		Disable_Motors();
		delete s_laser;
		s_laser = nullptr;
		g_enable_slam_offset = 0;
	} else
#endif
#if MANUAL_PAUSE_CLEANING
	{
		if (robot::instance()->Is_Cleaning_Manual_Paused())
		{
			Disable_Motors();
			delete s_laser;
			s_laser = nullptr;
			g_enable_slam_offset = 0;
			wav_play(WAV_TEST_FAIL);
		}
		else
#endif
		{
			Disable_Motors();

//			if(startup_flag[lidar])
			//try 3times;make sure to stop
			if(Get_Cliff_Trig())
				wav_play(WAV_ERROR_LIFT_UP);

			wav_play(WAV_CLEANING_FINISHED);

			if(s_slam != nullptr){
				delete s_laser;
				s_laser = nullptr;
			}

			g_is_line_angle_offset = false;

			if(s_slam != nullptr){
				delete s_slam;
				s_slam= nullptr;
			}
			nh_.shutdown();
		}
#if MANUAL_PAUSE_CLEANING
	}
#endif
};

