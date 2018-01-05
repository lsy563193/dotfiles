#include "pp.h"

#include <stdio.h>
#include <time.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <global.h>
#include <std_srvs/SetBool.h>
#include <pp.h>
#include <pp/SetLidar.h>
#include <odom.h>
#include <event_manager.h>
#include "lidar.hpp"
#include "robot.hpp"

#include "std_srvs/Empty.h"

#define RAD2DEG(rad) ((rad)*57.29578)

const double CHASE_X = 0.107;

// For obs dynamic adjustment
int OBS_adjust_count = 50;

//extern pp::x900sensor sensor;
robot::robot(std::string serial_port, int baudrate, std::string lidar_bumper_dev)/*:offset_angle_(0),saved_offset_angle_(0)*/
{
	// Subscribers.
	sensor_sub_ = robot_nh_.subscribe("/robot_sensor", 10, &robot::sensorCb, this);
	odom_sub_ = robot_nh_.subscribe("/odom", 1, &robot::robotOdomCb, this);
	map_sub_ = robot_nh_.subscribe("/map", 1, &robot::mapCb, this);
	scanLinear_sub_ = robot_nh_.subscribe("scanLinear", 1, &robot::scanLinearCb, this);
	scanOriginal_sub_ = robot_nh_.subscribe("scanOriginal", 1, &robot::scanOriginalCb, this);
	scanCompensate_sub_ = robot_nh_.subscribe("scanCompensate", 1, &robot::scanCompensateCb, this);
	lidarPoint_sub_ = robot_nh_.subscribe("lidarPoint", 1, &robot::lidarPointCb, this);
	/*map subscriber for exploration*/
	//map_metadata_sub = robot_nh_.subscribe("/map_metadata", 1, &robot::robot_map_metadata_cb, this);

	// Service clients.
	lidar_motor_cli_ = robot_nh_.serviceClient<pp::SetLidar>("lidar_motor_ctrl");
	end_slam_cli_ = robot_nh_.serviceClient<std_srvs::Empty>("End_Slam");
	start_slam_cli_ = robot_nh_.serviceClient<std_srvs::Empty>("Start_Slam");
	robot_tf_ = new tf::TransformListener(robot_nh_, ros::Duration(0.1), true);

	// Publishers.
	send_clean_marker_pub_ = robot_nh_.advertise<visualization_msgs::Marker>("clean_markers", 1);
	send_clean_map_marker_pub_ = robot_nh_.advertise<visualization_msgs::Marker>("clean_map_markers", 1);
	odom_pub_ = robot_nh_.advertise<nav_msgs::Odometry>("robot_odom", 1);
	scan_ctrl_pub_ = robot_nh_.advertise<pp::scan_ctrl>("scan_ctrl", 1);
	line_marker_pub_ = robot_nh_.advertise<visualization_msgs::Marker>("line_marker", 1);
	line_marker_pub2_ = robot_nh_.advertise<visualization_msgs::Marker>("line_marker2", 1);
	point_marker_pub_ = robot_nh_.advertise<visualization_msgs::Marker>("point_marker", 1);
	fit_line_marker_pub_ = robot_nh_.advertise<visualization_msgs::Marker>("fit_line_marker", 1);

	visualizeMarkerInit();
	is_sensor_ready_ = false;
	is_tf_ready_ = false;

	temp_spot_set_ = false;

	resetCorrection();

	setBaselinkFrameType(ODOM_POSITION_ODOM_ANGLE);

	// Init for serial.
	if (!serial.init(serial_port.c_str(), baudrate))
	{
		ROS_ERROR("%s %d: Serial init failed!!", __FUNCTION__, __LINE__);
		return;
	}

#if VERIFY_CPU_ID
	if (verify_cpu_id() < 0) {
		verify_ok = false;
	}
#endif

#if VERIFY_KEY
	if (verify_ok == true && verify_key() == 0) {
		verify_ok = false;
	}
#endif

	// Init for lidar bumper.
	if (bumper.lidarBumperInit(lidar_bumper_dev.c_str()) == -1)
		ROS_ERROR(" lidar bumper open fail!");

	// Init for robotbase.
	robotbase_init();

	// Init for event manager.
	event_manager_init();
	auto event_manager_thread = new boost::thread(event_manager_thread_cb);
	event_manager_thread->detach();
	auto event_handler_thread = new boost::thread(event_handler_thread_cb);
	event_handler_thread->detach();

	// Init for core thread.
	auto core_thread = new boost::thread(boost::bind(&robot::core_thread_cb,this));
	core_thread->detach();
	ROS_INFO("%s %d: robot init done!", __FUNCTION__, __LINE__);
}

void robot::core_thread_cb()
{
	ROS_INFO("Waiting for robot sensor ready.");
	while (!isSensorReady()) {
		usleep(1000);
	}
	ROS_INFO("Robot sensor ready.");
//	speaker.play(VOICE_WELCOME_ILIFE);
	usleep(200000);

	if (charger.isOnStub() || charger.isDirected())
		p_mode.reset(new ModeCharge());
	else
	{
		speaker.play(VOICE_PLEASE_START_CLEANING, false);
		p_mode.reset(new ModeIdle());
	}

	while(ros::ok())
	{
		ROS_INFO("%s %d: %x", __FUNCTION__, __LINE__, p_mode);
		p_mode->run();
		auto next_mode = p_mode->getNextMode();
		p_mode.reset();
		ROS_INFO("%s %d: %x", __FUNCTION__, __LINE__, p_mode);
		p_mode.reset(getNextMode(next_mode));
		ROS_INFO("%s %d: %x", __FUNCTION__, __LINE__, p_mode);
	}
}

robot::~robot()
{
	bumper.lidarBumperDeinit();
	robotbase_deinit();
	delete robot_tf_;
}

robot *robot::instance()
{
	extern robot* robot_instance;
	return robot_instance;
}

void robot::sensorCb(const pp::x900sensor::ConstPtr &msg)
{
	is_sensor_ready_ = true;

	// Dynamic adjust obs
	obs.DynamicAdjust(OBS_adjust_count);

	// Check for whether robot should publish this frame of scan.
		scan_ctrl_.allow_publishing =
						!(fabs(wheel.getLeftWheelActualSpeed() - wheel.getRightWheelActualSpeed()) > 0.1
					|| (wheel.getLeftWheelActualSpeed() * wheel.getRightWheelActualSpeed() < 0)
					|| bumper.getStatus()
					|| gyro.getTiltCheckingStatus()
					|| abs(wheel.getLeftSpeedAfterPid() - wheel.getRightSpeedAfterPid()) > 100
					|| wheel.getLeftSpeedAfterPid() * wheel.getRightSpeedAfterPid() < 0);

	scan_ctrl_pub_.publish(scan_ctrl_);
}

void robot::robotOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
	tf::StampedTransform		transform;
	float tmp_x = 0, tmp_y = 0;
	double tmp_yaw = 0;

	float	odom_pose_x_;
	float	odom_pose_y_;
	double	odom_pose_yaw_;

	if (getBaselinkFrameType() == SLAM_POSITION_SLAM_ANGLE)
	{
		if(slam.isMapReady()/* && !ev.slam_error*/)
		{
			try {
				robot_tf_->lookupTransform("/map", "/base_link", ros::Time(0), transform);
				tmp_x = transform.getOrigin().x();
				tmp_y = transform.getOrigin().y();
				tmp_yaw = tf::getYaw(transform.getRotation());
				robot_tf_->lookupTransform("/odom", "/base_link", ros::Time(0), transform);
				odom_pose_x_ = transform.getOrigin().x();
				odom_pose_y_ = transform.getOrigin().y();
				odom_pose_yaw_ = tf::getYaw(transform.getRotation());
				slam_correction_x_ = tmp_x - odom_pose_x_;
				slam_correction_y_ = tmp_y - odom_pose_y_;
				slam_correction_yaw_ = tmp_yaw - odom_pose_yaw_;
			} catch(tf::TransformException e)
			{
				ROS_WARN("%s %d: Failed to compute map transform, skipping scan (%s)", __FUNCTION__, __LINE__, e.what());
			}

			if (!isTfReady())
			{
				ROS_INFO("%s %d: TF ready.", __FUNCTION__, __LINE__);
				setTfReady(true);
			}
		}
	}
	else if (getBaselinkFrameType() == ODOM_POSITION_ODOM_ANGLE)
	{
		//ROS_INFO("SLAM = 0");
		odom_pose_x_ = odom.getX();
		odom_pose_y_ = odom.getY();
		odom_pose_yaw_ = odom.getAngle() * M_PI / 180;
	}
	else if (getBaselinkFrameType() == SLAM_POSITION_ODOM_ANGLE)
	{//Wall_Follow_Mode
		//ROS_INFO("SLAM = 2");
		if(slam.isMapReady()/* && !ev.slam_error*/)
		{
			tf::Stamped<tf::Pose> ident;
			ident.setIdentity();
			ident.frame_id_ = "base_link";
			ident.stamp_ = msg->header.stamp;
			try {
				robot_tf_->lookupTransform("/map", "/base_link", ros::Time(0), transform);
				//robot_tf_->waitForTransform("/map", ros::Time::now(), ident.frame_id_, msg->header.stamp, ident.frame_id_, ros::Duration(0.5));
				//robot_tf_->lookupTransform("/map", "/base_link", ros::Time(0), transform);
				tmp_x = transform.getOrigin().x();
				tmp_y = transform.getOrigin().y();
				//robot_tf_->waitForTransform("/odom", ros::Time::now(), ident.frame_id_, msg->header.stamp, ident.frame_id_, ros::Duration(0.5));
				robot_tf_->lookupTransform("/odom", "/base_link", ros::Time(0), transform);
				tmp_yaw = tf::getYaw(transform.getRotation());
				odom_pose_x_ = transform.getOrigin().x();
				odom_pose_y_ = transform.getOrigin().y();
				odom_pose_yaw_ = tf::getYaw(transform.getRotation());
				slam_correction_x_ = tmp_x - odom_pose_x_;
				slam_correction_y_ = tmp_y - odom_pose_y_;
				slam_correction_yaw_ = tmp_yaw - odom_pose_yaw_;
			} catch(tf::TransformException e)
			{
				ROS_WARN("%s %d: Failed to compute map transform, skipping scan (%s)", __FUNCTION__, __LINE__, e.what());
			}


			if (!isTfReady())
			{
				ROS_INFO("%s %d: TF ready.", __FUNCTION__, __LINE__);
				setTfReady(true);
			}
		}
	}

#if USE_ROBOT_TF
	updateRobotPose(odom_pose_x_, odom_pose_y_, odom_pose_yaw_,
					slam_correction_x_, slam_correction_y_, slam_correction_yaw_,
					robot_correction_x_, robot_correction_y_, robot_correction_yaw_,
					robot_x_, robot_y_, robot_yaw_);
	//robot_x = (tmp_x + robot_x) / 2;
	//robot_y = (tmp_y + robot_y) / 2;
	//robot_yaw = tmp_yaw;

	// Publish the robot tf.
	ros::Time cur_time;
	//robot_trans.header.stamp = cur_time;
	//robot_trans.header.frame_id = "map";
	//robot_trans.child_frame_id = "robot";
	//geometry_msgs::Quaternion	robot_quat;
	//robot_trans.transform.translation.x = robot_x_;
	//robot_trans.transform.translation.y = robot_y_;
	//robot_trans.transform.translation.z = 0.0;
	//robot_quat = tf::createQuaternionMsgFromYaw(robot_yaw_);
	//robot_trans.transform.rotation = robot_quat;
	//robot_broad.sendTransform(robot_trans);
	//ROS_WARN("%s %d: World position (%f, %f), yaw: %f.", __FUNCTION__, __LINE__, tmp_x, tmp_y, tmp_yaw);
	nav_msgs::Odometry robot_pose;
	robot_pose.header.stamp = cur_time;
	robot_pose.header.frame_id = "map";
	robot_pose.child_frame_id = "robot";
	robot_pose.pose.pose.position.x = robot_x_;
	robot_pose.pose.pose.position.y = robot_y_;
	robot_pose.pose.pose.position.z = 0.0;
	robot_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_yaw_);
	robot_pose.twist.twist.linear.x = 0.0;
	robot_pose.twist.twist.linear.y = 0.0;
	robot_pose.twist.twist.angular.z = 0.0;
	odom_pub_.publish(robot_pose);
	//printf("Map->base(%f, %f, %f). Map->robot (%f, %f, %f)\n", tmp_x, tmp_y, RAD2DEG(tmp_yaw), robot_x_, robot_y_, RAD2DEG(robot_yaw_));
	world_pose_.setX(robot_x_);
	world_pose_.setY(robot_y_);
	world_pose_.setAngle(ranged_angle(robot_yaw_ * 1800 / M_PI));
//	ROS_WARN("Position (%f, %f), angle: %f.", world_pose_.getX(), world_pose_.getY(), world_pose_.getAngle());
#else
	pose.setX(tmp_x_);
	pose.setY(tmp_y_);
	pose.setAngle(tmp_yaw_);
#endif
	//ROS_WARN("%s %d: Position (%f, %f), yaw: %f. Odom position(%f, %f), yaw: %f.", __FUNCTION__, __LINE__, tmp_x, tmp_y, tmp_yaw, odom_pose_x_, odom_pose_y_, odom_pose_yaw_);
	//ROS_WARN("%s %d: Position diff(%f, %f), yaw diff: %f.", __FUNCTION__, __LINE__, tmp_x - odom_pose_x_, tmp_y - odom_pose_y_, tmp_yaw - odom_pose_yaw_);
	//ROS_WARN("%s %d: Odom diff(%f, %f).", __FUNCTION__, __LINE__, odom_pose_x_ - msg->pose.pose.position.x, odom_pose_y_ - msg->pose.pose.position.y);
	//ROS_WARN("%s %d: Correct  diff(%f, %f), yaw diff: %f.", __FUNCTION__, __LINE__, correct_x, correct_y, correct_yaw);
}

void robot::mapCb(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
	slam.mapCb(map);
}

void robot::scanLinearCb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	lidar.scanLinearCb(msg);
}

bool robot::check_corner(const sensor_msgs::LaserScan::ConstPtr & scan, const Paras para) {
	int forward_wall_count = 0;
	int side_wall_count = 0;
	for (int i = 359; i > 0; i--) {
		if (scan->ranges[i] < 4) {
			auto point = polar_to_cartesian(scan->ranges[i], i);
			if (para.inForwardRange(point)) {
				forward_wall_count++;
//			ROS_INFO("point(%f,%f)",point.x,point.y);
//				ROS_INFO("forward_wall_count(%d)",forward_wall_count);
			}
			if (para.inSidedRange(point)) {
				side_wall_count++;
//			ROS_INFO("point(%f,%f)",point.x,point.y);
//				ROS_INFO("side_wall_count(%d)",side_wall_count);
			}
		}
	}
	return forward_wall_count > 10 && side_wall_count > 20;
}

Vector2<double> robot::polar_to_cartesian(double polar,int i)
{
	Vector2<double> point{cos((i * 1.0 + 180.0) * PI / 180.0) * polar,
					sin((i * 1.0 + 180.0) * PI / 180.0) * polar };

	coordinate_transform(&point.x, &point.y, LIDAR_THETA, LIDAR_OFFSET_X, LIDAR_OFFSET_Y);
	return point;

}

Vector2<double> robot::get_middle_point(const Vector2<double>& p1,const Vector2<double>& p2,const Paras& para) {
	auto p3 = (p1 + p2) / 2;
	Vector2<double> target{};

//	ROS_INFO("p1(%f,%f)", p1.x, p1.y);
//	ROS_INFO("p2(%f,%f)", p2.x, p2.y);
//	ROS_INFO("p3 (%f,%f)", p3.x, p3.y);

//	auto x4 = para.narrow / (sqrt(1 + p1.SquaredDistance(p2))) + p3.x;
//	auto y4 = ((x4 - p3.x) * (p1.x - p2.x) / (p2.y - p1.y)) + p3.y;
	auto dx = para.narrow / (sqrt(1 + ((p1.x - p2.x) / (p2.y - p1.y)) * ((p1.x - p2.x) / (p2.y - p1.y))));
	auto x4 = dx + p3.x;

	auto dy = (x4 - p3.x) * (p1.x - p2.x) / (p2.y - p1.y);
	auto y4 = dy + p3.y;

//	ROS_INFO("x4,y4(%f,%f)", x4, y4);

	if (((p1.x - x4) * (p2.y - y4) - (p1.y - y4) * (p2.x - x4)) < 0) {
		target = {x4,y4};
//		ROS_INFO_FL();
//		ROS_INFO("target(%f,%f)", target.x, target.y);
	}
	else {
		x4 =  -dx + p3.x;
		y4 = (x4 - p3.x) * (p1.x - p2.x) / (p2.y - p1.y) + p3.y;
		target = {x4,y4};
//		ROS_INFO_FL();
//		ROS_INFO("target(%f,%f)", target.x, target.y);
	}
	return target;
}

bool robot::check_is_valid(const Vector2<double>& point, Paras& para, const sensor_msgs::LaserScan::ConstPtr & scan) {
	for (int i = 359; i >= 0; i--) {
		auto tmp_point = polar_to_cartesian(scan->ranges[i], i);
		auto distance = point.Distance(tmp_point);
		//ROS_INFO("distance =  %lf", distance);
		if (distance < para.narrow - 0.03) {
			return false;
		}
	}
	return true;
}

bool robot::calcLidarPath(const sensor_msgs::LaserScan::ConstPtr & scan,bool is_left, std::deque<Vector2<double>>& points) {
	Paras para{is_left};

	auto is_corner = check_corner(scan, para);
	ROS_WARN("is_corner = %d", is_corner);
	for (int i = 359; i >= 0; i--) {
		//ROS_INFO("i = %d", i);
		if (scan->ranges[i] < 4 && scan->ranges[i - 1] < 4) {
			auto point1 = polar_to_cartesian(scan->ranges[i], i);

			if (!para.inPoint1Range(point1, is_corner))
				continue;

			auto point2 = polar_to_cartesian(scan->ranges[i - 1], i - 1);

			if (point2.Distance(point1) > 0.05) {
				//ROS_INFO("two points distance is too large");
				continue;
			}
			auto target = get_middle_point(point1, point2, para);

			if (!para.inTargetRange(target))
				continue;

			if (target.Distance({0, 0}) > 0.4)
				continue;

			if (!check_is_valid(target, para, scan))
				continue;

//			ROS_INFO("points(%d):target(%lf,%lf),dis(%f)", points.size(), target.x, target.y, target.Distance({CHASE_X, 0}));
			points.push_back(target);
		}
	}

	if (points.empty()) {
		return false;
	}
	if (!is_left) {
		std::reverse(points.begin(), points.end());//for the right wall follow
	}
	auto min = std::min_element(points.rbegin(), points.rend(), [](Vector2<double>& a, Vector2<double>& b) {
//		ROS_INFO("dis(%f,%f)", a.Distance({CHASE_X, 0}), b.Distance({CHASE_X, 0}));
		return a.Distance({CHASE_X, 0}) < b.Distance({CHASE_X, 0});
	});
//	ROS_INFO("min(%f,%f)",min->x, min->y);

	auto size = points.size();
	std::copy(points.rbegin(), min+1, std::front_inserter(points));
	points.resize(size);
//	for (const auto &target :points)
//			ROS_WARN("points(%d):target(%lf,%lf),dis(%f)", points.size(), target.x, target.y, target.Distance({CHASE_X, 0}));
//	}
	robot::instance()->pubPointMarkers(&points, "base_link");

	return true;
}

void robot::scanOriginalCb(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	lidar.scanOriginalCb(scan);
	if (lidar.isScanOriginalReady() && (p_mode->action_i_ == p_mode->ac_follow_wall_left || p_mode->action_i_ == p_mode->ac_follow_wall_right)) {
		std::deque<Vector2<double>> points{};
		if (calcLidarPath(scan, p_mode->action_i_ == p_mode->ac_follow_wall_left, points)) {
			setTempTarget(points);
		}
	}
}

void robot::scanCompensateCb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	lidar.scanCompensateCb(msg);
}

void robot::lidarPointCb(const visualization_msgs::Marker &point_marker)
{
	if (lidar.isScanOriginalReady())
		lidar.lidarPointCb(point_marker);
}

void robot::visualizeMarkerInit()
{
	clean_markers_.ns = "waypoints";
	clean_markers_.id = 0;
	clean_markers_.type = visualization_msgs::Marker::LINE_STRIP;
	clean_markers_.action= 0;//add
	clean_markers_.lifetime=ros::Duration(0);

	clean_markers_.scale.x = 0.33;
//	clean_markers_.scale.y = 0.31;
	clean_markers_.color.r = 0.0;
	clean_markers_.color.g = 1.0;
	clean_markers_.color.b = 0.0;
	clean_markers_.color.a = 0.4;
	clean_markers_.header.frame_id = "/map";
	clean_markers_.header.stamp = ros::Time::now();
	m_points_.x = 0.0;
	m_points_.y = 0.0;
	m_points_.z = 0.0;
	clean_markers_.points.clear();
	clean_markers_.points.push_back(m_points_);

	clean_map_markers_.ns = "cleaning_grid_map";
	clean_map_markers_.id = 1;
	clean_map_markers_.type = visualization_msgs::Marker::POINTS;
	clean_map_markers_.action= visualization_msgs::Marker::ADD;
	clean_map_markers_.lifetime=ros::Duration(0);
	clean_map_markers_.scale.x = 0.1;
	clean_map_markers_.scale.y = 0.1;
	color_.a = 0.7;
	clean_map_markers_.header.frame_id = "/map";
	clean_map_markers_.points.clear();
	clean_map_markers_.colors.clear();
}

void robot::setCleanMapMarkers(int16_t x, int16_t y, CellState type)
{
	m_points_.x = x * (float)CELL_SIZE / 1000;
	m_points_.y = y * (float)CELL_SIZE / 1000;
	m_points_.z = 0;
	if (type == CLEANED)
	{
		// Green
		if(y%2==0)
		{
			color_.r = 0.0;
			color_.g = 0.5;
			color_.b = 0.0;
		}
		else{
			color_.r = 0.0;
			color_.g = 1.0;
			color_.b = 0.0;
		}
	}
	else if (type == BLOCKED_FW)
	{
		color_.r = 0.2;
		color_.g = 0.1;
		color_.b = 0.5;
	}
	else if (type == BLOCKED_BUMPER)
	{
		// Red
		color_.r = 1.0;
		color_.g = 0.0;
		color_.b = 0.0;
	}
	else if (type == BLOCKED_CLIFF)
	{
		// Magenta
		color_.r = 1.0;
		color_.g = 0.0;
		color_.b = 1.0;
	}
	else if (type == BLOCKED_RCON)
	{
		// White
		color_.r = 1.0;
		color_.g = 1.0;
		color_.b = 1.0;
	}
	else if (type == BLOCKED_LIDAR)
	{
		//Blue
		color_.r = 0.0;
		color_.g = 0.0;
		color_.b = 1.0;
	}
	else if (type == BLOCKED_TILT)
	{
		// Gray
		color_.r = 0.5;
		color_.g = 0.5;
		color_.b = 0.5;
	}
	else if (type == SLAM_MAP_BLOCKED)
	{
		color_.r = 0.75;
		color_.g = 0.33;
		color_.b = 0.50;
	}
	else if (type == TARGET)// Next point
	{
		// Yellow
		color_.r = 1.0;
		color_.g = 1.0;
		color_.b = 0.0;
	}
	else if (type == TARGET_CLEAN)// Target point
	{
		// Cyan
		color_.r = 0.0;
		color_.g = 1.0;
		color_.b = 1.0;
	}
	else if (type == BLOCKED_TILT)
	{
		// Gray
		color_.r = 0.5;
		color_.g = 0.5;
		color_.b = 0.5;
	}
	else if (type == BLOCKED_SLIP)
	{
		// i dont know what color it is..
		color_.r = 0.7;
		color_.g = 0.7;
		color_.b = 0.2;
	}
	clean_map_markers_.points.push_back(m_points_);
	clean_map_markers_.colors.push_back(color_);
}

void robot::pubCleanMapMarkers(GridMap& map, const std::deque<Cell_t>& path)
{
	// temp_target is valid if only path is not empty.
	if (path.empty())
		return;

	int16_t x, y, x_min, x_max, y_min, y_max;
	CellState cell_state;
	Cell_t next = path.front();
	Cell_t target = path.back();
	map.getMapRange(CLEAN_MAP, &x_min, &x_max, &y_min, &y_max);

	if (next.x == SHRT_MIN )
		next.x = x_min;
	else if (next.x == SHRT_MAX)
		next.x = x_max;

	for (x = x_min; x <= x_max; x++)
	{
		for (y = y_min; y <= y_max; y++)
		{
			if (x == target.x && y == target.y)
				robot::instance()->setCleanMapMarkers(x, y, TARGET_CLEAN);
			else if (x == next.x && y == next.y)
				robot::instance()->setCleanMapMarkers(x, y, TARGET);
			else
			{
				cell_state = map.getCell(CLEAN_MAP, x, y);
				if (cell_state > UNCLEAN && cell_state < BLOCKED_BOUNDARY )
					robot::instance()->setCleanMapMarkers(x, y, cell_state);
			}
		}
	}
	if (!path.empty())
	{
		for (auto it = path.begin(); it->x != path.back().x || it->y != path.back().y; it++)
			robot::instance()->setCleanMapMarkers(it->x, it->y, TARGET);

		robot::instance()->setCleanMapMarkers(path.back().x, path.back().y, TARGET_CLEAN);
	}

	clean_map_markers_.header.stamp = ros::Time::now();
	send_clean_map_marker_pub_.publish(clean_map_markers_);
	clean_map_markers_.points.clear();
	clean_map_markers_.colors.clear();
}

void robot::pubLineMarker(const std::vector<LineABC> *lines)
{
	visualization_msgs::Marker line_marker;
	line_marker.ns = "line_marker_2";
	line_marker.id = 0;
	line_marker.type = visualization_msgs::Marker::LINE_LIST;
	line_marker.action= 0;//add
	line_marker.lifetime=ros::Duration(0);
	line_marker.scale.x = 0.05;
	//line_marker.scale.y = 0.05;
	//line_marker.scale.z = 0.05;
	line_marker.color.r = 0.5;
	line_marker.color.g = 1.0;
	line_marker.color.b = 0.2;
	line_marker.color.a = 1.0;
	line_marker.header.frame_id = "/map";
	line_marker.header.stamp = ros::Time::now();
	geometry_msgs::Point point1;
	point1.z = 0.0;
	geometry_msgs::Point point2;
	point2.z = 0.0;
	line_marker.points.clear();
	std::vector<LineABC>::const_iterator it;
	if(!lines->empty() && lines->size() >= 2){
		for(it = lines->cbegin(); it != lines->cend();it++){
			point1.x = it->x1;
			point1.y = it->y1;
			point2.x = it->x2;
			point2.y = it->y2;
			line_marker.points.push_back(point1);
			line_marker.points.push_back(point2);
		}
		line_marker_pub2_.publish(line_marker);
		line_marker.points.clear();
	}
	/*
	else{
		line_marker.points.clear();
		line_marker_pub2.publish(line_marker);
	}
	*/

}

void robot::pubLineMarker(std::vector<std::vector<Vector2<double>> > *groups,std::string name)
{
	int points_size;
	visualization_msgs::Marker line_marker;
	line_marker.ns = name;
	line_marker.id = 0;
	line_marker.type = visualization_msgs::Marker::SPHERE_LIST;
	line_marker.action= 0;//add
	line_marker.lifetime=ros::Duration(0);
	line_marker.scale.x = 0.05;
	line_marker.scale.y = 0.05;
	line_marker.scale.z = 0.05;
	line_marker.color.r = 0.0;
	line_marker.color.g = 0.0;
	line_marker.color.b = 1.0;
	line_marker.color.a = 1.0;
	line_marker.header.frame_id = "/base_link";
	line_marker.header.stamp = ros::Time::now();
	geometry_msgs::Point lidar_points_;
	lidar_points_.x = 0.0;
	lidar_points_.y = 0.0;
	lidar_points_.z = 0.0;

	/*line_marker.pose.position.x = 0.0;
	line_marker.pose.position.y = 0.0;
	line_marker.pose.position.z = 0.0;
	line_marker.pose.orientation.x = 0.0;
	line_marker.pose.orientation.y = 0.0;
	line_marker.pose.orientation.z = 0.0;
	line_marker.pose.orientation.w = 1.0;*/
	if (!(*groups).empty()) {
		for (auto iter = (*groups).begin(); iter != (*groups).end(); ++iter) {
			/*x1 = iter->begin()->x;
			y1 = iter->begin()->y;
			x2 = (iter->end() - 1)->x;
			y2 = (iter->end() - 1)->y;*/
			points_size = iter->size();
			for (int j = 0; j < points_size; j++) {
				//line_marker.pose.position.x = (iter->begin() + j)->x;
				//line_marker.pose.position.y = (iter->begin() + j)->y;
				lidar_points_.x = (iter->begin() + j)->x;
				lidar_points_.y = (iter->begin() + j)->y;
				//ROS_INFO("lidar_points_.x = %lf lidar_points_.y = %lf",lidar_points_.x, lidar_points_.y);
				line_marker.points.push_back(lidar_points_);
			}
		}
		line_marker_pub_.publish(line_marker);
		line_marker.points.clear();
	}
}

void robot::pubFitLineMarker(visualization_msgs::Marker fit_line_marker)
{
	fit_line_marker_pub_.publish(fit_line_marker);
}

void robot::pubPointMarkers(const std::deque<Vector2<double>> *points, std::string frame_id)
{
	visualization_msgs::Marker point_marker;
	point_marker.ns = "point_marker";
	point_marker.id = 0;
	point_marker.type = visualization_msgs::Marker::SPHERE_LIST;
	point_marker.action= 0;//add
	point_marker.lifetime=ros::Duration(0),"base_link";
	point_marker.scale.x = 0.05;
	point_marker.scale.y = 0.05;
	point_marker.scale.z = 0.05;
	point_marker.color.r = 0.0;
	point_marker.color.g = 1.0;
	point_marker.color.b = 0.0;
	point_marker.color.a = 1.0;
	point_marker.header.frame_id = frame_id;
	point_marker.header.stamp = ros::Time::now();

	geometry_msgs::Point lidar_points;
	lidar_points.z = 0;
	if (!points->empty()) {
		std::string msg("");
		for (auto iter = points->cbegin(); iter != points->cend(); ++iter) {
			lidar_points.x = iter->x;
			lidar_points.y = iter->y;
			point_marker.points.push_back(lidar_points);
			msg+="("+std::to_string(iter->x)+","+std::to_string(iter->y)+"),";
		}
		point_marker_pub_.publish(point_marker);
		//ROS_INFO("%s,%d,points size:%u,points %s",__FUNCTION__,__LINE__,points->size(),msg.c_str());
		point_marker.points.clear();
		ROS_INFO("pub point!!");
	}
	else {
		point_marker.points.clear();
		point_marker_pub_.publish(point_marker);
	}
}

bool robot::lidarMotorCtrl(bool switch_)
{
	pp::SetLidar ctrl_message;
	if(switch_){
/*		ctrl_message.request.x_acc_init= gyro.getInitXAcc();
		ctrl_message.request.y_acc_init= gyro.getInitYAcc();
		ctrl_message.request.z_acc_init= gyro.getInitZAcc();*/
		ctrl_message.request.x_acc_init= 0;
		ctrl_message.request.y_acc_init= 0;
		ctrl_message.request.z_acc_init= 0;
	}
	ctrl_message.request.data = switch_;

	if (lidar_motor_cli_.call(ctrl_message))
	{
		ROS_INFO("\033[35m" "%s %d: Service response: %s" "\033[0m", __FUNCTION__, __LINE__, ctrl_message.response.message.c_str());
		return true;
	}
	return false;
}

bool robot::slamStart(void)
{
	std_srvs::Empty empty;
	ROS_INFO("%s %d: Call start slam service.", __FUNCTION__, __LINE__);
	return start_slam_cli_.call(empty);
}

bool robot::slamStop(void)
{
	std_srvs::Empty empty;
	return end_slam_cli_.call(empty);
}

void robot::initOdomPosition()
{
	//is_tf_ready_ = false;
//	position_x_=0;
//	position_y_=0;
//	position_z_=0;
	robotbase_reset_odom_pose();
	visualizeMarkerInit();
}

void robot::updateRobotPose(const float& odom_x, const float& odom_y, const double& odom_yaw,
					const float& slam_correction_x, const float& slam_correction_y, const double& slam_correction_yaw,
					float& robot_correction_x, float& robot_correction_y, double& robot_correction_yaw,
					float& robot_x, float& robot_y, double& robot_yaw)
{
	if (wheel.getLeftSpeedAfterPid() * wheel.getRightSpeedAfterPid() > 0)
	{
		float scale;
		scale = fabs(slam_correction_x - robot_correction_x) > 0.05 ? 0.1 * fabs(slam_correction_x - robot_correction_x) / 0.05 : 0.03;
		scale = scale > 1.0 ? 1.0 : scale;
		robot_correction_x += (slam_correction_x - robot_correction_x) * scale;
		scale = fabs(slam_correction_y - robot_correction_y) > 0.05 ? 0.1 * fabs(slam_correction_y - robot_correction_y) / 0.05 : 0.03;
		scale = scale > 1.0 ? 1.0 : scale;
		robot_correction_y += (slam_correction_y - robot_correction_y) * scale;
		double yaw = slam_correction_yaw - robot_correction_yaw;
		while (yaw < -3.141592)
			yaw += 6.283184;
		while (yaw > 3.141592)
			yaw -= 6.283184;
		robot_correction_yaw += (yaw) * 0.8;
//		printf("Slam (%f, %f, %f). Adjust (%f, %f, %f)\n", slam_correction_x, slam_correction_y, RAD2DEG(slam_correction_yaw), robot_correction_x, robot_correction_y, RAD2DEG(robot_correction_yaw));
	}

	robot_x = odom_x + robot_correction_x;
	robot_y = odom_y + robot_correction_y;
	robot_yaw = odom_yaw + robot_correction_yaw;
}

void robot::resetCorrection()
{
	slam_correction_x_ = 0;
	slam_correction_y_ = 0;
	slam_correction_yaw_ = 0;
	robot_correction_x_ = 0;
	robot_correction_y_ = 0;
	robot_correction_yaw_ = 0;
	robot_x_ = 0;
	robot_y_ = 0;
	robot_yaw_ = 0;
}

void robot::obsAdjustCount(int count)
{
#ifdef OBS_DYNAMIC
	OBS_adjust_count = count;
#endif
}

void robot::setTempTarget(std::deque<Vector2<double>>& points) {
	boost::mutex::scoped_lock(temp_target_mutex_);
	tmp_plan_path_.clear();

//	ROS_ERROR("curr_point(%d,%d)",getPosition().x,getPosition().y);
	for (const auto &iter : points) {
		auto target = getPosition().getRelative(int(iter.x * 1000), int(iter.y * 1000));
		tmp_plan_path_.push_back(target);
//		ROS_INFO("temp_target(%d,%d)",target.x,target.y);
	}
}

Points robot::getTempTarget() const
{
	boost::mutex::scoped_lock(temp_target_mutex_);

//	auto tmp = tmp_plan_path_;
//	tmp_plan_path_.clear();
//	return tmp;
	return tmp_plan_path_;
}
//--------------------
static int32_t xCount{}, yCount{};

Point32_t getPosition()
{
	return {(int32_t)round(xCount), (int32_t)round(yCount), robot::instance()->getWorldPoseAngle()};
}

int32_t cellToCount(int16_t i) {
	return i * CELL_COUNT_MUL;
}

void setPosition(int32_t x, int32_t y) {
	xCount = x;
	yCount = y;
}

bool isPos(int dir)
{
	return (dir == MAP_POS_X || dir == MAP_POS_Y || dir == MAP_NONE);
}

bool isXAxis(int dir)
{
	return dir == MAP_POS_X || dir == MAP_NEG_X || dir == MAP_NONE;
}
bool isYAxis(int dir)
{
	return dir == MAP_POS_Y || dir == MAP_NEG_Y || dir == MAP_NONE;
}


Point32_t updatePosition()
{
	auto pos_x = robot::instance()->getWorldPoseX() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	auto pos_y = robot::instance()->getWorldPoseY() * 1000 * CELL_COUNT_MUL / CELL_SIZE;
	setPosition(pos_x, pos_y);
//	ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
	return getPosition();
}
