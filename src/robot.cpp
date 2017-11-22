#include <stdio.h>
#include <time.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <movement.h>
#include <motion_manage.h>
#include <core_move.h>
#include <std_srvs/SetBool.h>
#include <pp.h>

#include "std_srvs/Empty.h"

#define RAD2DEG(rad) ((rad)*57.29578)

using namespace obstacle_detector;

static	robot *robot_obj = NULL;

bool	g_is_low_bat_pause=false;
bool g_is_manual_pause=false;
time_t	start_time;


int16_t slam_error_count;

// For obs dynamic adjustment
int OBS_adjust_count = 50;


boost::mutex ros_map_mutex_;

//extern pp::x900sensor sensor;
robot::robot():offset_angle_(0),saved_offset_angle_(0)
{
	init();
	sensor_sub_ = robot_nh_.subscribe("/robot_sensor", 10, &robot::sensorCb, this);
	odom_sub_ = robot_nh_.subscribe("/odom", 1, &robot::robotOdomCb, this);
	map_sub_ = robot_nh_.subscribe("/map", 1, &robot::mapCb, this);
	robot_tf_ = new tf::TransformListener(robot_nh_, ros::Duration(0.1), true);
	/*map subscriber for exploration*/
	//map_metadata_sub = robot_nh_.subscribe("/map_metadata", 1, &robot::robot_map_metadata_cb, this);

	visualizeMarkerInit();
	send_clean_marker_pub_ = robot_nh_.advertise<visualization_msgs::Marker>("clean_markers",1);
	send_clean_map_marker_pub_ = robot_nh_.advertise<visualization_msgs::Marker>("clean_map_markers",1);
	odom_pub_ = robot_nh_.advertise<nav_msgs::Odometry>("robot_odom",1);
	scan_ctrl_pub_ = robot_nh_.advertise<pp::scan_ctrl>("scan_ctrl",1);

	is_sensor_ready_ = false;
	is_tf_ready_ = false;

	temp_spot_set_ = false;

	linear_x_ = 0.0;
	linear_y_ = 0.0;
	linear_z_ = 0.0;

	resetCorrection();

	start_time = time(NULL);

	// Initialize the low battery pause variable.
	// Initialize the key press count.
	// Initialize the manual pause variable.
	g_is_manual_pause = false;

	setBaselinkFrameType(Odom_Position_Odom_Angle);

	ROS_INFO("%s %d: robot init done!", __FUNCTION__, __LINE__);
}

robot::~robot()
{
	delete robot_tf_;

	robot_obj = NULL;
}

robot *robot::instance()
{
	return robot_obj;
}

void robot::init()
{
	if (robot_obj == NULL) {
		robot_obj = this;
	}
}

void robot::sensorCb(const pp::x900sensor::ConstPtr &msg)
{
	angle_ = msg->angle;

	angle_v_ = msg->angle_v;

	is_sensor_ready_ = true;

	// Dynamic adjust obs
	obs_dynamic_base(OBS_adjust_count);

	// Check for whether robot should publish this frame of scan.
	scan_ctrl_.allow_publishing = check_pub_scan();
	scan_ctrl_pub_.publish(scan_ctrl_);
}

void robot::robotOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
	tf::StampedTransform		transform;
	float tmp_x = 0, tmp_y = 0;
	double tmp_yaw = 0;

	linear_x_ = msg->twist.twist.linear.x;
	linear_y_ = msg->twist.twist.linear.y;
	linear_z_ = msg->twist.twist.linear.z;
	//odom_eualr_angle;

	if (getBaselinkFrameType() == Map_Position_Map_Angle)
	{
		if(MotionManage::s_slam != nullptr && MotionManage::s_slam->isMapReady() && !ev.slam_error)
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
			} catch(tf::TransformException e) {
				ROS_WARN("%s %d: Failed to compute map transform, skipping scan (%s)", __FUNCTION__, __LINE__, e.what());
				setTfReady(false);
				slam_error_count++;
				if (slam_error_count > 1)
				{
					ev.slam_error = true;
					slam_error_count = 0;
				}
				return;
			}

			if (!isTfReady())
			{
				ROS_INFO("%s %d: Set is_tf_ready_ to true.", __FUNCTION__, __LINE__);
				setTfReady(true);
				slam_error_count = 0;
			}
		}
//		if(! is_turn())
//			cm_update_map();
//		cm_update_position();
	}
	else if (getBaselinkFrameType() == Odom_Position_Odom_Angle)
	{
		//ROS_INFO("SLAM = 0");
		odom_pose_x_ = msg->pose.pose.position.x;
		odom_pose_y_ = msg->pose.pose.position.y;
		odom_pose_yaw_ = tf::getYaw(msg->pose.pose.orientation);
		tmp_x = odom_pose_x_;
		tmp_y = odom_pose_y_;
		tmp_yaw = odom_pose_yaw_;
	}
	else if (getBaselinkFrameType() == Map_Position_Odom_Angle)
	{//Wall_Follow_Mode
		//ROS_INFO("SLAM = 2");
		if(MotionManage::s_slam != nullptr && MotionManage::s_slam->isMapReady() && !ev.slam_error)
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
			} catch(tf::TransformException e) {
				ROS_WARN("%s %d: Failed to compute map transform, skipping scan (%s)", __FUNCTION__, __LINE__, e.what());
				setTfReady(false);
				slam_error_count++;
				if (slam_error_count > 1)
				{
					ev.slam_error = true;
					slam_error_count = 0;
				}
				return;
			}

			if (!isTfReady())
			{
				ROS_INFO("%s %d: Set is_tf_ready_ to true.", __FUNCTION__, __LINE__);
				setTfReady(true);
				slam_error_count = 0;
			}
		}

//		cm_update_map();
	}

#if USE_ROBOT_TF
	updateRobotPose(odom_pose_x_, odom_pose_y_, odom_pose_yaw_, slam_correction_x_, slam_correction_y_, slam_correction_yaw_, robot_correction_x_, robot_correction_y_, robot_correction_yaw_, robot_x_, robot_y_, robot_yaw_);
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
	odom.header.stamp = cur_time;
	odom.header.frame_id = "map";
	odom.child_frame_id = "robot";
	odom.pose.pose.position.x = robot_x_;
	odom.pose.pose.position.y = robot_y_;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_yaw_);
	odom.twist.twist.linear.x = 0.0;
	odom.twist.twist.linear.y = 0.0;
	odom.twist.twist.angular.z = 0.0;
	odom_pub_.publish(odom);
	//printf("Map->base(%f, %f, %f). Map->robot (%f, %f, %f)\n", tmp_x, tmp_y, RAD2DEG(tmp_yaw), robot_x_, robot_y_, RAD2DEG(robot_yaw_));
	position_x_ = robot_x_;
	position_y_ = robot_y_;
	position_yaw_ = robot_yaw_;
#else
	position_x_ = tmp_x;
	position_y_ = tmp_y;
	position_yaw_ = tmp_yaw;
#endif
	//ROS_WARN("%s %d: Position (%f, %f), yaw: %f. Odom position(%f, %f), yaw: %f.", __FUNCTION__, __LINE__, tmp_x, tmp_y, tmp_yaw, odom_pose_x_, odom_pose_y_, odom_pose_yaw_);
	//ROS_WARN("%s %d: Position diff(%f, %f), yaw diff: %f.", __FUNCTION__, __LINE__, tmp_x - odom_pose_x_, tmp_y - odom_pose_y_, tmp_yaw - odom_pose_yaw_);
	//ROS_WARN("%s %d: Odom diff(%f, %f).", __FUNCTION__, __LINE__, odom_pose_x_ - msg->pose.pose.position.x, odom_pose_y_ - msg->pose.pose.position.y);
	//ROS_WARN("%s %d: Correct  diff(%f, %f), yaw diff: %f.", __FUNCTION__, __LINE__, correct_x, correct_y, correct_yaw);
	gyro.set_angle(ranged_angle(position_yaw_ * 1800 / M_PI));
//	ROS_WARN("Position (%f, %f), angle: %d.", odom_pose_x_, odom_pose_y_, gyro.get_angle());
}

void robot::mapCb(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
	ros_map_mutex_.lock();
	width_ = map->info.width;
	height_ = map->info.height;
	resolution_ = map->info.resolution;
	origin_x_ = map->info.origin.position.x;
	origin_y_ = map->info.origin.position.y;
	map_data_ = map->data;
	map_ptr_ = &(map_data_);
	ros_map_mutex_.unlock();


	/*for exploration update map*/
	if (cm_is_exploration()) {
		ros_map_convert(MAP, false, false, true);
	}
	MotionManage::s_slam->isMapReady(true);

//	ROS_INFO("%s %d:finished map callback,map_size(\033[33m%d,%d\033[0m),resolution(\033[33m%f\033[0m),map_origin(\033[33m%f,%f\033[0m)", __FUNCTION__, __LINE__,width_,height_,resolution_,origin_x_,origin_y_);

}

//void robot::displayPositions()
//{
//	ROS_INFO("base_link->map: (%f, %f) Gyro: %d yaw_: %f(%f)",
//		position_x_, position_y_, gyro.get_angle(), position_yaw_, position_yaw_ * 1800 / M_PI);
//}

void robot::visualizeMarkerInit()
{
	clean_markers_.ns = "waypoints";
	clean_markers_.id = 0;
	clean_markers_.type = visualization_msgs::Marker::LINE_STRIP;
	clean_markers_.action= 0;//add
	clean_markers_.lifetime=ros::Duration(0);
#if __ROBOT_X400
	clean_markers_.scale.x = 0.31;
#elif __ROBOT_X900
	clean_markers_.scale.x = 0.33;
#endif
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

//void robot::pubCleanMarkers()
//{
//	m_points_.x = position_x_;
//	m_points_.y = position_y_;
//	m_points_.z = 0;
//	clean_markers_.header.stamp = ros::Time::now();
//	clean_markers_.points.push_back(m_points_);
//	send_clean_marker_pub_.publish(clean_markers_);
//}

void robot::setCleanMapMarkers(int8_t x, int8_t y, CellState type)
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
	else if (type == BLOCKED_OBS)
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
	else if (type == BLOCKED_LASER)
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
	else if (type == BLOCKED_ROS_MAP)
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

void robot::pubCleanMapMarkers(void)
{
	clean_map_markers_.header.stamp = ros::Time::now();
	send_clean_map_marker_pub_.publish(clean_map_markers_);
	clean_map_markers_.points.clear();
	clean_map_markers_.colors.clear();
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
	if (wheel.get_left_speed() * wheel.get_right_speed() > 0)
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
		//printf("Slam (%f, %f, %f). Adjust (%f, %f, %f)\n", slam_correction_x, slam_correction_y, RAD2DEG(slam_correction_yaw), robot_correction_x, robot_correction_y, RAD2DEG(robot_correction_yaw));
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


