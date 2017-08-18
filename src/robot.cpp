#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <movement.h>
#include <motion_manage.h>
#include <core_move.h>
#include <wall_follow_slam.h>
#include <move_type.h>
#include <std_srvs/SetBool.h>

#include "gyro.h"
#include "robot.hpp"
#include "robotbase.h"
#include "config.h"
#include "laser.hpp"
#include "figures/segment.h"
#include "slam.h"
#include "event_manager.h"

#include "std_srvs/Empty.h"
#include "map.h"
using namespace obstacle_detector;

static	robot *robot_obj = NULL;

time_t	start_time;

// For avoid key value palse.
int8_t key_press_count;
int8_t key_release_count;

int16_t slam_error_count;

//extern pp::x900sensor sensor;
robot::robot():offset_angle_(0),saved_offset_angle_(0)
{
	init();
	sensor_sub_ = robot_nh_.subscribe("/robot_sensor", 10, &robot::sensorCb, this);
	map_sub_ = robot_nh_.subscribe("/map", 1, &robot::mapCb, this);
	robot_tf_ = new tf::TransformListener(robot_nh_, ros::Duration(0.1), true);
	/*map subscriber for exploration*/
	//map_metadata_sub = robot_nh_.subscribe("/map_metadata", 1, &robot::robot_map_metadata_cb, this);

	visualizeMarkerInit();
	send_clean_marker_pub_ = robot_nh_.advertise<visualization_msgs::Marker>("clean_markers",1);
	send_clean_map_marker_pub_ = robot_nh_.advertise<visualization_msgs::Marker>("clean_map_markers",1);
	is_moving_ = false;
	is_sensor_ready_ = false;
	is_tf_ready_ = false;

	temp_spot_set_ = false;

	bumper_left_ = 0;
	bumper_right_ = 0;

	omni_wheel_ = 0;
	linear_x_ = 0.0;
	linear_y_ = 0.0;
	linear_z_ = 0.0;

	correction_x_ = 0.0;
	correction_y_ = 0.0;
	correction_yaw_ = 0.0;

	odom_sub_ = robot_nh_.subscribe("/odom", 1, &robot::robotOdomCb, this);

	ROS_INFO("%s %d: robot init done!", __FUNCTION__, __LINE__);
	start_time = time(NULL);

	// Initialize the low battery pause variable.
	low_bat_pause_cleaning_ = false;
	// Initialize the key press count.
	key_press_count = 0;

	// Initialize the manual pause variable.
	manual_pause_cleaning_ = false;

	setBaselinkFrameType(Odom_Position_Odom_Angle);

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

	lw_crt_ = msg->lw_crt;
	
	rw_crt_ = msg->rw_crt;

	left_wall_ = msg->left_wall;

	right_wall_ = msg->right_wall;

	x_acc_ = msg->x_acc;

	y_acc_ = msg->y_acc;

	z_acc_ = msg->z_acc;

	gyro_dymc_ = msg->gyro_dymc;

	obs_left_ = msg->l_obs;

	obs_right_ = msg->r_obs;

	obs_front_ = msg->f_obs;

	bumper_right_ = msg->rbumper;

	bumper_left_ = msg->lbumper;

	omni_wheel_ = msg->omni_wheel;

	visual_wall = msg->visual_wall;

	ir_ctrl_ = msg->ir_ctrl;
	if (ir_ctrl_ > 0)
	{
		set_rcon_remote(ir_ctrl_);
	}

	charge_stub_ = msg->c_stub;//charge stub signal
	set_rcon_status(get_rcon_status() | charge_stub_);
//	if(get_rcon_status())
//	ROS_WARN("%s %d: Rcon info: %x.", __FUNCTION__, __LINE__, charge_stub_);

	key = msg->key;
	// Mark down the key if key 'clean' is pressed. These functions is for anti-shake.
	if ((key & KEY_CLEAN) && !(get_key_press() & KEY_CLEAN))
	{
		key_press_count++;
		if (key_press_count > 0)
		{
			set_key_press(KEY_CLEAN);
			key_press_count = 0;
			// When key 'clean' is triggered, it will set touch status.
			set_touch();
		}
	}
	else if (!(key & KEY_CLEAN) && (get_key_press() & KEY_CLEAN))
	{
		key_release_count++;
		if (key_release_count > 5)
		{
			reset_key_press(KEY_CLEAN);
			key_release_count = 0;
		}
	}
	else
	{
		key_press_count = 0;
		key_release_count = 0;
	}

	charge_status_ = msg->c_s; //charge status
	// Debug
	//ROS_INFO("Subscribe charger status: %d.", charge_status_);

	w_tank_ = msg->w_tank;

	battery_voltage_ = msg->batv;

	cliff_right_ = msg->rcliff;

	cliff_left_ = msg->lcliff;

	cliff_front_ = msg->fcliff;

	vacuum_selfcheck_status_ = msg->vacuum_selfcheck_status;

	lbrush_oc_ = msg->lbrush_oc;
		
	rbrush_oc_ = msg->rbrush_oc;
	
	mbrush_oc_ = msg->mbrush_oc;

	vacuum_oc_ = msg->vcum_oc;

	plan = msg->plan;
	if(plan > 0)
		set_plan_status(plan);

	is_sensor_ready_ = true;

//	if (is_sensor_ready_ == false) {
//		if (time(NULL) - start_time > 2) {
//			ROS_INFO("%s %d: Gyro starting angle: %d", __FUNCTION__, __LINE__, (int16_t)((angle * 10 + 3600)) % 3600);

//			Gyro_SetImuOffset(((int16_t)(angle * 10 + 3600)) % 3600);
//			Gyro_SetImuAngle(((int16_t)(angle * 10 + 3600)) % 3600, angle_v_);
//			is_sensor_ready_ = true;
//		}
//	} else {
//		Gyro_SetImuAngle(((int16_t)(angle * 10 + 3600)) % 3600, angle_v_);
//	}

#if 0
	ROS_INFO("%s %d:\n\t\tangle: %f\tangle_v_: %f", __FUNCTION__, __LINE__, angle, angle_v_);
	ROS_INFO("\t\tvaccum: %d\tbox: %d\tbattery_voltage: %d, brush left: %d\t brush right: %d\tbrush main: %d", vaccum, box, battery_voltage_, brush_left_, brush_right_, brush_main_);
	ROS_INFO("\t\tbumper_right_: %d\tbumper_left_: %d\tobs_left: %d\tobs_right: %d\tobs_front: %d", bumper_right_, bumper_left_, obs_left_, obs_right_ , obs_front_);
	ROS_INFO("\t\tcliff right: %d\tcliff left: %d\t cliff front: %d\t wall: %d", cliff_right_, cliff_left_, cliff_front_, wall);
	ROS_INFO("\t\trcon left: %d\trcon right: %d\trcon fl: %d\trcon fr: %d\trcon bl: %d\trcon br: %d", rcon_left_, rcon_right_, rcon_front_left_, rcon_front_right_, rcon_back_left_, rcon_back_right_);
#endif
}

void robot::robotOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{

	tf::StampedTransform		transform;
	tf::StampedTransform		correction;

	linear_x_ = msg->twist.twist.linear.x;
	linear_y_ = msg->twist.twist.linear.y;
	linear_z_ = msg->twist.twist.linear.z;
	odom_pose_x_ = msg->pose.pose.position.x;
	odom_pose_y_ = msg->pose.pose.position.y;
	//odom_eualr_angle;
	if (linear_x_ == 0.0 && linear_y_ == 0.0 && linear_z_ == 0.0) {
		is_moving_ = false;
	} else {
		is_moving_ = true;
	}

	if (getBaselinkFrameType() == Map_Position_Map_Angle)
	{
		if(MotionManage::s_slam->isMapReady() && !g_slam_error)
		{
			try {
				robot_tf_->lookupTransform("/map", "/base_link", ros::Time(0), transform);
				robot_tf_->lookupTransform("/map", "/odom", ros::Time(0), correction);
				position_x_ = transform.getOrigin().x();
				position_y_ = transform.getOrigin().y();
				yaw_ = tf::getYaw(transform.getRotation());
				correction_x_ = correction.getOrigin().x();
				correction_y_ = correction.getOrigin().y();
				correction_yaw_ = tf::getYaw(correction.getRotation());
			} catch(tf::TransformException e) {
				ROS_WARN("%s %d: Failed to compute map transform, skipping scan (%s)", __FUNCTION__, __LINE__, e.what());
				setTfReady(false);
				slam_error_count++;
				if (slam_error_count > 1)
				{
					g_slam_error = true;
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
		cm_update_position();
	}
	else if (getBaselinkFrameType() == Odom_Position_Odom_Angle)
	{
		//ROS_INFO("SLAM = 0");
		yaw_ = tf::getYaw(msg->pose.pose.orientation);
		position_x_ = odom_pose_x_;
		position_y_ = odom_pose_y_;
	}
	else if (getBaselinkFrameType() == Map_Position_Odom_Angle)
	{//Wall_Follow_Mode
		//ROS_INFO("SLAM = 2");
		if(MotionManage::s_slam->isMapReady() && !g_slam_error)
		{
			//yaw_ = tf::getYaw(msg->pose.pose.orientation);
			//wf_position_x_ = odom_pose_x_;
			//wf_position_y_ = odom_pose_y_;

			tf::Stamped<tf::Pose> ident;
			ident.setIdentity();
			ident.frame_id_ = "base_link";
			ident.stamp_ = msg->header.stamp;
			try {
				robot_tf_->lookupTransform("/map", "/base_link", ros::Time(0), transform);
				//yaw_ = tf::getYaw(transform.getRotation());
				robot_tf_->waitForTransform("/map", ros::Time::now(), ident.frame_id_, msg->header.stamp, ident.frame_id_, ros::Duration(0.5));
				robot_tf_->lookupTransform("/map", "/base_link", ros::Time(0), transform);
				//robot_tf_->lookupTransform("/map", "/odom", ros::Time(0), correction);
				position_x_ = transform.getOrigin().x();
				position_y_ = transform.getOrigin().y();
				//correction_x_ = correction.getOrigin().x();
				//correction_y_ = correction.getOrigin().y();
				//correction_yaw_ = tf::getYaw(correction.getRotation());
				robot_tf_->waitForTransform("/odom", ros::Time::now(), ident.frame_id_, msg->header.stamp, ident.frame_id_, ros::Duration(0.5));
				robot_tf_->lookupTransform("/odom", "/base_link", ros::Time(0), transform);
				wf_position_x_ = transform.getOrigin().x();
				wf_position_y_ = transform.getOrigin().y();
				yaw_ = tf::getYaw(transform.getRotation());
			} catch(tf::TransformException e) {
				ROS_WARN("%s %d: Failed to compute map transform, skipping scan (%s)", __FUNCTION__, __LINE__, e.what());
				setTfReady(false);
				slam_error_count++;
				if (slam_error_count > 1)
				{
					g_slam_error = true;
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

	gyro_set_angle(yaw_ * 1800 / M_PI);
//	ROS_WARN("Odom position (%f, %f), angle: %d.", odom_pose_x_, odom_pose_y_, gyro_get_angle());
}

void robot::mapCb(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
	width_ = map->info.width;
	height_ = map->info.height;
	resolution_ = map->info.resolution;
	origin_x_ = map->info.origin.position.x;
	origin_y_ = map->info.origin.position.y;
	map_data_ = map->data;
	map_ptr_ = &(map_data_);
	//ros_map_convert();
	MotionManage::s_slam->isMapReady(true);

	ROS_INFO("%s %d:finished map callback", __FUNCTION__, __LINE__);

}

void robot::displayPositions()
{
	ROS_INFO("base_link->map: (%f, %f) Gyro: %d yaw_: %f(%f)",
		position_x_, position_y_, gyro_get_angle(), yaw_, yaw_ * 1800 / M_PI);
}

void robot::visualizeMarkerInit(){
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

void robot::pubCleanMarkers(){
	m_points_.x = position_x_;
	m_points_.y = position_y_;
	m_points_.z = 0;
	clean_markers_.header.stamp = ros::Time::now();
	clean_markers_.points.push_back(m_points_);
	send_clean_marker_pub_.publish(clean_markers_);
}

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
		// Blue
		color_.r = 0.0;
		color_.g = 0.0;
		color_.b = 1.0;
	}
	else if (type == BLOCKED_BUMPER)
	{
		color_.r = 1.0;
		color_.g = 0.0;
		color_.b = 0.0;
	}
	else if (type == BLOCKED_CLIFF)
	{
		color_.r = 1.0;
		color_.g = 0.0;
		color_.b = 1.0;
	}
	else if (type == BLOCKED_RCON)
	{
		color_.r = 1.0;
		color_.g = 1.0;
		color_.b = 1.0;
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
		color_.r = 0.5;
		color_.g = 1.0;
		color_.b = 0.5;
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
	position_x_=0;
	position_y_=0;
	position_z_=0;
	robotbase_reset_odom_pose();
	visualizeMarkerInit();
}


/*

std::vector<int8_t> *robot::getMapData()
{
	//ROS_INFO("%s %d: return the ptr address", __FUNCTION__, __LINE__);
	//ROS_INFO("%s %d: vector_pointer_address = %d", __FUNCTION__, __LINE__, (*(ptr))[0]);
	//ROS_INFO("%s %d: seq = %d", __FUNCTION__, __LINE__, seq);
	return ptr;
}
*/

//uint32_t robot::robot_get_rcon_front_left()
//{
//	// Move the 4 bits info to lowest bits
//	//ROS_INFO("%s %d: charge_stub_: %x.", __FUNCTION__, __LINE__, (charge_stub_ & 0xf00000) >> 20);
//	return this -> rcon_front_left_ = (charge_stub_ & 0xf00000) >> 20;
//}
//
//uint32_t robot::robot_get_rcon_front_right()
//{
//	// Move the 4 bits info to lowest bits
//	//ROS_INFO("%s %d: charge_stub_:%x.", __FUNCTION__, __LINE__, (charge_stub_ & 0x0f0000) >> 16);
//	return rcon_front_right_ = (charge_stub_ & 0x0f0000) >> 16;
//}
//
//uint32_t robot::robot_get_rcon_back_left()
//{
//	// Move the 4 bits info to lowest bits
//	//ROS_INFO("%s %d: charge_stub_:%x.", __FUNCTION__, __LINE__, (charge_stub_ & 0x0000f0) >> 4);
//	return rcon_back_left_ = (charge_stub_ & 0x0000f0) >> 4;
//}
//
//uint32_t robot::robot_get_rcon_back_right()
//{
//	// Move the 4 bits info to lowest bits
//	//ROS_INFO("%s %d: charge_stub_:%x.", __FUNCTION__, __LINE__, ( charge_stub_ & 0x00000f) >> 0);
//	return rcon_back_right_ = ( charge_stub_ & 0x00000f) >> 0;
//}
//
//uint32_t robot::robot_get_rcon_left()
//{
//	// Move the 4 bits info to lowest bits
//	//ROS_INFO("%s %d: charge_stub_:%x.", __FUNCTION__, __LINE__, ( charge_stub_ & 0x00f000) >> 12);
//	return rcon_left_ = ( charge_stub_ & 0x00f000) >> 12;
//}
//
//uint32_t robot::robot_get_rcon_right()
//{
//	// Move the 4 bits info to lowest bits
//	//ROS_INFO("%s %d: charge_stub_:%x.",  __FUNCTION__, __LINE__, (charge_stub_ & 0x000f00) >> 8);
//	return rcon_right_ = (charge_stub_ & 0x000f00) >> 8;
//}

/*
bool robot::getBumperRight()
{
	return bumper_right_;
}

bool robot::getBumperLeft()
{
	return bumper_left_;
}

*/

bool robot::isTilt()
{
	return g_is_tilt;
}

