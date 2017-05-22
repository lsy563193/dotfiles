#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include "gyro.h"
#include "robot.hpp"
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <movement.h>
#include <motion_manage.h>
#include "robotbase.h"
#include "config.h"
#include "laser.hpp"
#include "figures/segment.h"
#include "slam.h"

#include "std_srvs/Empty.h"
using namespace obstacle_detector;

extern int8_t g_enable_slam_offset;
static	robot *robot_obj = NULL;

time_t	start_time;

// For avoid key value palse.
int8_t key_press_count;
int8_t key_release_count;

//extern pp::x900sensor sensor;
robot::robot():offset_angle_(0)
{
	init();
	sensor_sub_ = robot_nh_.subscribe("/robot_sensor", 10, &robot::sensorCb, this);
	robot_tf_ = new tf::TransformListener(robot_nh_, ros::Duration(10), true);
	robot_wf_tf_ = new tf::TransformListener(robot_nh_, ros::Duration(10), true);
	/*map subscriber for exploration*/
//	map_metadata_sub = robot_nh_.subscribe("/map_metadata", 1, &robot::robot_map_metadata_cb, this);

	//odom_sub_ = robot_nh_.subscribe("/odom", 1, &robot::robotOdomCb, this);
	visualizeMarkerInit();
	send_clean_marker_pub_ = robot_nh_.advertise<visualization_msgs::Marker>("clean_markers",1);
	//send_bumper_marker_pub_ = robot_nh_.advertise<visualization_msgs::Marker>("bumper_markers_",1);
//  obstacles_pub_ = robot_nh_.advertise<Obstacles>("obstacles", 10);
//  ROS_INFO("Obstacle Detector [ACTIVE]");
	is_moving_ = false;
	is_sensor_ready_ = false;
	is_odom_ready_ = false;

	bumper_left_ = 0;
	bumper_right_ = 0;

	linear_x_ = 0.0;
	linear_y_ = 0.0;
	linear_z_ = 0.0;


	odom_sub_ = robot_nh_.subscribe("/odom", 1, &robot::robotOdomCb, this);

	ROS_INFO("%s %d: robot init done!", __FUNCTION__, __LINE__);
	start_time = time(NULL);

	// Initialize the low battery pause variable.
	low_bat_pause_cleaning_ = false;
	// Initialize the key press count.
	key_press_count = 0;
#if MANUAL_PAUSE_CLEANING
	// Initialize the manual pause variable.
	manual_pause_cleaning_ = false;
#endif
}

robot::~robot()
{
	delete robot_tf_;
	delete robot_wf_tf_;

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
//	angle = msg->angle;

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

	omni_wheel = msg->omni_wheel;

	visual_wall = msg->visual_wall;

	ir_ctrl_ = msg->ir_ctrl;
	if (ir_ctrl_ > 0)
	{
		Set_Rcon_Remote(ir_ctrl_);
	}

	charge_stub_ = msg->c_stub;//charge stub signal
	Rcon_Status |= charge_stub_;
	//ROS_INFO("%s %d: Rcon info: %x.", __FUNCTION__, __LINE__, charge_stub_);

	key = msg->key;
	// Mark down the key if key 'clean' is pressed. These functions is for anti-shake.
	if ((key & KEY_CLEAN) && !(Get_Key_Press() & KEY_CLEAN))
	{
		key_press_count++;
		if (key_press_count > 5)
		{
			Set_Key_Press(KEY_CLEAN);
			key_press_count = 0;
			// When key 'clean' is triggered, it will set touch status.
			Set_Touch();
		}
	}
	else if (!(key & KEY_CLEAN) && (Get_Key_Press() & KEY_CLEAN))
	{
		key_release_count++;
		if (key_release_count > 5)
		{
			Reset_Key_Press(KEY_CLEAN);
			key_release_count = 0;
		}
	}
	else
	{
		key_press_count = 0;
		key_release_count = 0;
	}

	charge_status_ =msg->c_s; //charge status
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
	if(plan != 0)
		Set_Plan_Status(true);

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
	double map_yaw, pitch, roll;

	tf::Matrix3x3				mat;
	tf::Stamped<tf::Pose>		ident;
	tf::StampedTransform		transform;
	tf::StampedTransform		WF_transform;
	tf::Stamped<tf::Transform>	odom_pose, map_pose, WF_map_pose;

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

	ident.setIdentity();
	ident.frame_id_ = "base_link";
	ident.stamp_ = msg->header.stamp;
	if (g_enable_slam_offset == 1){
		//ROS_INFO("SLAM = 1");
		if(MotionManage::s_slam->isMapReady()){
		try {
			robot_tf_->lookupTransform("/map", "base_link", ros::Time(0), transform);
			yaw_ = tf::getYaw(transform.getRotation());
			Gyro_SetAngle(((int16_t)(yaw_ * 1800 / M_PI + 3600)) % 3600);
			//ROS_INFO("%s %d: offset: %d", __FUNCTION__, __LINE__, ((int16_t)(yaw * 1800 / M_PI + 3600)) % 3600 - Gyro_GetAngle());
		} catch(tf::TransformException e) {
			ROS_WARN("Failed to compute map transform, skipping scan (%s)", e.what());
			return;
		}
	

		try {
			robot_tf_->waitForTransform("/map", ros::Time::now(), ident.frame_id_, msg->header.stamp, ident.frame_id_, ros::Duration(0.5));
			robot_tf_->transformPose("/map", ident, map_pose);
			mat = odom_pose.getBasis();
			mat.getEulerYPR(map_yaw, pitch, roll);
			map_yaw_ = map_yaw;
			map_pose = map_pose;
		} catch(tf::TransformException e) {
			ROS_WARN("Failed to compute map pose, skipping scan (%s)", e.what());
			return;
		}
		}
	}else if (g_enable_slam_offset == 0){
		//ROS_INFO("SLAM = 0");
		try {
			robot_tf_->lookupTransform("/odom", "base_link", ros::Time(0), transform);
			yaw_ = tf::getYaw(transform.getRotation());
			Gyro_SetAngle(((int16_t)(yaw_ * 1800 / M_PI + 3600)) % 3600);
			//ROS_INFO("%s %d: offset: %d", __FUNCTION__, __LINE__, ((int16_t)(yaw * 1800 / M_PI + 3600)) % 3600 - Gyro_GetAngle());
		} catch(tf::TransformException e) {
			ROS_WARN("Failed to compute map transform, skipping scan (%s)", e.what());
			return;
		}
	

		try {
			robot_tf_->waitForTransform("/odom", ros::Time::now(), ident.frame_id_, msg->header.stamp, ident.frame_id_, ros::Duration(0.5));
			robot_tf_->transformPose("/odom", ident, map_pose);
			mat = odom_pose.getBasis();
			mat.getEulerYPR(map_yaw, pitch, roll);
			map_yaw_ = map_yaw;
			map_pose = map_pose;
		} catch(tf::TransformException e) {
			ROS_WARN("Failed to compute map pose, skipping scan (%s)", e.what());
			return;
		}
	}else if (g_enable_slam_offset == 2){//Wall_Follow_Mode
		//ROS_INFO("SLAM = 2");
		if(MotionManage::s_slam->isMapReady()){
		try {
			robot_tf_->lookupTransform("/map", "base_link", ros::Time(0), transform);
			robot_wf_tf_->lookupTransform("/odom", "base_link", ros::Time(0), WF_transform);
			yaw_ = tf::getYaw(WF_transform.getRotation());
			Gyro_SetAngle(((int16_t)(yaw_ * 1800 / M_PI + 3600)) % 3600);
			//ROS_INFO("%s %d: offset: %d", __FUNCTION__, __LINE__, ((int16_t)(yaw * 1800 / M_PI + 3600)) % 3600 - Gyro_GetAngle());
		} catch(tf::TransformException e) {
			ROS_WARN("Failed to compute map transform, skipping scan (%s)", e.what());
			return;
		}
	

		try {
			robot_tf_->waitForTransform("/map", ros::Time::now(), ident.frame_id_, msg->header.stamp, ident.frame_id_, ros::Duration(0.5));
			robot_tf_->transformPose("/map", ident, map_pose);
			mat = odom_pose.getBasis();
			mat.getEulerYPR(map_yaw, pitch, roll);
			map_yaw_ = map_yaw;
			map_pose = map_pose;
		} catch(tf::TransformException e) {
			ROS_WARN("Failed to compute map pose, skipping scan (%s)", e.what());
			return;
		}
		try {
			robot_tf_->waitForTransform("/odom", ros::Time::now(), ident.frame_id_, msg->header.stamp, ident.frame_id_, ros::Duration(0.5));
			robot_tf_->transformPose("/odom", ident, WF_map_pose);
			mat = odom_pose.getBasis();
			mat.getEulerYPR(map_yaw, pitch, roll);
			map_yaw_ = map_yaw;
			wf_map_pose = WF_map_pose;
		} catch(tf::TransformException e) {
			ROS_WARN("Failed to compute map pose, skipping scan (%s)", e.what());
			return;
		}
		}
	}

  position_x_ = map_pose.getOrigin().x();
  position_y_ = map_pose.getOrigin().y();
	if (g_enable_slam_offset == 2){
		wf_position_x_ = WF_map_pose.getOrigin().x();
		WF_position_y_ = WF_map_pose.getOrigin().y();
	}
  if (is_odom_ready_ == false) {
    is_odom_ready_ = true;
  }
}


void robot::displayPositions()
{
	ROS_INFO("base_link->map: (%f, %f) %f(%f) Gyro: %d\tyaw_: %f(%f)",
		map_pose.getOrigin().x(), map_pose.getOrigin().y(), map_yaw_, map_yaw_ * 1800 / M_PI,
		Gyro_GetAngle(), yaw_, yaw_ * 1800 / M_PI);
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
/*
	bumper_markers_.id=1;
	bumper_markers_.type=visualization_msgs::Marker::POINTS;
	bumper_markers_.action=0;
	bumper_markers_.lifetime=ros::Duration(0);
	bumper_markers_.scale.x = 0.05;
	bumper_markers_.scale.y = 0.1;
	bumper_markers_.color.g = 1.0;
	bumper_markers_.color.r = 1.0;
	bumper_markers_.color.a =1.0;
	bumper_markers_.header.frame_id = "/map";
	bumper_markers_.header.stamp = ros::Time::now();
*/
}

void robot::pubCleanMarkers(){
	m_points_.x = position_x_;
	m_points_.y = position_y_;
	m_points_.z = 0;
	clean_markers_.header.stamp = ros::Time::now();
	clean_markers_.points.push_back(m_points_);
	send_clean_marker_pub_.publish(clean_markers_);
}

void robot::pubBumperMarkers(){
	float radius = 0.16;
	float angle = yaw_;//angle*M_PI/180.0f;//transform into angle
	float offset_x = sin(angle)*radius;
	float offset_y = cos(angle)*radius;
	m_points_.x = position_x_+offset_x;
	m_points_.y = position_y_+offset_y;
	bumper_markers_.header.stamp = ros::Time::now();
	bumper_markers_.points.push_back(m_points_);
	send_bumper_marker_pub_.publish(bumper_markers_);
}

void robot::initOdomPosition()
{
	//is_odom_ready_ = false;
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

