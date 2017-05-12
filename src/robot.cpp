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
//typedef double Angle;

extern pp::x900sensor   sensor;


time_t	start_time;

// For avoid key value palse.
int8_t key_press_count;
int8_t key_release_count;

//extern pp::x900sensor sensor;
robot::robot()
{
	this->init();
	this->robot_sensor_sub = this->robot_node_handler.subscribe("/robot_sensor", 10, &robot::robot_robot_sensor_cb, this);
	this->robot_tf = new tf::TransformListener(this->robot_node_handler, ros::Duration(10), true);
	this->robot_WF_tf = new tf::TransformListener(this->robot_node_handler, ros::Duration(10), true);
	/*map subscriber for exploration*/
//	this->map_metadata_sub = this->robot_node_handler.subscribe("/map_metadata", 1, &robot::robot_map_metadata_cb, this);

	//this->odom_sub = this->robot_node_handler.subscribe("/odom", 1, &robot::robot_odom_cb, this);
	visualize_marker_init();
	this->send_clean_marker_pub = this->robot_node_handler.advertise<visualization_msgs::Marker>("clean_markers",1);
	//this->send_bumper_marker_pub = this->robot_node_handler.advertise<visualization_msgs::Marker>("bumper_markers",1);
//  obstacles_pub_ = robot_node_handler.advertise<Obstacles>("obstacles", 10);
//  ROS_INFO("Obstacle Detector [ACTIVE]");
	this->is_moving = false;
	this->is_sensor_ready = false;
	this->is_odom_ready_ = false;

	this->bumper_left = 0;
	this->bumper_right = 0;

	this->linear_x = 0.0;
	this->linear_y = 0.0;
	this->linear_z = 0.0;


	this->odom_sub = this->robot_node_handler.subscribe("/odom", 1, &robot::robot_odom_cb, this);

	ROS_INFO("%s %d: robot init done!", __FUNCTION__, __LINE__);
	start_time = time(NULL);

	// Initialize the low battery pause variable.
	this->low_bat_pause_cleaning = false;
	// Initialize the key press count.
	key_press_count = 0;
#if MANUAL_PAUSE_CLEANING
	// Initialize the manual pause variable.
	this->manual_pause_cleaning = false;
#endif
}

robot::~robot()
{
	delete this->robot_tf;
	delete this->robot_WF_tf;

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

bool robot::robot_is_all_ready() {
  return (is_sensor_ready ) ? true : false;
}

void robot::robot_robot_sensor_cb(const pp::x900sensor::ConstPtr& msg)
{
//	this->angle = msg->angle;

	this->angle_v = msg->angle_v;

	this->lw_crt = msg->lw_crt;
	
	this->rw_crt = msg->rw_crt;

	this->left_wall = msg->left_wall;

	this->right_wall = msg->right_wall;

	this->x_acc = msg->x_acc;

	this->y_acc = msg->y_acc;

	this->z_acc = msg->z_acc;

	this->gyro_dymc = msg->gyro_dymc;

	this->obs_left = msg->l_obs;

	this->obs_right = msg->r_obs;

	this->obs_front = msg->f_obs;

	this->bumper_right = msg->rbumper;

	this->bumper_left = msg->lbumper;

	this->omni_wheel = msg->omni_wheel;

	this->visual_wall = msg->visual_wall;

	this->ir_ctrl = msg->ir_ctrl;
	if (this->ir_ctrl > 0)
	{
		Set_Rcon_Remote(this->ir_ctrl);
	}

	this->charge_stub = msg->c_stub;//charge stub signal
	Rcon_Status |= this->charge_stub;
	//ROS_INFO("%s %d: Rcon info: %x.", __FUNCTION__, __LINE__, this->charge_stub);

	this->key = msg->key;
	// Mark down the key if key 'clean' is pressed. These functions is for anti-shake.
	if ((this->key & KEY_CLEAN) && !(Get_Key_Press() & KEY_CLEAN))
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
	else if (!(this->key & KEY_CLEAN) && (Get_Key_Press() & KEY_CLEAN))
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

	this->charge_status =msg->c_s; //charge status
	// Debug
	//ROS_INFO("Subscribe charger status: %d.", this->charge_status);

	this->w_tank = msg->w_tank;

	this->battery_voltage = msg->batv;

	this->cliff_right = msg->rcliff;

	this->cliff_left = msg->lcliff;

	this->cliff_front = msg->fcliff;

	this->lbrush_oc = msg->lbrush_oc;
		
	this->rbrush_oc = msg->rbrush_oc;
	
	this->mbrush_oc = msg->mbrush_oc;

	this->vacuum_oc = msg->vcum_oc;

	this->plan = msg->plan;
	if(this->plan != 0)
		Set_Plan_Status(true);

	this->is_sensor_ready = true;

//	if (this->is_sensor_ready == false) {
//		if (time(NULL) - start_time > 2) {
//			ROS_INFO("%s %d: Gyro starting angle: %d", __FUNCTION__, __LINE__, (int16_t)((this->angle * 10 + 3600)) % 3600);

//			Gyro_SetImuOffset(((int16_t)(this->angle * 10 + 3600)) % 3600);
//			Gyro_SetImuAngle(((int16_t)(this->angle * 10 + 3600)) % 3600, this->angle_v);
//			this->is_sensor_ready = true;
//		}
//	} else {
//		Gyro_SetImuAngle(((int16_t)(this->angle * 10 + 3600)) % 3600, this->angle_v);
//	}

#if 0
	ROS_INFO("%s %d:\n\t\tangle: %f\tangle_v: %f", __FUNCTION__, __LINE__, angle, angle_v);
	ROS_INFO("\t\tvaccum: %d\tbox: %d\tbattery_voltage: %d, brush left: %d\t brush right: %d\tbrush main: %d", vaccum, box, battery_voltage, brush_left, brush_right, brush_main);
	ROS_INFO("\t\tbumper_right: %d\tbumper_left: %d\tobs_left: %d\tobs_right: %d\tobs_front: %d", bumper_right, bumper_left, obs_left, obs_right , obs_front);
	ROS_INFO("\t\tcliff right: %d\tcliff left: %d\t cliff front: %d\t wall: %d", cliff_right, cliff_left, cliff_front, wall);
	ROS_INFO("\t\trcon left: %d\trcon right: %d\trcon fl: %d\trcon fr: %d\trcon bl: %d\trcon br: %d", rcon_left, rcon_right, rcon_front_left, rcon_front_right, rcon_back_left, rcon_back_right);
#endif
}

void robot::robot_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
	double map_yaw, pitch, roll;

	tf::Matrix3x3				mat;
	tf::Stamped<tf::Pose>		ident;
	tf::StampedTransform		transform;
	tf::StampedTransform		WF_transform;
	tf::Stamped<tf::Transform>	odom_pose, map_pose, WF_map_pose;

	this->linear_x = msg->twist.twist.linear.x;
	this->linear_y = msg->twist.twist.linear.y;
	this->linear_z = msg->twist.twist.linear.z;
	this->odom_pose_x = msg->pose.pose.position.x;
	this->odom_pose_y = msg->pose.pose.position.y;
	//this->odom_eualr_angle;
	if (this->linear_x == 0.0 && this->linear_y == 0.0 && this->linear_z == 0.0) {
		this->is_moving = false;
	} else {
		this->is_moving = true;
	}

	ident.setIdentity();
	ident.frame_id_ = "base_link";
	ident.stamp_ = msg->header.stamp;
	if (g_enable_slam_offset == 1){
		//ROS_INFO("SLAM = 1");
		if(MotionManage::s_slam->is_map_ready()){
		try {
			this->robot_tf->lookupTransform("/map", "base_link", ros::Time(0), transform);
			this->yaw = tf::getYaw(transform.getRotation());
			Gyro_SetAngle(((int16_t)(this->yaw * 1800 / M_PI + 3600)) % 3600);
			//ROS_INFO("%s %d: offset: %d", __FUNCTION__, __LINE__, ((int16_t)(this->yaw * 1800 / M_PI + 3600)) % 3600 - Gyro_GetAngle());
		} catch(tf::TransformException e) {
			ROS_WARN("Failed to compute map transform, skipping scan (%s)", e.what());
			return;
		}
	

		try {
			this->robot_tf->waitForTransform("/map", ros::Time::now(), ident.frame_id_, msg->header.stamp, ident.frame_id_, ros::Duration(0.5));
			this->robot_tf->transformPose("/map", ident, map_pose);
			mat = odom_pose.getBasis();
			mat.getEulerYPR(map_yaw, pitch, roll);
			this->map_yaw = map_yaw;
			this->map_pose = map_pose;
		} catch(tf::TransformException e) {
			ROS_WARN("Failed to compute map pose, skipping scan (%s)", e.what());
			return;
		}
		}
	}else if (g_enable_slam_offset == 0){
		//ROS_INFO("SLAM = 0");
		try {
			this->robot_tf->lookupTransform("/odom", "base_link", ros::Time(0), transform);
			this->yaw = tf::getYaw(transform.getRotation());
			Gyro_SetAngle(((int16_t)(this->yaw * 1800 / M_PI + 3600)) % 3600);
			//ROS_INFO("%s %d: offset: %d", __FUNCTION__, __LINE__, ((int16_t)(this->yaw * 1800 / M_PI + 3600)) % 3600 - Gyro_GetAngle());
		} catch(tf::TransformException e) {
			ROS_WARN("Failed to compute map transform, skipping scan (%s)", e.what());
			return;
		}
	

		try {
			this->robot_tf->waitForTransform("/odom", ros::Time::now(), ident.frame_id_, msg->header.stamp, ident.frame_id_, ros::Duration(0.5));
			this->robot_tf->transformPose("/odom", ident, map_pose);
			mat = odom_pose.getBasis();
			mat.getEulerYPR(map_yaw, pitch, roll);
			this->map_yaw = map_yaw;
			this->map_pose = map_pose;
		} catch(tf::TransformException e) {
			ROS_WARN("Failed to compute map pose, skipping scan (%s)", e.what());
			return;
		}
	}else if (g_enable_slam_offset == 2){//Wall_Follow_Mode
		//ROS_INFO("SLAM = 2");
		if(MotionManage::s_slam->is_map_ready()){
		try {
			this->robot_tf->lookupTransform("/map", "base_link", ros::Time(0), transform);
			this->robot_WF_tf->lookupTransform("/odom", "base_link", ros::Time(0), WF_transform);
			this->yaw = tf::getYaw(WF_transform.getRotation());
			Gyro_SetAngle(((int16_t)(this->yaw * 1800 / M_PI + 3600)) % 3600);
			//ROS_INFO("%s %d: offset: %d", __FUNCTION__, __LINE__, ((int16_t)(this->yaw * 1800 / M_PI + 3600)) % 3600 - Gyro_GetAngle());
		} catch(tf::TransformException e) {
			ROS_WARN("Failed to compute map transform, skipping scan (%s)", e.what());
			return;
		}
	

		try {
			this->robot_tf->waitForTransform("/map", ros::Time::now(), ident.frame_id_, msg->header.stamp, ident.frame_id_, ros::Duration(0.5));
			this->robot_tf->transformPose("/map", ident, map_pose);
			mat = odom_pose.getBasis();
			mat.getEulerYPR(map_yaw, pitch, roll);
			this->map_yaw = map_yaw;
			this->map_pose = map_pose;
		} catch(tf::TransformException e) {
			ROS_WARN("Failed to compute map pose, skipping scan (%s)", e.what());
			return;
		}
		try {
			this->robot_tf->waitForTransform("/odom", ros::Time::now(), ident.frame_id_, msg->header.stamp, ident.frame_id_, ros::Duration(0.5));
			this->robot_tf->transformPose("/odom", ident, WF_map_pose);
			mat = odom_pose.getBasis();
			mat.getEulerYPR(map_yaw, pitch, roll);
			this->map_yaw = map_yaw;
			this->WF_map_pose = WF_map_pose;
		} catch(tf::TransformException e) {
			ROS_WARN("Failed to compute map pose, skipping scan (%s)", e.what());
			return;
		}
		}
	}

  this->position_x = map_pose.getOrigin().x();
  this->position_y = map_pose.getOrigin().y();
	if (g_enable_slam_offset == 2){
		this->WF_position_x = WF_map_pose.getOrigin().x();
		this->WF_position_y = WF_map_pose.getOrigin().y();
	}
  if (this->is_odom_ready_ == false) {
    this->is_odom_ready_ = true;
  }
}
/*

std::vector<int8_t> *robot::robot_get_map_data()
{
	//ROS_INFO("%s %d: return the ptr address", __FUNCTION__, __LINE__);
	//ROS_INFO("%s %d: vector_pointer_address = %d", __FUNCTION__, __LINE__, (*(this->ptr))[0]);
	//ROS_INFO("%s %d: seq = %d", __FUNCTION__, __LINE__, this->seq);
	return this->ptr;
}
*/

float robot::robot_get_angle() {
  return this->angle;
}

void robot::set_angle(float angle_) {
	this->angle = angle_;
}

float robot::robot_get_angle_v()
{
	return this->angle_v;
}

int16_t robot::robot_get_cliff_right()
{
	return sensor.rcliff;
}

int16_t robot::robot_get_cliff_left()
{
	return sensor.lcliff;
}

int16_t robot::robot_get_cliff_front()
{
	//ROS_INFO("Cliff_Front = %d", sensor.fcliff);
	//ROS_INFO("Topic_Cliff_Front = %d", this->cliff_front);
	return sensor.fcliff;
}

int16_t robot::robot_get_left_wall()
{
	return sensor.left_wall - Left_Wall_BaseLine;
}

int16_t robot::robot_get_right_wall()
{
#if __ROBOT_X900
	return sensor.right_wall - Right_Wall_BaseLine;
#elif __ROBOT_X400
	return 0;
#endif
}

int16_t robot::robot_get_omni_wheel()
{
#if __ROBOT_X9000
	   return sensor.omni_wheel;
#elif __ROBOT_X400
	   return 0;
#endif
}

int16_t robot::robot_get_visual_wall()
{
#if __ROBOT_X900
	return sensor.visual_wall;
#elif __ROBOT_X400
	return 0;
#endif
}

bool robot::robot_get_lbrush_oc()//oc : over current
{
	return this->lbrush_oc;
}

bool robot::robot_get_rbrush_oc()
{
	return this->rbrush_oc;
}

bool robot::robot_get_mbrush_oc()
{
	return this->mbrush_oc;
}

bool robot::robot_get_vacuum_oc()
{
	return this->vacuum_oc;
}

int robot::robot_get_charge_status()
{
	return this->charge_status;
}

uint8_t robot::robot_get_key(){
	return this->key;
}
uint8_t robot::robot_get_ir_ctrl()
{
	return  this->ir_ctrl;
}

float robot::robot_get_lwheel_current()
{
	return this->lw_crt;
}

float robot::robot_get_rwheel_current()
{
	return this->rw_crt;
}

uint32_t robot::robot_get_rcon()
{
	return this->charge_stub;
}

//uint32_t robot::robot_get_rcon_front_left()
//{
//	// Move the 4 bits info to lowest bits
//	//ROS_INFO("%s %d: charge_stub: %x.", __FUNCTION__, __LINE__, (this->charge_stub & 0xf00000) >> 20);
//	return this -> rcon_front_left = (this->charge_stub & 0xf00000) >> 20;
//}
//
//uint32_t robot::robot_get_rcon_front_right()
//{
//	// Move the 4 bits info to lowest bits
//	//ROS_INFO("%s %d: charge_stub:%x.", __FUNCTION__, __LINE__, (this->charge_stub & 0x0f0000) >> 16);
//	return this->rcon_front_right = (this->charge_stub & 0x0f0000) >> 16;
//}
//
//uint32_t robot::robot_get_rcon_back_left()
//{
//	// Move the 4 bits info to lowest bits
//	//ROS_INFO("%s %d: charge_stub:%x.", __FUNCTION__, __LINE__, (this->charge_stub & 0x0000f0) >> 4);
//	return this->rcon_back_left = (this->charge_stub & 0x0000f0) >> 4;
//}
//
//uint32_t robot::robot_get_rcon_back_right()
//{
//	// Move the 4 bits info to lowest bits
//	//ROS_INFO("%s %d: charge_stub:%x.", __FUNCTION__, __LINE__, (this-> charge_stub & 0x00000f) >> 0);
//	return this->rcon_back_right = (this-> charge_stub & 0x00000f) >> 0;
//}
//
//uint32_t robot::robot_get_rcon_left()
//{
//	// Move the 4 bits info to lowest bits
//	//ROS_INFO("%s %d: charge_stub:%x.", __FUNCTION__, __LINE__, (this-> charge_stub & 0x00f000) >> 12);
//	return this->rcon_left = (this-> charge_stub & 0x00f000) >> 12;
//}
//
//uint32_t robot::robot_get_rcon_right()
//{
//	// Move the 4 bits info to lowest bits
//	//ROS_INFO("%s %d: charge_stub:%x.",  __FUNCTION__, __LINE__, (this->charge_stub & 0x000f00) >> 8);
//	return this->rcon_right = (this->charge_stub & 0x000f00) >> 8;
//}
/*
bool robot::robot_get_bumper_right()
{
	return this->bumper_right;
}

bool robot::robot_get_bumper_left()
{
	return this->bumper_left;
}

*/
bool robot::robot_get_bumper_right()
{
	//ROS_INFO("rbumper = %d",sensor.rbumper);
	//ROS_INFO("topic_rbumper = %d",this->bumper_right);
	return sensor.rbumper;
}

bool robot::robot_get_bumper_left()
{
	//ROS_INFO("lbumper = %d",sensor.lbumper);
	return sensor.lbumper;
}
int16_t robot::robot_get_obs_left()
{
	return sensor.l_obs;
}

int16_t robot::robot_get_obs_right()
{
	return sensor.r_obs;
}

int16_t robot::robot_get_obs_front()
{
	return sensor.f_obs;
}

bool robot::robot_get_water_tank()
{
	return this->w_tank;
}

uint16_t robot::robot_get_battery_voltage()
{
	return this->battery_voltage*10;
}

bool robot::robot_is_moving()
{
	return this->is_moving;
}

float robot::robot_get_linear_x()
{
	return this->linear_x;
}

float robot::robot_get_linear_y()
{
	return this->linear_y;
}

float robot::robot_get_linear_z()
{
	return this->linear_z;
}

float robot::robot_get_position_x()
{
	return this->position_x;
}

float robot::robot_get_position_y()
{
	return this->position_y;
}

float robot::robot_get_WF_position_x()
{
	return this->WF_position_x;
}

float robot::robot_get_WF_position_y()
{
	return this->WF_position_y;
}

float robot::robot_get_position_z()
{
	return this->position_z;
}

float robot::robot_get_odom_position_x()
{
	return this->odom_pose_x;
}

float robot::robot_get_odom_position_y()
{
	return this->odom_pose_y;
}

int16_t robot::robot_get_yaw()
{
	return ((int16_t)(this->yaw * 1800 / M_PI));
}

void robot::robot_display_positions() {
	ROS_INFO("base_link->map: (%f, %f) %f(%f) Gyro: %d\tyaw: %f(%f)",
		this->map_pose.getOrigin().x(), this->map_pose.getOrigin().y(), this->map_yaw, this->map_yaw * 1800 / M_PI,
		Gyro_GetAngle(), this->yaw, this->yaw * 1800 / M_PI);
}

void robot::visualize_marker_init(){
	this->clean_markers.ns = "waypoints";
	this->clean_markers.id = 0;
	this->clean_markers.type = visualization_msgs::Marker::LINE_STRIP;
	this->clean_markers.action= 0;//add
	this->clean_markers.lifetime=ros::Duration(0);
#if __ROBOT_X400
	this->clean_markers.scale.x = 0.31;
#elif __ROBOT_X900
	this->clean_markers.scale.x = 0.33;
#endif
//	this->clean_markers.scale.y = 0.31;
	this->clean_markers.color.r = 0.0;
	this->clean_markers.color.g = 1.0;
	this->clean_markers.color.b = 0.0;
	this->clean_markers.color.a = 0.4;
	this->clean_markers.header.frame_id = "/map";
	this->clean_markers.header.stamp = ros::Time::now();
	this->m_points.x = 0.0;
	this->m_points.y = 0.0;
	this->m_points.z = 0.0;
	this->clean_markers.points.clear();
	this->clean_markers.points.push_back(m_points);
/*
	this->bumper_markers.id=1;
	this->bumper_markers.type=visualization_msgs::Marker::POINTS;
	this->bumper_markers.action=0;
	this->bumper_markers.lifetime=ros::Duration(0);
	this->bumper_markers.scale.x = 0.05;
	this->bumper_markers.scale.y = 0.1;
	this->bumper_markers.color.g = 1.0;
	this->bumper_markers.color.r = 1.0;
	this->bumper_markers.color.a =1.0;
	this->bumper_markers.header.frame_id = "/map";
	this->bumper_markers.header.stamp = ros::Time::now();
*/
}

double robot::robot_get_map_yaw()
{
	return this->yaw;
}

void robot::pub_clean_markers(){
	this->m_points.x = this->position_x;
	this->m_points.y = this->position_y;
	this->m_points.z = 0;
	this->clean_markers.header.stamp = ros::Time::now();
	this->clean_markers.points.push_back(this->m_points);
	this->send_clean_marker_pub.publish(this->clean_markers);
}

void robot::pub_bumper_markers(){
	float radius = 0.16;
	float angle = this->yaw;//this->angle*M_PI/180.0f;//transform into angle
	float offset_x = sin(angle)*radius;
	float offset_y = cos(angle)*radius;
	this->m_points.x = this->position_x+offset_x;
	this->m_points.y = this->position_y+offset_y;
	this->bumper_markers.header.stamp = ros::Time::now();
	this->bumper_markers.points.push_back(this->m_points);
	this->send_bumper_marker_pub.publish(this->bumper_markers);
}

void robot::init_mumber()
{
	//is_odom_ready_ = false;
	position_x=0;
	position_y=0;
	position_z=0;
	robotbase_reset_odom_pose();
	this->visualize_marker_init();
}

#if CONTINUE_CLEANING_AFTER_CHARGE
// This 3 functions is for declaring whether the robot is at status of pausing for charge.
bool robot::Is_Cleaning_Low_Bat_Paused(void)
{
	return this->low_bat_pause_cleaning;
}
void robot::Set_Cleaning_Low_Bat_Pause(void)
{
	this->low_bat_pause_cleaning = true;
}
void robot::Reset_Cleaning_Low_Bat_Pause(void)
{
	this->low_bat_pause_cleaning = false;
}
#endif

#if MANUAL_PAUSE_CLEANING
// These 3 functions are for manual pause cleaning.
bool robot::Is_Cleaning_Manual_Paused(void)
{
	return this->manual_pause_cleaning;
}
void robot::Set_Cleaning_Manual_Pause(void)
{
	this->manual_pause_cleaning = true;
}
void robot::Reset_Cleaning_Manual_Pause(void)
{
	this->manual_pause_cleaning = false;
}
#endif
