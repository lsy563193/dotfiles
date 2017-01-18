#include <stdio.h>
#include <time.h>
#include <math.h>

#include "gyro.h"
#include "robot.hpp"
#include "robotbase.h"
static	robot *robot_obj = NULL;

time_t	start_time;

//extern pp::x900sensor sensor;
robot::robot()
{
	this->init();
	this->robot_sensor_sub = this->robot_node_handler.subscribe("/robot_sensor", 1, &robot::robot_robot_sensor_cb, this);
	this->odom_sub = this->robot_node_handler.subscribe("/odom", 1, &robot::robot_odom_cb, this);
	this->send_clean_marker_pub = this->robot_node_handler.advertise<visualization_msgs::Marker>("clean_markers",1);
	this->send_bumper_marker_pub = this->robot_node_handler.advertise<visualization_msgs::Marker>("bumper_markers",1);
	this->robot_tf = new tf::TransformListener(this->robot_node_handler, ros::Duration(10), true);
	this->map_metadata_sub = this->robot_node_handler.subscribe("/map_metadata", 1, &robot::robot_map_metadata_cb, this);

	this->is_moving = false;
	this->is_sensor_ready = false;
	this->is_scan_ready = false;
	this->is_map_ready = false;

	this->bumper_left = 0;
	this->bumper_right = 0;

	this->linear_x = 0.0;
	this->linear_y = 0.0;
	this->linear_z = 0.0;

	printf("%s %d: robot init done!\n", __FUNCTION__, __LINE__);
	start_time = time(NULL);
	visualize_marker_init();
}

robot::~robot()
{
	delete this->robot_tf;

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

bool robot::robot_is_all_ready()
{
	return (this->is_sensor_ready && this->is_scan_ready && this->is_map_ready) ? true : false;
}

void robot::robot_robot_sensor_cb(const pp::x900sensor::ConstPtr& msg)
{
	this->angle = msg->angle;

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

	this->ir_ctrl = msg->ir_ctrl;

	this->charge_stub = msg->c_stub;//charge stub signal

	this->key = msg->key;

	this->charge_status =msg->c_s; //charge status

	this->w_tank = msg->w_tank;

	this->battery_voltage = msg->batv;

	this->cliff_right = msg->rcliff;

	this->cliff_left = msg->lcliff;

	this->cliff_front = msg->fcliff;

	this->lbrush_oc = msg->lbrush_oc;
		
	this->rbrush_oc = msg->rbrush_oc;
	
	this->mbrush_oc = msg->mbrush_oc;

	this->vacuum_oc = msg->vcum_oc;

	if (this->is_sensor_ready == false) {
		if (time(NULL) - start_time > 2) {
			//printf("%s %d: Gyro starting angle: %d\n", __FUNCTION__, __LINE__, (int16_t)((this->angle * 10 + 3600)) % 3600);

			Gyro_SetImuOffset(((int16_t)(this->angle * 10 + 3600)) % 3600);
			Gyro_SetImuAngle(((int16_t)(this->angle * 10 + 3600)) % 3600, this->angle_v);
			this->is_sensor_ready = true;
		}
	} else {
		Gyro_SetImuAngle(((int16_t)(this->angle * 10 + 3600)) % 3600, this->angle_v);
	}

#if 0
	printf("%s %d:\n\t\tangle: %f\tangle_v: %f\n", __FUNCTION__, __LINE__, angle, angle_v);
	printf("\t\tvaccum: %d\tbox: %d\tbattery_voltage: %d, brush left: %d\t brush right: %d\tbrush main: %d\n", vaccum, box, battery_voltage, brush_left, brush_right, brush_main);
	printf("\t\tbumper_right: %d\tbumper_left: %d\tobs_left: %d\tobs_right: %d\tobs_front: %d\n", bumper_right, bumper_left, obs_left, obs_right , obs_front);
	printf("\t\tcliff right: %d\tcliff left: %d\t cliff front: %d\t wall: %d\n", cliff_right, cliff_left, cliff_front, wall);
	printf("\t\trcon left: %d\trcon right: %d\trcon fl: %d\trcon fr: %d\trcon bl: %d\trcon br: %d\n", rcon_left, rcon_right, rcon_front_left, rcon_front_right, rcon_back_left, rcon_back_right);
#endif
}

void robot::robot_map_metadata_cb(const nav_msgs::MapMetaData::ConstPtr& msg)
{
	static int count = 0;

	count++;
	if (count > 1) {
		this->is_map_ready = true;
	}
}

void robot::robot_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
	double map_yaw, pitch, roll;

	tf::Matrix3x3				mat;
	tf::Stamped<tf::Pose>		ident;
	tf::StampedTransform		transform;
	tf::Stamped<tf::Transform>	odom_pose, map_pose;

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

	try {
		this->robot_tf->lookupTransform("/map", "base_link", ros::Time(0), transform);
		this->yaw = tf::getYaw(transform.getRotation());

		Gyro_SetAngle(((int16_t)(this->yaw * 1800 / M_PI + 3600)) % 3600, this->angle_v);
		//printf("%s %d: offset: %d\n", __FUNCTION__, __LINE__, ((int16_t)(this->yaw * 1800 / M_PI + 3600)) % 3600 - Gyro_GetAngle(0));
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

	if (!this->is_scan_ready) {
		this->position_x_off = map_pose.getOrigin().x();
		this->position_y_off = map_pose.getOrigin().y();
		this->position_z_off = map_pose.getOrigin().z();
	}

	this->position_y = map_pose.getOrigin().y() - position_y_off;
	this->position_x = map_pose.getOrigin().x() - position_x_off;

	if (this->is_scan_ready == false) {
		this->is_scan_ready = true;
	}
}

float robot::robot_get_angle()
{
	return this->angle;
}

float robot::robot_get_angle_v()
{
	return this->angle_v;
}

int16_t robot::robot_get_cliff_right()
{
	return this->cliff_right;
}

int16_t robot::robot_get_cliff_left()
{
	return this->cliff_left;
}

int16_t robot::robot_get_cliff_front()
{
	return this->cliff_front;
}

int16_t robot::robot_get_wall()
{
	return this->left_wall;
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

bool robot::robot_get_charge_status()
{
	return this->charge_status;
}

uint8_t robot::robot_get_key(){
	return this->key;
}
uint8_t robot::robot_get_ir_ctrl()
{
	return this->ir_ctrl;
}

float robot::robot_get_lwheel_current()
{
	return this->lw_crt;
}

float robot::robot_get_rwheel_current()
{
	return this->rw_crt;
}
uint32_t robot::robot_get_rcon_front_left()
{
	return this -> rcon_front_left = this->charge_stub & 0x000000f0;
}

uint32_t robot::robot_get_rcon_front_right()
{
	return this->rcon_front_right = this->charge_stub & 0x0000000f;
}

uint32_t robot::robot_get_rcon_back_left()
{
	return this->rcon_back_left = this->charge_stub & 0x00f00000;
}

uint32_t robot::robot_get_rcon_back_right()
{
	return this->rcon_back_right = this-> charge_stub & 0x000f0000;
}

uint32_t robot::robot_get_rcon_left()
{
	return this->rcon_left = this-> charge_stub & 0x0000f000;
}

uint32_t robot::robot_get_rcon_right()
{
	return this->rcon_right = this->charge_stub & 0x00000f00;
}

bool robot::robot_get_bumper_right()
{
	return this->bumper_right;
}

bool robot::robot_get_bumper_left()
{
	return this->bumper_left;
}

int16_t robot::robot_get_obs_left()
{
	return this->obs_left;
}

int16_t robot::robot_get_obs_right()
{
	return this->obs_right;
}

int16_t robot::robot_get_obs_front()
{
	return this->obs_front;
}

bool robot::robot_get_water_tank()
{
	return this->w_tank;
}

uint16_t robot::robot_get_battery_voltage()
{
	return this->battery_voltage;
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

float robot::robot_get_position_z()
{
	return this->position_z;
}

int16_t robot::robot_get_yaw()
{
	return ((int16_t)(this->yaw * 1800 / M_PI));
}

void robot::robot_display_positions()
{
	printf("base_link->map: (%f, %f) %f(%f) Gyro: %d\tyaw: %f(%f)\n",
			this->map_pose.getOrigin().x(), this->map_pose.getOrigin().y(), this->map_yaw, this->map_yaw * 1800 / M_PI, Gyro_GetAngle(0), this->yaw, this->yaw * 1800 / M_PI);
}

void robot::visualize_marker_init(){
	this->clean_markers.ns = "waypoints";
	this->clean_markers.id = 0;
	this->clean_markers.type = visualization_msgs::Marker::LINE_STRIP;
	this->clean_markers.action= 0;//add
	this->clean_markers.lifetime=ros::Duration(0);
	this->clean_markers.scale.x = 0.31;
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
	this->clean_markers.points.push_back(m_points);

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
