#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include "gyro.h"
#include "robot.hpp"
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <movement.h>
#include <Angles.h>
#include "robotbase.h"
#include "config.h"
#include "laser.hpp"
#include "figures/segment.h"

#include "std_srvs/Empty.h"
using namespace obstacle_detector;
extern bool is_line_angle_offset;
extern bool enable_slam_offset;
static	robot *robot_obj = NULL;
//typedef double Angle;

extern pp::x900sensor   sensor;

Segment_set segmentss;

time_t	start_time;

//extern pp::x900sensor sensor;
robot::robot():is_align_active_(false),line_align_(finish),slam_type_(0),is_map_ready(false)
{
	this->init();
	this->robot_sensor_sub = this->robot_node_handler.subscribe("/robot_sensor", 10, &robot::robot_robot_sensor_cb, this);
	this->robot_tf = new tf::TransformListener(this->robot_node_handler, ros::Duration(10), true);
	/*map subscriber for exploration*/
//	this->map_metadata_sub = this->robot_node_handler.subscribe("/map_metadata", 1, &robot::robot_map_metadata_cb, this);

	this->is_moving = false;
	this->is_sensor_ready = false;
	this->is_odom_ready = false;

	this->bumper_left = 0;
	this->bumper_right = 0;

	this->linear_x = 0.0;
	this->linear_y = 0.0;
	this->linear_z = 0.0;

	this->line_angle = 0;

//	this->map_sub = this->robot_node_handler.subscribe("/map", 1, &robot::robot_map_cb, this);
	this->odom_sub = this->robot_node_handler.subscribe("/odom", 1, &robot::robot_odom_cb, this);

	start_mator_cli_ = robot_node_handler.serviceClient<std_srvs::Empty>("start_motor");
	stop_mator_cli_ = robot_node_handler.serviceClient<std_srvs::Empty>("stop_motor");
	printf("%s %d: robot init done!\n", __FUNCTION__, __LINE__);
	start_time = time(NULL);
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

	if(msg->ir_ctrl>0)
		this->ir_ctrl = msg->ir_ctrl;

	this->charge_stub = msg->c_stub;//charge stub signal
	Rcon_Status |= this->charge_stub;
	//printf("[robot.cpp] Rcon info:%x.\n", this->charge_stub);

	this->key = msg->key;

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


	this->is_sensor_ready = true;
//	if (this->is_sensor_ready == false) {
//		if (time(NULL) - start_time > 2) {
//			printf("%s %d: Gyro starting angle: %d\n", __FUNCTION__, __LINE__, (int16_t)((this->angle * 10 + 3600)) % 3600);

//			Gyro_SetImuOffset(((int16_t)(this->angle * 10 + 3600)) % 3600);
//			Gyro_SetImuAngle(((int16_t)(this->angle * 10 + 3600)) % 3600, this->angle_v);
//			this->is_sensor_ready = true;
//		}
//	} else {
//		Gyro_SetImuAngle(((int16_t)(this->angle * 10 + 3600)) % 3600, this->angle_v);
//	}

#if 0
	printf("%s %d:\n\t\tangle: %f\tangle_v: %f\n", __FUNCTION__, __LINE__, angle, angle_v);
	printf("\t\tvaccum: %d\tbox: %d\tbattery_voltage: %d, brush left: %d\t brush right: %d\tbrush main: %d\n", vaccum, box, battery_voltage, brush_left, brush_right, brush_main);
	printf("\t\tbumper_right: %d\tbumper_left: %d\tobs_left: %d\tobs_right: %d\tobs_front: %d\n", bumper_right, bumper_left, obs_left, obs_right , obs_front);
	printf("\t\tcliff right: %d\tcliff left: %d\t cliff front: %d\t wall: %d\n", cliff_right, cliff_left, cliff_front, wall);
	printf("\t\trcon left: %d\trcon right: %d\trcon fl: %d\trcon fr: %d\trcon bl: %d\trcon br: %d\n", rcon_left, rcon_right, rcon_front_left, rcon_front_right, rcon_back_left, rcon_back_right);
#endif
}
/*

void robot::robot_map_metadata_cb(const nav_msgs::MapMetaData::ConstPtr& msg)
{
	static int count = 0;

	count++;
	if (count > 1) {
		this->is_map_ready = true;
	}
}
*/

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
	if (enable_slam_offset == 1){
		//ROS_INFO("SLAM = 1");
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
	}else{
		//ROS_INFO("SLAM = 0");
		try {
			this->robot_tf->lookupTransform("/odom", "base_link", ros::Time(0), transform);
			this->yaw = tf::getYaw(transform.getRotation());

			Gyro_SetAngle(((int16_t)(this->yaw * 1800 / M_PI + 3600)) % 3600, this->angle_v);
			//printf("%s %d: offset: %d\n", __FUNCTION__, __LINE__, ((int16_t)(this->yaw * 1800 / M_PI + 3600)) % 3600 - Gyro_GetAngle(0));
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
	}

  if (!this->is_odom_ready) {
    this->position_x_off = map_pose.getOrigin().x();
    this->position_y_off = map_pose.getOrigin().y();
    this->position_z_off = map_pose.getOrigin().z();
  }

  this->position_y = map_pose.getOrigin().y() - position_y_off;
  this->position_x = map_pose.getOrigin().x() - position_x_off;

  if (this->is_odom_ready == false) {
    this->is_odom_ready = true;
  }
}
void robot::robot_map_cb(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
	int map_size, vector_size;
	//uint32_t seq;
	//uint32_t width;
	//uint32_t height;
	//float resolution;
	//std::vector<int8_t> *ptr;
	this->width = map->info.width;
	this->height = map->info.height;
	this->resolution = map->info.resolution;
	this->seq = map->header.seq;
	this->origin_x = map->info.origin.position.x;
	this->origin_y = map->info.origin.position.y;
	//map_size = (width * height);
	//int8_t map_data[n];
	//std::vector<int8_t> map_data(map->data);
	this->map_data = map->data;
	//this->map_data(map->data);
	this->ptr = &(map_data);
	//vector_size = v1.size();

	//v1.swap(map->data);
	//memcpy(map_data, &map->data, sizeof(map->data));
	//printf("width=%dheight=%dresolution=%fseq=%d\n",width,height,resolution,seq);
	//printf("map_size=%d\nvector_size=%d", map_size, vector_size);
	//printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",map_data[0],map_data[1],map_data[2],map_data[3],map_data[4],map_data[5],map_data[6],map_data[7],map_data[8],map_data[9],map_data[10],map_data[11]);
	//printf("map->data=%d,%d,%d,%d,%d,%d\n",(map->data)[0],(map->data)[1],(map->data)[2],(map->data)[3],(map->data)[4],(map->data)[5]);
	printf("vector=%d,%d,%d\n",map_data[0],map_data[1],map_data[2]);
	printf("vector_pointer=%d\n",(*(this->ptr))[0]);
	printf("finished map callback\n");
	is_map_ready=true;

}

uint32_t robot::robot_get_width()
{
	return this->width;
}

uint32_t robot::robot_get_height()
{
	return this->height;
}

float robot::robot_get_resolution()
{
	return this->resolution;
}

double robot::robot_get_origin_x()
{
	return this->origin_x;
}

double robot::robot_get_origin_y()
{
	return this->origin_y;
}

std::vector<int8_t> *robot::robot_get_map_data()
{
	//printf("return the ptr address\n");
	//printf("vector_pointer_address=%d\n",(*(this->ptr))[0]);
	//printf("seq=%d\n",this->seq);
	return this->ptr;
}

void robot::robot_obstacles_cb(const obstacle_detector::Obstacles::ConstPtr &msg)
{
	if (laser::instance()->is_ready() == false || is_sensor_ready == false)
		return;

	if (line_align_ == detecting)
	{
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
	return sensor.left_wall;
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
void robot::robot_set_ir_cmd(uint8_t cmd)
{
	if(cmd>0xff)
		return;
	this->ir_ctrl = cmd;
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
//	//printf("[robot.cpp] charge_stub:%x.\n", (this->charge_stub & 0xf00000) >> 20);
//	return this -> rcon_front_left = (this->charge_stub & 0xf00000) >> 20;
//}
//
//uint32_t robot::robot_get_rcon_front_right()
//{
//	// Move the 4 bits info to lowest bits
//	//printf("[robot.cpp] charge_stub:%x.\n", (this->charge_stub & 0x0f0000) >> 16);
//	return this->rcon_front_right = (this->charge_stub & 0x0f0000) >> 16;
//}
//
//uint32_t robot::robot_get_rcon_back_left()
//{
//	// Move the 4 bits info to lowest bits
//	//printf("[robot.cpp] charge_stub:%x.\n", (this->charge_stub & 0x0000f0) >> 4);
//	return this->rcon_back_left = (this->charge_stub & 0x0000f0) >> 4;
//}
//
//uint32_t robot::robot_get_rcon_back_right()
//{
//	// Move the 4 bits info to lowest bits
//	//printf("[robot.cpp] charge_stub:%x.\n", (this-> charge_stub & 0x00000f) >> 0);
//	return this->rcon_back_right = (this-> charge_stub & 0x00000f) >> 0;
//}
//
//uint32_t robot::robot_get_rcon_left()
//{
//	// Move the 4 bits info to lowest bits
//	//printf("[robot.cpp] charge_stub:%x.\n", (this-> charge_stub & 0x00f000) >> 12);
//	return this->rcon_left = (this-> charge_stub & 0x00f000) >> 12;
//}
//
//uint32_t robot::robot_get_rcon_right()
//{
//	// Move the 4 bits info to lowest bits
//	//printf("[robot.cpp] charge_stub:%x.\n", (this->charge_stub & 0x000f00) >> 8);
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

int16_t robot::robot_get_home_angle() {
  return ((int16_t) (this->line_angle));
}

void robot::robot_display_positions() {
  printf("base_link->map: (%f, %f) %f(%f) Gyro: %d\tyaw: %f(%f)\n",
         this->map_pose.getOrigin().x(), this->map_pose.getOrigin().y(), this->map_yaw, this->map_yaw * 1800 / M_PI,
         Gyro_GetAngle(0), this->yaw, this->yaw * 1800 / M_PI);
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
	this->clean_markers.points.clear();
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

void robot::start_slam(void)
{
	is_map_ready = false;
	if (slam_type_ == 0)
		system("roslaunch pp gmapping.launch 2>/dev/null &");
	else if (slam_type_ == 1)
		system("roslaunch slam_karto karto_slam_w_params.launch 2>/dev/null &");
	else if (slam_type_ == 2)
		system("roslaunch pp cartographer_slam.launch 2>/dev/null &");
}

//void start_obstacle_detector(void)
//{
//	system("roslaunch obstacle_detector single_scanner.launch 2>/dev/null&");
//}

/*
void robot::stop_obstacle_detector(void)
{
	system("rosnode kill /obstacle_detector");
}
*/

void robot::stop_slam(void)
{
	is_map_ready = false;
	if (slam_type_ == 0)
		system("rosnode kill /slam_gmapping 2>/dev/null &");
	else if (slam_type_ == 1)
		system("rosnode kill /slam_karto 2>/dev/null &");
	else if (slam_type_ == 2)
		system("rosnode kill /cartographer_node 2>/dev/null &");
}

void robot::align(void)
{

	if (is_align_active_ != true)
		return;

	line_align_ = detecting;
	is_odom_ready = false;
	segmentss.clear();
	obstacles_sub = robot_node_handler.subscribe("/obstacles", 1, &robot::robot_obstacles_cb, this);
	auto count_n_10ms = 500;
	while (line_align_ == detecting && --count_n_10ms > 0)
	{
		if (count_n_10ms % 100 == 0)
			ROS_WARN("detecting time remain %d s\n", count_n_10ms / 100);
		usleep(10000);
	}

	auto seg = segmentss.min_distant_segment();
	line_angle = static_cast<int16_t>((seg.last_point - seg.first_point).angleDeg() * 10);
	auto angle = static_cast<int16_t>(std::abs(line_angle));
	ROS_INFO("line detect: rotating line_angle(%d)", line_angle);

	if (line_angle > 0)
	{
		ROS_INFO("Turn_Left %d", angle);
		Turn_Left(3, angle);
	} else if (line_angle < 0)
	{
		ROS_INFO("Turn_Right %d", angle);
		Turn_Right(3, angle);
	}
	line_align_ = finish;
//	ros::WallDuration(100).sleep();
	auto count = 2;
	while (count-- != 0)
	{
		std::cout << robot::angle << std::endl;
		sleep(1);
	}
	is_line_angle_offset = true;
}

void robot::align_exit(void){ is_align_active_ =  true;
	line_align_ = detecting;
}

void robot::align_active(bool active){
	is_align_active_ =  active;
	if(is_align_active_ == true){
		line_align_ = detecting;
	}
}

void robot::start_lidar(void)
{
	std_srvs::Empty empty;
	auto count_3s = 0;
	do
	{

		ROS_INFO("start_mator");
		start_mator_cli_.call(empty);
		count_3s = 300;
		laser::instance()->is_ready(false);
		while (laser::instance()->is_ready() == false && --count_3s > 0)
		{
			if (count_3s % 100 == 0)
				ROS_INFO("start_lidar time remain %d", count_3s / 100);
			usleep(10000);
		}
	}while (count_3s == 0);

//	ROS_INFO("start_mator success!!!");
}

void robot::stop_lidar(void){
	std_srvs::Empty empty;
//	is_odom_ready = false;
	do
	{
		laser::instance()->is_ready(false);
		ROS_INFO("stop_lidar");
		stop_mator_cli_.call(empty);
		sleep(2);
	}while (laser::instance()->is_ready() == true);

}

void robot::slam_type(int type)
{
	slam_type_ = type;
}

void robot::map_ready(bool val)
{
	is_map_ready = val;
}

bool robot::map_ready(void)
{
	return is_map_ready;
}

void robot::Subscriber(void)
{
	this->map_sub = this->robot_node_handler.subscribe("/map", 1, &robot::robot_map_cb, this);
	//this->odom_sub = this->robot_node_handler.subscribe("/odom", 1, &robot::robot_odom_cb, this);

	visualize_marker_init();
	this->send_clean_marker_pub = this->robot_node_handler.advertise<visualization_msgs::Marker>("clean_markers",1);
	this->send_bumper_marker_pub = this->robot_node_handler.advertise<visualization_msgs::Marker>("bumper_markers",1);
	if(is_align_active_)
	  obstacles_sub = robot_node_handler.subscribe("/obstacles", 1, &robot::robot_obstacles_cb, this);

}

void robot::UnSubscriber(void)
{
	map_sub.shutdown();
	//odom_sub.shutdown();

	if(is_align_active_)
	  obstacles_sub.shutdown();
}
void robot::init_mumber()
{
	is_odom_ready = false;
	position_x=0;
	position_y=0;
	position_z=0;
	odom_pose_x=0;
	odom_pose_y=0;
	position_map_x=0;
	position_map_y=0;
	position_x_off=0;
	position_y_off=0;
	position_z_off=0;
}
