#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pp/x900sensor.h>
#include <pp/slam_angle_offset.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>

#include <movement.h>
#include "crc8.h"
#include "serial.h"

#include "robotbase.h"

#define RECEI_LEN	41

static int TOPIC_PUB_RATE = 50;

bool is_robotbase_init = false;
bool robotbase_thread_stop = false;

pthread_t robotbaseThread_id;
pthread_t receiPortThread_id;

uint8_t receiStream[RECEI_LEN] = {0xAA, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
									0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
									0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
									0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xcc,
									0x33};

// Initialize the slam_angle_offset
float slam_angle_offset = 0;

int robotbase_init(void)
{
	int		ser_ret, base_ret;
	uint8_t	t_buf[2];

	ser_ret = base_ret = 0;
	robotbase_thread_stop = false;

	if (!is_serial_ready()) {
		printf("serial not ready\n");
		return -1;
	}

	set_main_pwr(0);
	set_gyro(1, 0);

	do {
		printf("waiting robot awake...\n");
		usleep(20000);
	} while ((serial_read(2, t_buf) <= 0) && ros::ok());

	ser_ret = pthread_create(&receiPortThread_id, NULL, serial_receive, NULL);
	base_ret = pthread_create(&robotbaseThread_id, NULL, base_run, NULL);

	if (base_ret != 0 || ser_ret != 0) {
		is_robotbase_init = false;
		robotbase_thread_stop = true;

		if (base_ret != 0) {
			fprintf(stderr,"fail to create robotbase thread!! %s \n", strerror(base_ret));
		} else if (ser_ret != 0) {
			fprintf(stderr,"fail to create serial receive thread!! %s \n", strerror(ser_ret));
		}
		return -1;
	} else {
		printf("threading running...\n");
		is_robotbase_init = true;
	}
	return 0;
}

bool is_robotbase_stop(void)
{
	return robotbase_thread_stop ? true : false;
}

void robotbase_deinit(void)
{
	uint8_t buf[2];

	if (is_robotbase_init) {
		printf("robotbase deinit...\n");
		is_robotbase_init = false;
		robotbase_thread_stop = true;

		do {
			control_stop_all();
			printf("shutdown robot power\n");
			usleep(300000);
		} while (serial_read(2, buf) > 0);

		serial_flush();
		printf("shutdown power OK\n");
		usleep(20000);
	}
}

void *serial_receive(void *)
{
	int		i, j, ret, wh_len, wht_len, whtc_len;

	uint8_t	r_crc, c_crc;
	uint8_t	h1 = 0xaa, h2 = 0x55, header[2], t1 = 0xcc, t2 = 0x33;
	uint8_t tmpSet[RECEI_LEN], receiData[RECEI_LEN];

	wh_len = RECEI_LEN - 2;
	wht_len = wh_len - 2;
	whtc_len = wht_len - 1;

	while (ros::ok() && (!robotbase_thread_stop)) {
		ret = serial_read(2, header);
		if (ret == 0)
			continue;

		if (header[0] == h1 && header[1] == h2) {
			ret = serial_read(wh_len, receiData);
			r_crc = receiData[whtc_len];

			tmpSet[0] = h1;
			tmpSet[1] = h2;
			for (i = 0; i < whtc_len; i++){
				tmpSet[i + 2] = receiData[i];
			}

			c_crc = calcBufCrc8((char *)tmpSet, wh_len - 1);
			if (r_crc == c_crc){
				if (receiData[wh_len - 1] == t2 && receiData[wh_len - 2] == t1) {
					for (j = 0; j < wht_len; j++) {
						receiStream[j + 2] = receiData[j];
					}
				} else {
					printf("tail incorret\n");
				}
			} else {
				printf("crc incorret\n");
			}
		}
	}
}

void *base_run(void*)
{
	printf("[robotbase.cpp] base_run is on.\n");
	float	th_last, vth, pose_x, pose_y;
	float	previous_angle = 999;
	float	delta_angle = 0;
	int16_t	angle;

	uint16_t	lw_speed, rw_speed;

	ros::Rate	r(TOPIC_PUB_RATE);
	ros::Time	cur_time, last_time;

	pp::x900sensor				sensor;
	nav_msgs::Odometry			odom;
	tf::TransformBroadcaster	odom_broad;
	geometry_msgs::Quaternion	odom_quat;

	geometry_msgs::TransformStamped	odom_trans;

	ros::Publisher	odom_pub,sensor_pub;
	ros::Subscriber	slam_angle_offset_sub;
	ros::NodeHandle	robotsensor_node;
	ros::NodeHandle	odom_node;
	ros::NodeHandle	slam_angle_offset_node;

	th_last = vth = pose_x = pose_y = 0.0;

	sensor_pub = robotsensor_node.advertise<pp::x900sensor>("/robot_sensor",1);
	odom_pub = odom_node.advertise<nav_msgs::Odometry>("/odom",5);
	slam_angle_offset_sub = slam_angle_offset_node.subscribe<pp::slam_angle_offset>("/slam_angle_offset", 1, slam_angle_offset_callback);

	cur_time = ros::Time::now();
	last_time  = cur_time;

	while (ros::ok() && !robotbase_thread_stop) {
		r.sleep();
		// Spin once to get the updated slam_angle_offset
		ros::spinOnce();

		lw_speed = (receiStream[2] << 8) | receiStream[3];
		rw_speed = (receiStream[4] << 8) | receiStream[5];
		sensor.lw_vel = (lw_speed > 0x7fff) ? -((float)(lw_speed - 0x8000) / 1000.0) : (float)(lw_speed) / 1000.0;
		sensor.rw_vel = (rw_speed > 0x7fff) ? -((float)(rw_speed - 0x8000) / 1000.0) : (float)(rw_speed) / 1000.0;

		angle = (receiStream[6] << 8) | receiStream[7];
		sensor.angle = -(float)(angle) / 100.0;
		// Compensate the angle with the offset published by slam
		sensor.angle -= slam_angle_offset;
		// Check for avoiding angle's sudden change
		if (previous_angle == 999){
			previous_angle = sensor.angle;
		}else{
			delta_angle = sensor.angle - previous_angle;
			// Format the delta_angle into Range(-180, 180)
			delta_angle = (180 < delta_angle ? delta_angle - 360 : delta_angle);
			delta_angle = (delta_angle < -180 ? delta_angle + 360 : delta_angle);
			// Decide whether it is a sudden change
			if (10 < fabs(delta_angle)){
				// It is a sudden change, discard this value
				sensor.angle = previous_angle;
			}
			// Save current angle as previous_angle
			previous_angle = sensor.angle;
		}
		//sensor.angle_v = -(float)((receiStream[8] << 8) | receiStream[9]) / 100.0;

		sensor.lw_crt = (((receiStream[10] << 8) | receiStream[11]) & 0x7fff) * 1.622;
		sensor.rw_crt = (((receiStream[12] << 8) | receiStream[13]) & 0x7fff) * 1.622;

		sensor.wall = ((receiStream[14] << 8)| receiStream[15]);
		sensor.l_obs = ((receiStream[16] << 8) | receiStream[17]);
		sensor.f_obs = ((receiStream[18] << 8) | receiStream[19]);
		sensor.r_obs = ((receiStream[20] << 8) | receiStream[21]);

		sensor.lbumper = (receiStream[22] & 0xf0) ? true : false;
		sensor.rbumper = (receiStream[22] & 0x0f) ? true : false;

		sensor.ir_ctrl = receiStream[23];
		sensor.c_stub = (receiStream[24] << 16) | (receiStream[25] << 8) | receiStream[26];
		sensor.key = receiStream[27];
		sensor.c_s = (receiStream[28] > 0) ? true : false;
		sensor.w_tank = (receiStream[29] > 0) ? true : false;
		sensor.batv = receiStream[30];

		sensor.lcliff = ((receiStream[31] << 8) | receiStream[32]);
		sensor.fcliff = ((receiStream[33] << 8) | receiStream[34]);
		sensor.rcliff = ((receiStream[35] << 8) | receiStream[36]);

		sensor.lbrush_oc = (receiStream[37] & 0x08) ? true : false;		// left brush over current
		sensor.mbrush_oc = (receiStream[37] & 0x04) ? true : false;		// main brush over current
		sensor.rbrush_oc = (receiStream[37] & 0x02) ? true : false;		// right brush over current
		sensor.vcum_oc = (receiStream[37] & 0x01) ? true : false;		// vaccum over current

		cur_time = ros::Time::now();
		float vx = (sensor.lw_vel + sensor.rw_vel) / 2.0;
		float th = (-(float)(angle) / 100.0) * 0.01745;					//turn degrees into radians
		float dt = (cur_time - last_time).toSec();

		last_time = cur_time;
		vth = (th - th_last) / dt;
		sensor.angle_v = vth * 57.3;
		//sensor.angle = th;
		//sensor.angle_v = vth;
		th_last = th;

		pose_x += (vx * cos(th) - 0 * sin(th)) * dt;
		pose_y += (vx * sin(th) + 0 * cos(th)) * dt;

		odom_quat = tf::createQuaternionMsgFromYaw(th);
		odom.header.stamp = cur_time;
		odom.header.frame_id = "/odom";
		odom.child_frame_id = "/base_link";
		odom.pose.pose.position.x = pose_x;
		odom.pose.pose.position.y = pose_y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.angular.z = vth;
		odom_trans.header.stamp = cur_time;
		odom_trans.header.frame_id = "/odom";
		odom_trans.child_frame_id = "/base_link";
		odom_trans.transform.translation.x = pose_x;
		odom_trans.transform.translation.y = pose_y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		odom_broad.sendTransform(odom_trans);

		odom_pub.publish(odom);
		sensor_pub.publish(sensor);
	}
}

void slam_angle_offset_callback(const pp::slam_angle_offset::ConstPtr& msg)
{
	// Update the angle offset given by slam
	if(ros::ok()&&(!robotbase_thread_stop)){
		slam_angle_offset = msg->slam_angle_offset;
	}
	printf("[robotbase.cpp] Get slam_angle_offset as: %f.\n", slam_angle_offset);
}
