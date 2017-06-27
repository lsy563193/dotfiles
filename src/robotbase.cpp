#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pp/x900sensor.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>

#include "movement.h"
#include "gyro.h"
#include "crc8.h"
#include "serial.h"
#include "robotbase.h"
#include "config.h"
#include "robot.hpp"
#include "wav.h"

#define ROBOTBASE "robotbase"
#define _H_LEN 2
#if __ROBOT_X400
uint8_t receiStream[RECEI_LEN]={				0xaa,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xcc,0x33};
uint8_t g_send_stream[SEND_LEN]={0xaa,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0xcc,0x33};

#elif __ROBOT_X900
uint8_t receiStream[RECEI_LEN]={				0xaa,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,0x00,0xcc,0x33};
uint8_t g_send_stream[SEND_LEN]={0xaa,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x64,0x00,0x02,0x00,0x00,0xcc,0x33};
#endif



#define  _RATE 50 

bool is_robotbase_init = false;
bool robotbase_thread_stop = false;
bool send_stream_thread = false;
pthread_t robotbaseThread_id;
pthread_t receiPortThread_id;
pthread_t sendPortThread_id;
pthread_mutex_t recev_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  recev_cond = PTHREAD_COND_INITIALIZER;

pthread_mutex_t serial_data_ready_mtx = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t serial_data_ready_cond = PTHREAD_COND_INITIALIZER;

pp::x900sensor	sensor;

// This flag is for reset beep action
bool robotbase_beep_update_flag = false;
// Speaker totally sound time count, every count means once of send streem loop, if this count < 0, it will be a constant beep action
int robotbase_speaker_sound_loop_count = 0;
// Sound code to be set in g_send_stream
uint8_t robotbase_sound_code = 0;
// A speaker sound loop contains one sound time and one silence time
// Speaker sound time count in one speaker sound loop, every count means once of send streem loop
int robotbase_speaker_sound_time_count = 0;
int temp_speaker_sound_time_count = -1;
// Speaker silence time count in one speaker sound loop, every count means once of send streem loop
int robotbase_speaker_silence_time_count = 0;
int temp_speaker_silence_time_count = 0;

// Lock for odom coordinate
boost::mutex odom_mutex;

// Odom coordinate
float pose_x, pose_y;

// For obs dynamic adjustment
int OBS_adjust_count;

int robotbase_init(void)
{
	int		ser_ret, base_ret,sers_ret;
	uint8_t	t_buf[2];

	ser_ret = base_ret = 0;
	robotbase_thread_stop = false;
	send_stream_thread = true;
	
	if (!is_serial_ready()) {
		ROS_INFO("[robotbase] serial not ready\n");
		return -1;
	}
	set_main_pwr_byte(POWER_ACTIVE);
	g_send_stream[SEND_LEN-3] = calcBufCrc8((char *)g_send_stream, SEND_LEN-3);
	ROS_INFO("[robotbase] waiting robotbase awake ");
//	do {
//		serial_write(SEND_LEN,g_send_stream);
//		usleep(20000);
//	} while ((serial_read(2, t_buf) <= 0) && ros::ok());
	//ROS_INFO("OK!");
	ser_ret = pthread_create(&receiPortThread_id, NULL, serial_receive_routine, NULL);
	base_ret = pthread_create(&robotbaseThread_id, NULL, robotbase_routine, NULL);
	sers_ret = pthread_create(&sendPortThread_id,NULL,serial_send_routine,NULL);
	if (base_ret != 0 || ser_ret != 0 || sers_ret !=0) {
		is_robotbase_init = false;
		robotbase_thread_stop = true;
		send_stream_thread = false;
		if (base_ret < 0) {ROS_INFO("[robotbase] fail to create robotbase thread!! %s ", strerror(base_ret));}
		if (ser_ret < 0) {ROS_INFO("[robotbase] fail to create serial receive thread!! %s ", strerror(ser_ret));}
		if (sers_ret < 0){ROS_INFO("[robotbase] fail to create serial send therad!! %s ",strerror(sers_ret));}
		return -1;
	}
	is_robotbase_init = true;
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
		ROS_INFO("%s,%d",__FUNCTION__,__LINE__);
		is_robotbase_init = false;
		robotbase_thread_stop = true;
		ROS_INFO("\tshutdown robotbase power");
		set_led(0, 0);
		control_set(CTL_BUZZER, 0x00);
		Set_Gyro_Off();
		usleep(40000);
		disable_motors();
		usleep(40000);
		set_main_pwr_byte(POWER_DOWN);
		usleep(40000);	
		send_stream_thread = false;
		usleep(40000);
		serial_close();
		ROS_INFO("%s,%d, Stop OK",__FUNCTION__,__LINE__);
		int mutex_ret = pthread_mutex_destroy(&recev_lock);
		if(mutex_ret<0)
			ROS_ERROR("_%s,%d, pthread mutex destroy fail",__FUNCTION__,__LINE__);
		int cond_ret = pthread_cond_destroy(&recev_cond);
		if(cond_ret<0)
			ROS_ERROR("%s,%d,pthread cond destroy fail",__FUNCTION__,__LINE__);
	}
}

void robotbase_reset_send_stream(void)
{
	for (int i = 0; i < SEND_LEN; i++) {
		if (i != CTL_LED_GREEN)
			g_send_stream[i] = 0x0;
		else
			g_send_stream[i] = 0x64;
	}
	g_send_stream[0] = 0xaa;
	g_send_stream[1] = 0x55;
	g_send_stream[SEND_LEN - 2] = 0xcc;
	g_send_stream[SEND_LEN - 1] = 0x33;
}

void *serial_receive_routine(void *)
{
	pthread_detach(pthread_self());
	ROS_INFO("%s,%d thread running",__FUNCTION__,__LINE__);
	int i, j, ret, wh_len, wht_len, whtc_len;

	uint8_t r_crc, c_crc;
	uint8_t h1 = 0xaa, h2 = 0x55, header[2], t1 = 0xcc, t2 = 0x33;
	uint8_t tmpSet[RECEI_LEN], receiData[RECEI_LEN];

	wh_len = RECEI_LEN - 2; //length without header bytes
	wht_len = wh_len - 2; //length without header and tail bytes
	whtc_len = wht_len - 1; //length without header and tail and crc bytes

	while (ros::ok() && (!robotbase_thread_stop)) {
		ret = serial_read(1, &header[0]);
		if (ret != 1 ){
			ROS_WARN("%s, %d, serial read %d bytes,  requst %d byte",__FUNCTION__,__LINE__,ret,1);
			continue;
		}
		if(header[0] != h1)
			continue;
		ret= serial_read(1,&header[1]);
		if (ret != 1 ){
			ROS_WARN("%s,%d,serial read %d bytes, requst %d byte",__FUNCTION__,__LINE__,ret,1);
			continue;
		}
		if(header[1] != h2){
			continue;
		}
		ret = serial_read(wh_len, receiData);
		if(ret != wh_len){
			ROS_WARN("%s,%d,serial read %d bytes, requst %d bytes",__FUNCTION__,__LINE__,ret,wh_len);
			continue;
		}
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
				if(pthread_cond_signal(&recev_cond)<0)
					ROS_ERROR(" in serial read, pthread signal fail !");//if receive data corret than send signal
			} else {
				ROS_WARN(" in serial read ,data tail error\n");
			}
		} else {
			ROS_WARN( " in serial read ,data crc error\n");
		}
	}
	ROS_INFO("pthread serial read exit!");
}

void *robotbase_routine(void*)
{
	pthread_detach(pthread_self());
	ROS_INFO("%s.%d, thread running",__FUNCTION__,__LINE__);
	float	th_last, vth;

	uint16_t	lw_speed, rw_speed;

	ros::Rate	r(_RATE);
	ros::Time	cur_time, last_time;

	//Debug
	uint16_t error_count = 0;

	nav_msgs::Odometry			odom;
	tf::TransformBroadcaster	odom_broad;
	geometry_msgs::Quaternion	odom_quat;

	geometry_msgs::TransformStamped	odom_trans;

	ros::Publisher	odom_pub,sensor_pub;
	ros::NodeHandle	robotsensor_node;
	ros::NodeHandle	odom_node;
	ros::NodeHandle	slam_angle_offset_node;

	th_last = vth = pose_x = pose_y = 0.0;

	sensor_pub = robotsensor_node.advertise<pp::x900sensor>("/robot_sensor",1);
	odom_pub = odom_node.advertise<nav_msgs::Odometry>("/odom",5);

	cur_time = ros::Time::now();
	last_time  = cur_time;

	while (ros::ok() && !robotbase_thread_stop) {
		
		if(pthread_mutex_lock(&recev_lock)!=0)ROS_ERROR("robotbase pthread receive lock fail");
		if(pthread_cond_wait(&recev_cond,&recev_lock)!=0)ROS_ERROR("robotbase pthread receive cond wait fail");	
		//ros::spinOnce();

		boost::mutex::scoped_lock(odom_mutex);
		lw_speed = (receiStream[2] << 8) | receiStream[3];
		rw_speed = (receiStream[4] << 8) | receiStream[5];
		sensor.lw_vel = (lw_speed > 0x7fff) ? -((float)(lw_speed - 0x8000) / 1000.0) : (float)(lw_speed) / 1000.0;
		sensor.rw_vel = (rw_speed > 0x7fff) ? -((float)(rw_speed - 0x8000) / 1000.0) : (float)(rw_speed) / 1000.0;
		sensor.angle = -(float)(int16_t)((receiStream[6] << 8) | receiStream[7]) / 100;

		sensor.angle -= robot::instance()->offsetAngle();
		//ROS_INFO("sensor:%f",robot::instance()->getAngle());

		sensor.angle_v = -(float)((receiStream[8] << 8) | receiStream[9]) / 100.0;
		sensor.lw_crt = (((receiStream[10] << 8) | receiStream[11]) & 0x7fff) * 1.622;
		sensor.rw_crt = (((receiStream[12] << 8) | receiStream[13]) & 0x7fff) * 1.622;
		sensor.left_wall = ((receiStream[14] << 8)| receiStream[15]);
		sensor.l_obs = ((receiStream[16] << 8) | receiStream[17]);
		sensor.f_obs = ((receiStream[18] << 8) | receiStream[19]);
		sensor.r_obs = ((receiStream[20] << 8) | receiStream[21]);
#if __ROBOT_X900
		sensor.right_wall = ((receiStream[22]<<8)|receiStream[23]);		
		sensor.lbumper = (receiStream[24] & 0xf0) ? true : false;
		sensor.rbumper = (receiStream[24] & 0x0f) ? true : false;
		sensor.ir_ctrl = receiStream[25];
		sensor.c_stub = (receiStream[26] << 24 ) | (receiStream[27] << 16) | (receiStream[28] << 8) | receiStream[29];
		sensor.visual_wall = (receiStream[30] << 8)| receiStream[31];
		sensor.key = receiStream[32];
		sensor.c_s = receiStream[33];
		sensor.w_tank = (receiStream[34]>0)?true:false;
		sensor.batv = (receiStream[35]);
		sensor.lcliff = ((receiStream[36] << 8) | receiStream[37]);
		sensor.fcliff = ((receiStream[38] << 8) | receiStream[39]);
		sensor.rcliff = ((receiStream[40] << 8) | receiStream[41]);
		sensor.vacuum_selfcheck_status = (receiStream[42] & 0x30);
		sensor.lbrush_oc = (receiStream[42] & 0x08) ? true : false;		// left brush over current
		sensor.mbrush_oc = (receiStream[42] & 0x04) ? true : false;		// main brush over current
		sensor.rbrush_oc = (receiStream[42] & 0x02) ? true : false;		// right brush over current
		sensor.vcum_oc = (receiStream[42] & 0x01) ? true : false;		// vaccum over current
		sensor.gyro_dymc = receiStream[43];
		sensor.omni_wheel = (receiStream[44]<<8)|receiStream[45];
		sensor.x_acc = ((receiStream[46]<<8)|receiStream[47])/258.0f; //in mG
		sensor.y_acc = ((receiStream[48]<<8)|receiStream[49])/258.0f; //in mG
		sensor.z_acc = ((receiStream[50]<<8)|receiStream[51])/258.0f; //in mG
		sensor.plan = receiStream[52];
#elif __ROBOT_X400
		sensor.lbumper = (receiStream[22] & 0xf0)?true:false;
		sensor.rbumper = (receiStream[22] & 0x0f)?true:false;
		sensor.ir_ctrl_ = receiStream[23];
		sensor.c_stub = (receiStream[24] << 16) | (receiStream[25] << 8) | receiStream[26];
		sensor.key = receiStream[27];
		sensor.c_s = receiStream[28];
		sensor.w_tank_ = (receiStream[29] > 0) ? true : false;
		sensor.batv = receiStream[30];

		sensor.lcliff = ((receiStream[31] << 8) | receiStream[32]);
		sensor.fcliff = ((receiStream[33] << 8) | receiStream[34]);
		sensor.rcliff = ((receiStream[35] << 8) | receiStream[36]);

		sensor.lbrush_oc_ = (receiStream[37] & 0x08) ? true : false;		// left brush over current
		sensor.mbrush_oc_ = (receiStream[37] & 0x04) ? true : false;		// main brush over current
		sensor.rbrush_oc_ = (receiStream[37] & 0x02) ? true : false;		// right brush over current
		sensor.vcum_oc = (receiStream[37] & 0x01) ? true : false;		// vaccum over current
		sensor.gyro_dymc_ = receiStream[38];
		sensor.right_wall_ = ((receiStream[39]<<8)|receiStream[40]);
		sensor.x_acc_ = ((receiStream[41]<<8)|receiStream[42])/258.0f; //in mG
		sensor.y_acc_ = ((receiStream[43]<<8)|receiStream[44])/258.0f; //in mG
		sensor.z_acc_ = ((receiStream[45]<<8)|receiStream[46])/258.0f; //in mG
#endif	

		pthread_mutex_lock(&serial_data_ready_mtx);
		pthread_cond_broadcast(&serial_data_ready_cond);
		pthread_mutex_unlock(&serial_data_ready_mtx);

		/*------------publish odom and robot_sensor topic -----------------------*/
		cur_time = ros::Time::now();
		float vx = (sensor.lw_vel + sensor.rw_vel) / 2.0;
		float th = sensor.angle * 0.01745;					//turn degrees into radians
		float dt = (cur_time - last_time).toSec();
		last_time = cur_time;
		pose_x += (vx * cos(th) - 0 * sin(th)) * dt;
		pose_y += (vx * sin(th) + 0 * cos(th)) * dt;
		odom_quat = tf::createQuaternionMsgFromYaw(th);
		odom.header.stamp = cur_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "base_link";
		odom.pose.pose.position.x = pose_x;
		odom.pose.pose.position.y = pose_y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.angular.z = vth;
		odom_trans.header.stamp = cur_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		odom_trans.transform.translation.x = pose_x;
		odom_trans.transform.translation.y = pose_y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		odom_broad.sendTransform(odom_trans);

		odom_pub.publish(odom);

/*
		if(get_angle_offset() == 2){
			set_angle_offset(3);
			g_cond_var.notify_all();
		}
*/

		sensor_pub.publish(sensor);
		/*---------------publish end --------------------------*/

		if(pthread_mutex_unlock(&recev_lock)!=0)ROS_WARN("robotbase pthread receive unlock fail");
		// Dynamic adjust obs
		obs_dynamic_base(OBS_adjust_count);
	}
	ROS_INFO("robotbase thread exit");
	//pthread_exit(NULL);
}

void *serial_send_routine(void*){
	pthread_detach(pthread_self());
	ROS_INFO("%s,%d thread running",__FUNCTION__,__LINE__);
	ros::Rate r(_RATE);
	uint8_t buf[SEND_LEN];
	int sl = SEND_LEN-3;
	reset_send_flag();
	while(send_stream_thread){
		r.sleep();
		if(get_sleep_mode_flag()){
			continue;
		}
		/*-------------------speaker variable counter -----------------------*/
		// Force reset the beep action when beep() function is called, especially when last beep action is not over. It can stop last beep action and directly start the updated beep action.
		if (robotbase_beep_update_flag){
			temp_speaker_sound_time_count = -1;
			temp_speaker_silence_time_count = 0;
			robotbase_beep_update_flag = false;
		}
		//ROS_INFO("%s %d: tmp_sound_count: %d, tmp_silence_count: %d, sound_loop_count: %d.", __FUNCTION__, __LINE__, temp_speaker_sound_time_count, temp_speaker_silence_time_count, robotbase_speaker_sound_loop_count);
		// If beep_time_count has ran out, it will not sound anymore and check the battary status. If low battary, it will constantly beep to alarm.
		// If count > 0, it is processing for different alarm, if count < 0, it should be processing low battary alarm.
		if (robotbase_speaker_sound_loop_count != 0){
			process_beep();
		}
		/*-------------------counter end-------------------------------------*/

		if(!is_send_busy()){
			memcpy(buf,g_send_stream,sizeof(uint8_t)*SEND_LEN);
			buf[CTL_CRC] = calcBufCrc8((char *)buf, sl);
			serial_write(SEND_LEN, buf);
		}
	}
	ROS_INFO("serial send pthread exit");
	//pthread_exit(NULL);
}

/*---------process_beep()---------------*/
/*--------author: austin---------------*/

void process_beep(){
	// This routine handles the speaker sounding logic
	// If temp_speaker_silence_time_count == 0, it is the end of loop of silence, so decrease the count and set sound in g_send_stream.
	if (temp_speaker_silence_time_count == 0){
		temp_speaker_silence_time_count--;
		temp_speaker_sound_time_count = robotbase_speaker_sound_time_count;
		control_set(CTL_BUZZER, robotbase_sound_code & 0xFF);
	}
	// If temp_speaker_sound_time_count == 0, it is the end of loop of sound, so decrease the count and set sound in g_send_stream.
	if (temp_speaker_sound_time_count == 0){
		temp_speaker_sound_time_count--;
		temp_speaker_silence_time_count = robotbase_speaker_silence_time_count;
		control_set(CTL_BUZZER, 0x00);
		// Decreace the speaker sound loop count because when it turns to silence this sound loop will be over when silence end, so we can decreace the sound loop count here.
		// If it is for constant beep, the loop count will be less than 0, it will not decrease either.
		if (robotbase_speaker_sound_loop_count > 0){
			robotbase_speaker_sound_loop_count--;
		}
	}
	// If temp_speaker_silence_time_count == -1, it is in loop of sound, so decrease the count.
	if (temp_speaker_silence_time_count == -1){
		temp_speaker_sound_time_count--;
	}
	// If temp_speaker_sound_time_count == -1, it is in loop of silence, so decrease the count.
	if (temp_speaker_sound_time_count == -1){
		temp_speaker_silence_time_count--;
	}
}

void robotbase_reset_odom_pose(void)
{
	// Reset the odom pose to (0, 0)
	boost::mutex::scoped_lock(pose_mutex);
	pose_x = pose_y = 0;
}

void robotbase_restore_slam_correction()
{
	// For restarting slam
	boost::mutex::scoped_lock(odom_mutex);
	pose_x += robot::instance()->getCorrectionX();
	pose_y += robot::instance()->getCorrectionY();
	robot::instance()->offsetAngle(robot::instance()->offsetAngle() + robot::instance()->getCorrectionYaw());
}

void robotbase_obs_adjust_count(int count)
{
#ifdef OBS_DYNAMIC
	boost::mutex::scoped_lock(odom_mutex);
	OBS_adjust_count = count;
#endif
}
bool is_turn(void)
{
	boost::mutex::scoped_lock(odom_mutex);
	return ((abs(sensor.rw_vel - sensor.rw_vel) > 0.1) ||
					(sensor.lw_vel * sensor.rw_vel < 0)
					);
}
