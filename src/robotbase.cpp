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

#include "movement.h"
#include "crc8.h"
#include "serial.h"
#include "robotbase.h"
#include "config.h"

#define ROBOTBASE "robotbase"

#if ROBOT_X400
uint8_t receiStream[50]={				0xaa,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xcc,0x33};
#elif ROBOT_X600
uint8_t receiStream[60]={				0xaa,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
										0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xcc,0x33};
#endif

uint8_t sendStream[19]={0xaa,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0xcc,0x33};

#define  _RATE 50 

bool is_robotbase_init = false;
bool robotbase_thread_stop = false;
bool send_stream_thread = false;
pthread_t robotbaseThread_id;
pthread_t receiPortThread_id;
pthread_t sendPortThread_id;
pthread_mutex_t send_lock;
pp::x900sensor	sensor;
// Initialize the slam_angle_offset
float slam_angle_offset = 0;
//When you restart gmapping, gyro may be have a angle offset, compensate it
bool is_line_angle_offset = false;
float line_angle_offset = 0;

// This flag is for reset beep action
bool robotbase_beep_update_flag = false;
// Speaker totally sound time count, every count means once of send streem loop, if this count < 0, it will be a constant beep action
int robotbase_speaker_sound_loop_count = 0;
// Sound code to be set in sendStream
uint8_t robotbase_sound_code = 0;
// A speaker sound loop contains one sound time and one silence time
// Speaker sound time count in one speaker sound loop, every count means once of send streem loop
int robotbase_speaker_sound_time_count = 0;
int temp_speaker_sound_time_count = -1;
// Speaker silence time count in one speaker sound loop, every count means once of send streem loop
int robotbase_speaker_silence_time_count = 0;
int temp_speaker_silence_time_count = 0;

// Low battery flag
extern uint8_t lowBattery;
extern bool enable_slam_offset;
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
	set_main_pwr(0);
	set_gyro(0, 0);
	Set_LED(100,0);
	sendStream[SEND_LEN-3] = calcBufCrc8((char *)sendStream, SEND_LEN-3);
	ROS_INFO("[robotbase] waiting robotbase awake ");
	do {
		serial_write(SEND_LEN,sendStream);
		usleep(20000);
	} while ((serial_read(2, t_buf) <= 0) && ros::ok());
	printf("OK!\n");
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
	int mutex_ret = pthread_mutex_init(&send_lock,NULL);
	if(mutex_ret <0)
		ROS_INFO("mutex lock crate fail\n");
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
		printf("[robotbase]deinit...\n");
		is_robotbase_init = false;
		robotbase_thread_stop = true;
		printf("\tshutdown robotbase power \n");
		Set_LED(0,0);
		control_set(CTL_BUZZER, 0x00);
		set_gyro(0,0);
		usleep(40000);
		Disable_Motors();
		usleep(40000);
		set_main_pwr(1);
		usleep(40000);	
		send_stream_thread = false;
		usleep(40000);
		serial_close();
		printf("[robotbase.cpp] Stop ok\n");
		int mutex_ret = pthread_mutex_destroy(&send_lock);
		if(mutex_ret<0)
			printf("[robotbase] mutex destroy fail\n");
	}
}

void *serial_receive_routine(void *)
{
	pthread_detach(pthread_self());
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
					ROS_DEBUG_NAMED(ROBOTBASE," tail incorret\n");
				}
			} else {
				ROS_DEBUG_NAMED(ROBOTBASE, "crc incorret\n");
			}
		}
	}
	//pthread_exit(NULL);
}

void *robotbase_routine(void*)
{
	pthread_detach(pthread_self());
	float	th_last, vth, pose_x, pose_y;
	float	previous_angle = std::numeric_limits<float>::max();
	float	delta_angle = 0;
	int16_t	angle;

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

		if(is_line_angle_offset == true){
			if(line_angle_offset == std::numeric_limits<float>::max())
				line_angle_offset = sensor.angle;
			sensor.angle -= line_angle_offset;
		}else
			line_angle_offset =std::numeric_limits<float>::max();

		if(enable_slam_offset)
		{
			// Compensate the angle with the offset published by slam
			sensor.angle -= slam_angle_offset;
			// Check for avoiding angle's sudden change
			if (previous_angle == std::numeric_limits<float>::max())
				previous_angle = sensor.angle;
			else
			{
				delta_angle = sensor.angle - previous_angle;
				// Format the delta_angle into Range(-180, 180)
				delta_angle = (180 < delta_angle ? delta_angle - 360 : delta_angle);
				delta_angle = (delta_angle < -180 ? delta_angle + 360 : delta_angle);
				// Decide whether it is a sudden change
				if (10 < fabs(delta_angle))
				{
					// It is a sudden change, discard this value
					sensor.angle = previous_angle;
				}
				// Save current angle as previous_angle
				previous_angle = sensor.angle;
			}
		}
		else{
			previous_angle = std::numeric_limits<float>::max();
			slam_angle_offset=0;
		}

//		ROS_WARN("angle(%d),\n",angle);
//		ROS_INFO("previous_angle(%f),\n",previous_angle);
//		ROS_WARN("line_angle_offset(%f)\n",line_angle_offset);
//		ROS_INFO("sensor.angle(%f),\n", sensor.angle);

		sensor.angle_v = -(float)((receiStream[8] << 8) | receiStream[9]) / 100.0;

		sensor.lw_crt = (((receiStream[10] << 8) | receiStream[11]) & 0x7fff) * 1.622;
		sensor.rw_crt = (((receiStream[12] << 8) | receiStream[13]) & 0x7fff) * 1.622;

		sensor.left_wall = ((receiStream[14] << 8)| receiStream[15]);
		#if ROBOT_X600
		sensor.obs0 = ((receiStream[16]<<8) | receiStream[17]);
		sensor.obs1 = ((receiStream[18] << 8) | receiStream[19]);
		sensor.obs2 = ((receiStream[20] << 8) | receiStream[21]);
		#elif ROBOT_X400
		sensor.l_obs = ((receiStream[16] << 8) | receiStream[17]);
		sensor.f_obs = ((receiStream[18] << 8) | receiStream[19]);
		sensor.r_obs = ((receiStream[20] << 8) | receiStream[21]);
		#endif
		sensor.lbumper = (receiStream[22] & 0xf0) ? true : false;
		sensor.rbumper = (receiStream[22] & 0x0f) ? true : false;

		sensor.ir_ctrl = receiStream[23];
		sensor.c_stub = (receiStream[24] << 16) | (receiStream[25] << 8) | receiStream[26];
		sensor.key = receiStream[27];
		sensor.c_s = receiStream[28];
//		ROS_INFO("charge status: %x.", sensor.c_s);
		sensor.w_tank = (receiStream[29] > 0) ? true : false;
		sensor.batv = receiStream[30];

		sensor.lcliff = ((receiStream[31] << 8) | receiStream[32]);
		sensor.fcliff = ((receiStream[33] << 8) | receiStream[34]);
		sensor.rcliff = ((receiStream[35] << 8) | receiStream[36]);

		sensor.lbrush_oc = (receiStream[37] & 0x08) ? true : false;		// left brush over current
		sensor.mbrush_oc = (receiStream[37] & 0x04) ? true : false;		// main brush over current
		sensor.rbrush_oc = (receiStream[37] & 0x02) ? true : false;		// right brush over current
		sensor.vcum_oc = (receiStream[37] & 0x01) ? true : false;		// vaccum over current
		sensor.gyro_dymc = receiStream[38];
		sensor.right_wall = ((receiStream[39]<<8)|receiStream[40]);
		sensor.x_acc = ((receiStream[41]<<8)|receiStream[42])/66564.0f; //in G
		sensor.y_acc = ((receiStream[43]<<8)|receiStream[44])/66564.0f; //in G
		sensor.z_acc = ((receiStream[45]<<8)|receiStream[46])/66564.0f; //in G

		#if ROBOT_X600
		sensor.obs3 = ((receiStream[47]<<8)|receiStream[48]);
		sensor.obs4 = ((receiStream[49]<<8)|receiStream[50]);
		sensor.obs5 = ((receiStream[51]<<8)|receiStream[52]);
		sensor.obs6 = ((receiStream[53]<<8)|receiStream[54]);
		sensor.obs7 = ((receiStream[55]<<8)|receiStream[56]);
		#endif
	
		cur_time = ros::Time::now();

		if(sensor.right_wall>0)
		{
			ROS_DEBUG_NAMED(ROBOTBASE,"on stm32 crc calculate bad time %d ",sensor.right_wall);
			error_count++;
			ROS_WARN("[robotbase.cpp] sensor.right_wall value: %d, error count: %d.", sensor.right_wall, error_count);
		}
		else{
			error_count = 0;
		}
		float vx = (sensor.lw_vel + sensor.rw_vel) / 2.0;
		float th = sensor.angle * 0.01745;					//turn degrees into radians
		float dt = (cur_time - last_time).toSec();

		last_time = cur_time;
		//vth = (th - th_last) / dt;
		//sensor.angle_v = vth * 57.3;
		//sensor.angle = th;
		//sensor.angle_v = vth;
		//th_last = th;

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
		sensor_pub.publish(sensor);
	}
	//pthread_exit(NULL);
}

void *serial_send_routine(void*){
	pthread_detach(pthread_self());
	ros::Time start_t;
	ros::Rate r(_RATE);double proc_t;
	uint8_t buf[SEND_LEN];
	int sl = SEND_LEN-3;
	ResetSendFlag();
	while(send_stream_thread){
		start_t =  ros::Time::now();
		r.sleep();
		// Force reset the beep action when Beep() function is called, especially when last beep action is not over. It can stop last beep action and directly start the updated beep action.
		if (robotbase_beep_update_flag){
			temp_speaker_sound_time_count = -1;
			temp_speaker_silence_time_count = 0;
			robotbase_beep_update_flag = false;
		}
		//printf("[robotbase.cpp] tmp_sound_count:%d, tmp_silence_count:%d, sound_loop_count:%d.\n", temp_speaker_sound_time_count, temp_speaker_silence_time_count, robotbase_speaker_sound_loop_count);
		// If beep_time_count has ran out, it will not sound anymore and check the battary status. If low battary, it will constantly beep to alarm.
		// If count > 0, it is processing for different alarm, if count < 0, it should be processing low battary alarm.
		if (robotbase_speaker_sound_loop_count != 0){
			process_beep();
		}else{
			// Trigger constant beep alarm for low battary alarm, it has the lowest priority among all the alarms, so it can be interrupted by other alarm.
			if (lowBattery){
				Beep(3, 25, 25, -1);
			}
		}

		SetSendFlag();
		//pthread_mutex_lock(&send_lock);
		memcpy(buf,sendStream,sizeof(uint8_t)*SEND_LEN);
		//pthread_mutex_unlock(&send_lock);
		buf[CTL_CRC] = calcBufCrc8((char *)buf, sl);
		if(buf[CTL_CRC] != calcBufCrc8((char*)sendStream,sl))
			ROS_DEBUG_NAMED(ROBOTBASE,"on send process crc incorret!!");
		serial_write(SEND_LEN, buf);
		proc_t = (ros::Time::now()-start_t).toSec();
		if(proc_t>0.025)
			ROS_DEBUG_NAMED(ROBOTBASE,"process time %f",proc_t);
		ResetSendFlag();
	}
	//pthread_exit(NULL);
}

void slam_angle_offset_callback(const pp::slam_angle_offset::ConstPtr& msg)
{
	// Update the angle offset given by slam
	if(ros::ok()&&(!robotbase_thread_stop)){
		slam_angle_offset = msg->slam_angle_offset;
	}
	ROS_INFO("[robotbase] Get slam_angle_offset as: %f.\n", slam_angle_offset);
}

void process_beep(){
	// This routine handles the speaker sounding logic
	// If temp_speaker_silence_time_count == 0, it is the end of loop of silence, so decrease the count and set sound in sendStream.
	if (temp_speaker_silence_time_count == 0){
		temp_speaker_silence_time_count--;
		temp_speaker_sound_time_count = robotbase_speaker_sound_time_count;
		control_set(CTL_BUZZER, robotbase_sound_code & 0xFF);
	}
	// If temp_speaker_sound_time_count == 0, it is the end of loop of sound, so decrease the count and set sound in sendStream.
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
