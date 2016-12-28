#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pp/sensor.h>
#include <pp/peripheral.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>
#include <errno.h>
#include "serial.h"
#include "robotbase.h"
#include "crc8.h"
#include "control.h"
#include "log.h"
bool robotbase_thread_stop = false;
bool is_robotbase_init = 0;
pthread_t robotbaseThread_id;
pthread_t receiPortThread_id;
pthread_mutex_t rb_lock;
uint8_t recei_stream_len= 41;
uint8_t receiStream[41] = {0xAA,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xcc,0x33};

void robotbase_init(){
	if (!is_serial_ready()){
		printf("serial not ready");
	}
	int ser_ret,base_ret;
	int mutex_ret;
	control_set_main_pwr(0);//active robot
	control_set_gyro(1,0);//active gyro
	usleep(2000000);//wait 2 second
	mutex_ret = pthread_mutex_init(&rb_lock,NULL);
	if (mutex_ret!=0){
		fprintf(stderr,"failed to initialize robotbase lock %s\n",strerror(mutex_ret));
	}
	ser_ret = pthread_create(&receiPortThread_id,NULL,serial_receive,NULL);
	base_ret = pthread_create(&robotbaseThread_id,NULL,baserun,NULL);
	if (base_ret !=0 || ser_ret!=0){
		is_robotbase_init = false;
		printf("fail to create robotbase thread!!");
	}
	else{
		printf("robotbase thread running...");
		is_robotbase_init = true;
	}
	ros::NodeHandle cmd_node;
	ros::Subscriber cmdvel_sub = cmd_node.subscribe("cmd_vel",10,cmd_vel_callback);
	
}

void robotbase_deinit(void){
	if (is_robotbase_init){
		is_robotbase_init= false;
		robotbase_thread_stop = true;
		pthread_join(robotbaseThread_id,NULL);
		pthread_join(receiPortThread_id,NULL);
		pthread_mutex_destroy(&rb_lock);
		control_set_gyro(0,0);
		control_set_main_pwr(1);
	}
	
}

void * serial_receive(void *){
	int ret;uint8_t r_crc,c_crc;
	uint8_t h1 = 0xaa,h2=0x55,header[2],t1 = 0xcc,t2 = 0x33;
	uint16_t tail = 0xcc33;
	uint8_t wh_len = recei_stream_len -2;//without header bytes
	uint8_t wht_len =wh_len -2;//without header and tail bytes
	uint8_t receiData[wh_len];
	uint8_t tmpSet[wht_len-1];//without crc byte
	while (ros::ok() || (robotbase_thread_stop == false)){
		ret = serial_read(2,header);
		printf("receive stream header %s\n",header);
		if (header[0] == h1 && header[1] == h2){
			ret = serial_read(wh_len,receiData);
			r_crc = receiData[wht_len-1];//receive crc value
			int i ;
			for (i = 0;i<wht_len-1;i++){
				tmpSet[i] = receiData[i];
			}
			c_crc =calcBufCrc8((char *)tmpSet,wht_len-1); //after calculate crc value
			if (r_crc == c_crc){
				if (receiData[wh_len-1] == t2 && receiData[wh_len-2] == t1){
					//pthread_mutex_lock(&rb_lock);
					int j=0;
					for(;j<wht_len;j++){
						receiStream[j+2] =receiData[j];
					}
					//pthread_mutex_unlock(&rb_lock);	
				}else continue;//tail incorrect
			}else continue;//crc incorrect
		}else continue;//header incorrect
		//printf("receive stream %s \n",receiStream);
		usleep(20000);
	}
}

void *baserun(void*){
	ros::NodeHandle robotsensor_node;
	ros::NodeHandle odom_node;
	ros::Publisher odom_pub,sensor_pub;
	sensor_pub = robotsensor_node.advertise<pp::sensor>("/robot_sensor",2);
	odom_pub = odom_node.advertise<nav_msgs::Odometry>("/odom",2);
	nav_msgs::Odometry odom;
	pp::sensor sensor;
	tf::TransformBroadcaster odom_broad;
	geometry_msgs::TransformStamped odom_trans;
	int16_t lw_speed,rw_speed,angle,angle_vel,wall_sensor,left_cliff,right_cliff,front_cliff,left_obs,right_obs,front_obs;
	uint16_t lw_current,rw_current;
	uint8_t  right_bumper,left_bumper,ir_control,key,charge_state,water_tank,battery_vol,left_brush_oc,main_brush_oc,right_brush_oc,vacuum_oc;
	uint32_t charge_stub;
	uint16_t last_bv = 170;
	double pose_x,pose_y;
	geometry_msgs::Quaternion odom_quat;
	ros::Time cur_time,last_time;
	cur_time = ros::Time::now();
	last_time  = cur_time;
	while(ros::ok()){
		lw_speed = receiStream[2]*0x100+receiStream[3];
		rw_speed = receiStream[4]*0x100+receiStream[5];
		angle = receiStream[6]*0x100+receiStream[7];
		angle_vel = receiStream[8]*0x100+receiStream[9];
		lw_current = receiStream[10]*0x100+receiStream[11];
		rw_current = receiStream[12]*0x100+receiStream[13];
		wall_sensor = receiStream[14]*0x100+receiStream[15];
		left_obs = receiStream[16]*0x100+receiStream[17];
		front_obs = receiStream[18]*0x100+receiStream[19];
		right_obs = receiStream[20]*0x100+receiStream[21];
		right_bumper = receiStream[22]&0x0f;
		left_bumper = receiStream[22]&0xf0;
		ir_control = receiStream[23];
		charge_stub = receiStream[24]*0x10000+receiStream[25]*0x100+receiStream[26];
		key = receiStream[27];
		charge_state = receiStream[28];
		water_tank = receiStream[29];
		battery_vol = receiStream[30];
		left_cliff = receiStream[31]*0x100+receiStream[32];
		front_cliff = receiStream[33]*0x100+receiStream[34];
		right_cliff = receiStream[35]*0x100+receiStream[36];
		left_brush_oc = receiStream[37]&0x08;//left brush over current
		main_brush_oc = receiStream[37]&0x04;//main brush over current
		right_brush_oc = receiStream[37]&0x02;//right brush over current
		vacuum_oc = receiStream[37]&0x01;//vacuum over current

		sensor.lw_vel = (float)(lw_speed)/1000.0;
		sensor.rw_vel = (float)(rw_speed)/1000.0;
		sensor.angle = (float)(angle)/100.0;
		sensor.angle_v = (float)(angle_vel)/100.0;
		sensor.lw_crt = lw_current;
		sensor.rw_crt = rw_current;
		sensor.l_obs = left_obs;
		sensor.f_obs = front_obs;
		sensor.r_obs = right_obs;
		sensor.lbumper = left_bumper;
		sensor.rbumper = right_bumper;
		sensor.ir_ctrl = ir_control;
		sensor.c_stub = charge_stub;
		sensor.key = key;
		sensor.c_s = charge_state;
		sensor.w_tank = water_tank;
		sensor.batv = battery_vol;
		sensor.rcliff = right_cliff;
		sensor.lcliff = left_cliff;
		sensor.fcliff = front_cliff;
		sensor.lbrush_oc = left_brush_oc;
		sensor.rbrush_oc = right_brush_oc;
		sensor.mbrush_oc = main_brush_oc;
		sensor.vcum_oc = vacuum_oc;

		if (abs(last_bv - battery_vol) > 1){
			last_bv = battery_vol;
			ROS_INFO("battery voltege: %f",(float)(battery_vol)/10.0);
		}
		
		cur_time = ros::Time::now();
		double vx = (float)(lw_speed+rw_speed)/2.0;
		double th = sensor.angle*3.14159/180.0;
		double dt = (cur_time-last_time).toSec();
		last_time = cur_time;
		double d_x = (vx*cos(th)-0*sin(th))*dt;
		double d_y = (vx*sin(th)+0*cos(th))*dt;
		pose_x +=d_x;
		pose_y +=d_y;
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
		odom.twist.twist.angular.z = sensor.angle_v;
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
		usleep(20000);
	}
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
	float twd = 0.202 ;
	float v_l,v_r;
	v_l = ((2*msg->linear.x)-(msg->angular.z*twd))/2;
	v_r = ((2*msg->linear.x)+(msg->angular.z*twd))/2;
	printf("left wheel speed %f,right wheel speed %f",v_l,v_r);
	printf(" int16 left wheel speed %d,right wheel speed %d",(int16_t)v_l,(int16_t)v_r);
	//control_set_wheel_left_speed((int16_t)v_l);
	//control_set_wheel_right_speed((int16_t)v_r);
}

