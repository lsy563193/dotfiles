#include "dev.h"

#include <stdio.h>
#include <time.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/SetBool.h>
#include <odom.h>
#include <event_manager.h>
#include <r16_board_test.hpp>
#include "lidar.hpp"
#include "robot.hpp"
#include "slam.h"


#include "action.hpp"
#include "movement.hpp"
#include "move_type.hpp"
#include "state.hpp"
#include "mode.hpp"
#include "std_srvs/Empty.h"


pthread_mutex_t recev_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  recev_cond = PTHREAD_COND_INITIALIZER;

pthread_mutex_t serial_data_ready_mtx = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t serial_data_ready_cond = PTHREAD_COND_INITIALIZER;

// For obs dynamic adjustment
int OBS_adjust_count = 50;

// Lock for odom coordinate
boost::mutex odom_mutex;

bool g_pp_shutdown = false;

bool robotbase_thread_stop = false;
bool send_thread_stop = false;
bool recei_thread_stop = false;
bool event_manager_thread_stop = false;
bool event_handle_thread_stop = false;
bool core_thread_stop = false;

// todo: These variables and the function should be moved to Beeper class.
bool robotbase_beep_update_flag = false;
int robotbase_beeper_sound_loop_count = 0;
uint8_t robotbase_sound_code = 0;
int robotbase_beeper_sound_time_count = 0;
int temp_beeper_sound_time_count = -1;
int robotbase_beeper_silence_time_count = 0;
int temp_beeper_silence_time_count = 0;

// For led control.
uint8_t robotbase_led_type = LED_STEADY;
bool robotbase_led_update_flag = false;
uint8_t robotbase_led_color = LED_GREEN;
uint16_t robotbase_led_cnt_for_one_cycle = 0;
uint16_t live_led_cnt_for_switch = 0;

void process_led()
{
	uint16_t led_brightness = 100;
	switch (robotbase_led_type)
	{
		case LED_STEADY:
		{
			robotbase_led_update_flag = false;
			break;
		}
		case LED_FLASH:
		{
			if (live_led_cnt_for_switch > robotbase_led_cnt_for_one_cycle / 2)
				led_brightness = 0;
			break;
		}
		case LED_BREATH:
		{
			if (live_led_cnt_for_switch > robotbase_led_cnt_for_one_cycle / 2)
				led_brightness = led_brightness * (2 * (float)live_led_cnt_for_switch / (float)robotbase_led_cnt_for_one_cycle - 1.0);
			else
				led_brightness = led_brightness * (1.0 - 2 * (float)live_led_cnt_for_switch / (float)robotbase_led_cnt_for_one_cycle);
			break;
		}
	}

	if (live_led_cnt_for_switch++ > robotbase_led_cnt_for_one_cycle)
		live_led_cnt_for_switch = 0;

	switch (robotbase_led_color)
	{
		case LED_GREEN:
		{
			led.set(led_brightness, 0);
			break;
		}
		case LED_ORANGE:
		{
			led.set(led_brightness, led_brightness);
			break;
		}
		case LED_RED:
		{
			led.set(0, led_brightness);
			break;
		}
		case LED_OFF:
		{
			led.set(0, 0);
			break;
		}
	}
	//ROS_INFO("%s %d: live_led_cnt_for_switch: %d, led_brightness: %d.", __FUNCTION__, __LINE__, live_led_cnt_for_switch, led_brightness);
}

//extern pp::x900sensor sensor;
robot::robot()/*:offset_angle_(0),saved_offset_angle_(0)*/
{

	robotbase_thread_stop = false;
	send_thread_stop = false;
	recei_thread_stop = false;
	core_thread_stop = false;
	event_manager_thread_stop = false;
	event_handle_thread_stop = false;

	// Subscribers.
	odom_sub_ = robot_nh_.subscribe("/odom", 1, &robot::robotOdomCb, this);
	// Service clients.
	lidar_motor_cli_ = robot_nh_.serviceClient<rplidar_ros::SetLidar>("lidar_motor_ctrl");
	end_slam_cli_ = robot_nh_.serviceClient<std_srvs::Empty>("End_Slam");
	start_slam_cli_ = robot_nh_.serviceClient<std_srvs::Empty>("Start_Slam");
	robot_tf_ = new tf::TransformListener(robot_nh_, ros::Duration(0.1), true);

	// Publishers.
	odom_pub_ = robot_nh_.advertise<nav_msgs::Odometry>("robot_odom", 1);
	scan_ctrl_pub_ = robot_nh_.advertise<pp::scan_ctrl>("scan_ctrl", 1);

	resetCorrection();

	setBaselinkFrameType(ODOM_POSITION_ODOM_ANGLE);

#if VERIFY_CPU_ID
	if (verify_cpu_id() < 0) {
		verify_ok = false;
	}
#endif

#if VERIFY_KEY
	if (verify_ok == true && verify_key() == 0) {
		verify_ok = false;
	}
#endif

	std::string serial_port;
	robot_nh_.param<std::string>("serial_port", serial_port, "/dev/ttyS2");

	int baud_rate;
	robot_nh_.param<int>("baud_rate", baud_rate, 115200);

#if !R16_BOARD_TEST

	std::string lidar_bumper_dev;
	robot_nh_.param<std::string>("lidar_bumper_file", lidar_bumper_dev, "/dev/input/event0");

	if (bumper.lidarBumperInit(lidar_bumper_dev.c_str()) == -1)
		ROS_ERROR(" lidar bumper open fail!");

	// Init for serial.
	if (!serial.init(serial_port, baud_rate))
	{
		ROS_ERROR("%s %d: Serial init failed!!", __FUNCTION__, __LINE__);
	}

	while (!serial.isReady()) {
		ROS_ERROR("serial not ready\n");
	}

	serial.resetSendStream();

	ROS_INFO("waiting robotbase awake ");
	auto serial_receive_routine = new boost::thread(boost::bind(&Serial::receive_routine_cb, &serial));
	auto robotbase_routine = new boost::thread(boost::bind(&robot::robotbase_routine_cb, this));
	auto serial_send_routine = new boost::thread(boost::bind(&Serial::send_routine_cb, &serial));
	auto speaker_play_routine = new boost::thread(boost::bind(&Speaker::playRoutine, &speaker));
	// Init for event manager.
	event_manager_init();
	auto event_manager_thread = new boost::thread(event_manager_thread_cb);
	auto event_handler_thread = new boost::thread(event_handler_thread_cb);
	auto core_thread = new boost::thread(boost::bind(&robot::core_thread_cb,this));
#else
	auto speaker_play_routine = new boost::thread(boost::bind(&Speaker::playRoutine, &speaker));
	auto robotbase_routine = new boost::thread(boost::bind(&robot::robotbase_routine_cb, this));

	auto test_routine = new boost::thread(boost::bind(&r16_board_test, serial_port, baud_rate));
#endif
	ROS_INFO("%s %d: robot init done!", __FUNCTION__, __LINE__);
}

void robot::robotbase_routine_cb()
{
	ROS_INFO("robotbase,\033[32m%s\033[0m,%d is up.",__FUNCTION__,__LINE__);

	ros::Rate	r(_RATE);
	ros::Time	cur_time, last_time;
	uint8_t buf[RECEI_LEN];

	//Debug
	uint16_t error_count = 0;

	nav_msgs::Odometry			odom_msg;
	tf::TransformBroadcaster	odom_broad;
	geometry_msgs::Quaternion	odom_quat;

	geometry_msgs::TransformStamped	odom_trans;

	ros::Publisher	odom_pub, sensor_pub;
	ros::NodeHandle	robotsensor_node;
	ros::NodeHandle	odom_node;

	sensor_pub = robotsensor_node.advertise<pp::x900sensor>("/robot_sensor",1);
	odom_pub = odom_node.advertise<nav_msgs::Odometry>("/odom",1);

	cur_time = ros::Time::now();
	last_time  = cur_time;
	int16_t last_rcliff = 0, last_fcliff = 0, last_lcliff = 0;
	int16_t last_x_acc = -1000, last_y_acc = -1000, last_z_acc = -1000;
	while (ros::ok() && !robotbase_thread_stop)
	{
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock)!=0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond,&recev_lock)!=0, "robotbase pthread receive cond wait fail");
		memcpy(buf,serial.receive_stream,sizeof(uint8_t)*RECEI_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock)!=0, "robotbase pthread receive unlock fail");
//		debugReceivedStream(buf);

		pp::x900sensor sensor;

		// For wheel device.
		wheel.setLeftWheelActualSpeed(static_cast<float>(static_cast<int16_t>((buf[REC_WHEEL_L_SPEED_H] << 8) | buf[REC_WHEEL_L_SPEED_L]) / 1000.0));
		wheel.setRightWheelActualSpeed(static_cast<float>(static_cast<int16_t>((buf[REC_WHEEL_R_SPEED_H] << 8) | buf[REC_WHEEL_R_SPEED_L]) / 1000.0));
		sensor.left_wheel_speed = wheel.getLeftWheelActualSpeed();
		sensor.right_wheel_speed = wheel.getRightWheelActualSpeed();

		wheel.setLeftWheelCliffStatus((buf[REC_WHEEL_CLIFF] & 0x02) != 0);
		wheel.setRightWheelCliffStatus((buf[REC_WHEEL_CLIFF] & 0x01) != 0);
		sensor.left_wheel_cliff = wheel.getLeftWheelCliffStatus();
		sensor.right_wheel_cliff = wheel.getRightWheelCliffStatus();

		// For gyro device.
		gyro.setCalibration(buf[REC_GYRO_CALIBRATION] != 0);
		sensor.gyro_calibration = gyro.getCalibration();

		gyro.setAngle(static_cast<float>(static_cast<int16_t>((buf[REC_ANGLE_H] << 8) | buf[REC_ANGLE_L]) / 100.0 * -1));
		sensor.angle = gyro.getAngle();
		gyro.setAngleV(static_cast<float>(static_cast<int16_t>((buf[REC_ANGLE_V_H] << 8) | buf[REC_ANGLE_V_L]) / 100.0 * -1));
		sensor.angle_v = gyro.getAngleV();

		if (gyro.getXAcc() == -1000)
			gyro.setXAcc(static_cast<int16_t>((buf[REC_XACC_H] << 8) | buf[REC_XACC_L]));
		else
			gyro.setXAcc(static_cast<int16_t>((static_cast<int16_t>((buf[REC_XACC_H] << 8)|buf[REC_XACC_L]) + gyro.getXAcc()) / 2));
		if (gyro.getYAcc() == -1000)
			gyro.setYAcc(static_cast<int16_t>((buf[REC_YACC_H] << 8) | buf[REC_YACC_L]));
		else
			gyro.setYAcc(static_cast<int16_t>((static_cast<int16_t>((buf[REC_YACC_H] << 8)|buf[REC_YACC_L]) + gyro.getYAcc()) / 2));
		if (gyro.getZAcc() == -1000)
			gyro.setZAcc(static_cast<int16_t>((buf[REC_ZACC_H] << 8) | buf[REC_ZACC_L]));
		else
			gyro.setZAcc(static_cast<int16_t>((static_cast<int16_t>((buf[REC_ZACC_H] << 8)|buf[REC_ZACC_L]) + gyro.getZAcc()) / 2));

		sensor.x_acc = gyro.getXAcc();//in mG
		sensor.y_acc = gyro.getYAcc();//in mG
		sensor.z_acc = gyro.getZAcc();//in mG

		// For wall sensor device.
		wall.setLeft((buf[REC_L_WALL_H] << 8)| buf[REC_L_WALL_L]);
		sensor.left_wall = wall.getLeft();
		wall.setRight((buf[REC_R_WALL_H] << 8)| buf[REC_R_WALL_L]);
		sensor.right_wall = wall.getRight();

		// For obs sensor device.
		obs.setLeft((buf[REC_L_OBS_H] << 8) | buf[REC_L_OBS_L]);
		sensor.left_obs = obs.getLeft();
		obs.setFront((buf[REC_F_OBS_H] << 8) | buf[REC_F_OBS_L]);
		sensor.front_obs = obs.getFront();
		obs.setRight((buf[REC_R_OBS_H] << 8) | buf[REC_R_OBS_L]);
		sensor.right_obs = obs.getRight();

		// For bumper device.
		bumper.setLeft((buf[REC_BUMPER_AND_CLIFF] & 0x20) != 0);
		bumper.setRight((buf[REC_BUMPER_AND_CLIFF] & 0x10) != 0);
		sensor.left_bumper = bumper.getLeft();
		sensor.right_bumper = bumper.getRight();

		bumper.setLidarBumperStatus();
		sensor.lidar_bumper = bumper.getLidarBumperStatus();
		if (bumper.getLidarBumperStatus())
		{
			bumper.setLeft(true);
			bumper.setRight(true);
		}

		// For cliff device.
		cliff.setLeft((buf[REC_BUMPER_AND_CLIFF] & 0x04) != 0);
		cliff.setFront((buf[REC_BUMPER_AND_CLIFF] & 0x02) != 0);
		cliff.setRight((buf[REC_BUMPER_AND_CLIFF] & 0x01) != 0);
		sensor.right_cliff = cliff.getRight();
		sensor.front_cliff = cliff.getFront();
		sensor.left_cliff = cliff.getLeft();

		// For remote device.
		auto remote_signal = buf[REC_REMOTE];
		remote.set(remote_signal);
		sensor.remote = remote.get();
		if (remote_signal > 0)
			ROS_INFO("%s %d: Remote received:%d", __FUNCTION__, __LINE__, remote_signal);

		// For rcon device.
		c_rcon.setStatus((buf[REC_RCON_CHARGER_4] << 24) | (buf[REC_RCON_CHARGER_3] << 16)
						 | (buf[REC_RCON_CHARGER_2] << 8) | buf[REC_RCON_CHARGER_1]);
		sensor.rcon = c_rcon.getStatus();

		// For virtual wall.
		sensor.virtual_wall = (buf[REC_VISUAL_WALL_H] << 8)| buf[REC_VISUAL_WALL_L];

		// For key device.
		key.eliminate_jitter((buf[REC_MIX_BYTE] & 0x01) != 0);
		sensor.key = key.getTriggerStatus();

		// For timer device.
		robot_timer.setPlanStatus(static_cast<uint8_t>((buf[REC_MIX_BYTE] >> 1) & 0x03));
		sensor.plan = robot_timer.getPlanStatus();

		// For water tank device.
		sensor.water_tank = (buf[REC_MIX_BYTE] & 0x08) != 0;

		// For charger device.
		charger.setChargeStatus((buf[REC_MIX_BYTE] >> 4) & 0x07);
		sensor.charge_status = charger.getChargeStatus();

		// For sleep status.
		serial.isMainBoardSleep((buf[REC_MIX_BYTE] & 0x80) == 0);
		sensor.main_board_sleep_status = serial.isMainBoardSleep();

		// For battery device.
		battery.setVoltage(buf[REC_BATTERY] * 10);
		sensor.battery = static_cast<float>(battery.getVoltage() / 100.0);

		// For over current checking.
		vacuum.setOc((buf[REC_OC] & 0x01) != 0);
		sensor.vacuum_oc = vacuum.getOc();
		brush.setRightOc((buf[REC_OC] & 0x02) != 0);
		sensor.right_brush_oc = brush.getRightOc();
		brush.setMainOc((buf[REC_OC] & 0x04) != 0);
		sensor.main_brush_oc = brush.getMainOc();
		brush.setLeftOc((buf[REC_OC] & 0x08) != 0);
		sensor.left_brush_oc = brush.getLeftOc();
		wheel.setRightWheelOc((buf[REC_OC] & 0x10) != 0);
		sensor.right_wheel_oc = wheel.getRightWheelOc();
		wheel.setLeftWheelOc((buf[REC_OC] & 0x20) != 0);
		sensor.left_wheel_oc = wheel.getLeftWheelOc();

		// For debug.
//		printf("%d: REC_MIX_BYTE:(%2x), REC_RESERVED:(%2x).\n.",
//			   __LINE__, buf[REC_MIX_BYTE], buf[REC_RESERVED]);
//		printf("%d: charge:(%d), remote:(%d), key:(%d), rcon(%d).\n.",
//			   __LINE__, charger.getChargeStatus(), remote.get(), key.getTriggerStatus(), c_rcon.getStatus());

		pthread_cond_broadcast(&serial_data_ready_cond);
		sensor_pub.publish(sensor);
		is_sensor_ready_ = true;

		/*---------extrict end-------*/


		/*------------setting for odom and publish odom topic --------*/
		boost::mutex::scoped_lock lock(odom_mutex);
		odom.setMovingSpeed(static_cast<float>((wheel.getLeftWheelActualSpeed() + wheel.getRightWheelActualSpeed()) / 2.0));
		odom.setRadian(degree_to_radian(gyro.getAngle()));
		odom.setAngleSpeed(gyro.getAngleV());
		cur_time = ros::Time::now();
		double angle_rad, dt;
		angle_rad = odom.getRadian();
		dt = (cur_time - last_time).toSec();
		last_time = cur_time;
		odom.setX(static_cast<float>(odom.getX() + (odom.getMovingSpeed() * cos(angle_rad)) * dt));
		odom.setY(static_cast<float>(odom.getY() + (odom.getMovingSpeed() * sin(angle_rad)) * dt));
		odom_quat = tf::createQuaternionMsgFromYaw(angle_rad);
		odom_msg.header.stamp = cur_time;
		odom_msg.header.frame_id = "odom";
		odom_msg.child_frame_id = "base_link";
		odom_msg.pose.pose.position.x = odom.getX();
		odom_msg.pose.pose.position.y = odom.getY();
		odom_msg.pose.pose.position.z = 0.0;
		odom_msg.pose.pose.orientation = odom_quat;
		odom_msg.twist.twist.linear.x = odom.getMovingSpeed();
		odom_msg.twist.twist.linear.y = 0.0;
		odom_msg.twist.twist.angular.z = odom.getAngleSpeed();
		odom_pub.publish(odom_msg);

		odom_trans.header.stamp = cur_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		odom_trans.transform.translation.x = odom.getX();
		odom_trans.transform.translation.y = odom.getY();
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		odom_broad.sendTransform(odom_trans);
		/*------publish end -----------*/

		// Check tilt
#if GYRO_DYNAMIC_ADJUSTMENT
		if (wheel.getLeftWheelActualSpeed() < 0.01 && wheel.getRightWheelActualSpeed() < 0.01)
			gyro.setDynamicOn();
		else
			gyro.setDynamicOff();
		gyro.checkTilt();
#endif
		// Dynamic adjust obs
		obs.DynamicAdjust(OBS_adjust_count);

		// Check for whether robot should publish this frame of scan.
		if (p_mode != nullptr)
		{
			if (p_mode->sp_action_ != nullptr && p_mode->isInitState())
				scan_ctrl_.allow_publishing = 1;
			else
				scan_ctrl_.allow_publishing =
						!(fabs(wheel.getLeftWheelActualSpeed() - wheel.getRightWheelActualSpeed()) > 0.1
						  || (wheel.getLeftWheelActualSpeed() * wheel.getRightWheelActualSpeed() < 0)
						  || bumper.getStatus()
						  || gyro.getTiltCheckingStatus()
						  || abs(wheel.getLeftSpeedAfterPid() - wheel.getRightSpeedAfterPid()) > 100
						  || wheel.getLeftSpeedAfterPid() * wheel.getRightSpeedAfterPid() < 0);
		}
		else
		scan_ctrl_.allow_publishing = 1;
		scan_ctrl_pub_.publish(scan_ctrl_);

	}//end while
	pthread_cond_broadcast(&serial_data_ready_cond);
	event_manager_thread_stop = true;
	ROS_ERROR("%s,%d,exit",__FUNCTION__,__LINE__);
}

void robot::core_thread_cb()
{
	ROS_INFO("Waiting for robot sensor ready.");
	while (!isSensorReady()) {
		usleep(1000);
	}
	ROS_INFO("Robot sensor ready.");
//	speaker.play(VOICE_WELCOME_ILIFE);
	usleep(200000);

	if (charger.isOnStub() || charger.isDirected())
		p_mode.reset(new ModeCharge());
	else
	{
		speaker.play(VOICE_PLEASE_START_CLEANING, false);
		p_mode.reset(new ModeIdle());
	}

	while(ros::ok())
	{
//		ROS_INFO("%s %d: %x", __FUNCTION__, __LINE__, p_mode);
		p_mode->run();
		if(core_thread_stop) break;
		auto next_mode = p_mode->getNextMode();
		p_mode.reset();
//		ROS_INFO("%s %d: %x", __FUNCTION__, __LINE__, p_mode);
		p_mode.reset(getNextMode(next_mode));
//		ROS_INFO("%s %d: %x", __FUNCTION__, __LINE__, p_mode);
	}
	g_pp_shutdown = true;
	ROS_ERROR("%s,%d,exit",__FUNCTION__,__LINE__);
}

robot::~robot()
{
	bumper.lidarBumperDeinit();
	recei_thread_stop = true;
	led.setMode(LED_STEADY, LED_OFF);
	serial.setSendData(CTL_BEEPER, 0x00);
	gyro.setOff();
	wheel.stop();
	brush.stop();
	vacuum.stop();
	serial.setMainBoardMode(NORMAL_SLEEP_MODE);
	usleep(40000);
	while(ros::ok() && !g_pp_shutdown){
		usleep(2000);
	}
	serial.close();
	pthread_mutex_destroy(&recev_lock);
	pthread_mutex_destroy(&serial_data_ready_mtx);

	pthread_cond_destroy(&recev_cond);
	pthread_cond_destroy(&serial_data_ready_cond);

	pthread_mutex_destroy(&new_event_mtx);
	pthread_mutex_destroy(&event_handler_mtx);
	pthread_cond_destroy(&new_event_cond);
	pthread_cond_destroy(&event_handler_cond);

	delete robot_tf_;
	ROS_INFO("pp shutdown!");
}

robot *robot::instance()
{
	extern robot* robot_instance;
	return robot_instance;
}

void robot::robotOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
	tf::StampedTransform		transform;

	tf::Vector3	odom_pose;
	double	odom_pose_radian_;

	if (getBaselinkFrameType() == SLAM_POSITION_SLAM_ANGLE || getBaselinkFrameType() == SLAM_POSITION_ODOM_ANGLE)
	{
		if(slam.isMapReady())
		{
			try {
				robot_tf_->lookupTransform("/map", "/base_link", ros::Time(0), transform);
				auto tmp = transform.getOrigin();
				auto tmp_yaw = tf::getYaw(transform.getRotation());

				robot_tf_->lookupTransform("/odom", "/base_link", ros::Time(0), transform);
				if(getBaselinkFrameType() == SLAM_POSITION_ODOM_ANGLE)
					tmp_yaw = tf::getYaw(transform.getRotation());

				odom_pose = transform.getOrigin();
				odom_pose_radian_ = tf::getYaw(transform.getRotation());
				slam_correction_pos = tmp - odom_pose;
				slam_correction_yaw_ = tmp_yaw - odom_pose_radian_;
			} catch(tf::TransformException e)
			{
				ROS_WARN("%s %d: Failed to compute map transform, skipping scan (%s)", __FUNCTION__, __LINE__, e.what());
			}

			if (!isTfReady())
			{
				ROS_INFO("%s %d: TF ready.", __FUNCTION__, __LINE__);
				setTfReady(true);
			}
		}
	}
	else if (getBaselinkFrameType() == ODOM_POSITION_ODOM_ANGLE)
	{
		//ROS_INFO("SLAM = 0");
		odom_pose.setX(odom.getX());
		odom_pose.setY(odom.getY());
		odom_pose_radian_ = odom.getRadian();
	}

#if 1
	updateRobotPose(odom_pose, odom_pose_radian_);
	odomPublish();
	//printf("Map->base(%f, %f, %f). Map->robot (%f, %f, %f)\n", tmp_x, tmp_y, RAD2DEG(tmp_yaw), robot_x_, robot_y_, RAD2DEG(robot_radian_));
//	ROS_WARN("Position (%f, %f), angle: %f.", robot_pos.getX(), robot_pos.getY(), robot_pos.getRadian());
#else
	pose.setX(tmp_x_);
std::	pose.setY(tmp_y_);
	pose.setAngle(tmp_yaw_);
#endif
	//ROS_WARN("%s %d: Position (%f, %f), yaw: %f. Odom position(%f, %f), yaw: %f.", __FUNCTION__, __LINE__, tmp_x, tmp_y, tmp_yaw, odom_pose_x_, odom_pose_y_, odom_pose_radian_);
	//ROS_WARN("%s %d: Position diff(%f, %f), yaw diff: %f.", __FUNCTION__, __LINE__, tmp_x - odom_pose_x_, tmp_y - odom_pose_y_, tmp_yaw - odom_pose_radian_);
	//ROS_WARN("%s %d: Odom diff(%f, %f).", __FUNCTION__, __LINE__, odom_pose_x_ - msg->pose.pose.position.x, odom_pose_y_ - msg->pose.pose.position.y);
	//ROS_WARN("%s %d: Correct  diff(%f, %f), yaw diff: %f.", __FUNCTION__, __LINE__, correct_x, correct_y, correct_yaw);
}

void robot::odomPublish()
{
	ros::Time cur_time;
	nav_msgs::Odometry robot_pose;
	robot_pose.header.stamp = cur_time;
	robot_pose.header.frame_id = "map";
	robot_pose.child_frame_id = "robot";
	robot_pose.pose.pose.position.x = robot_pos.x();
	robot_pose.pose.pose.position.y = robot_pos.y();
	robot_pose.pose.pose.position.z = 0.0;
	robot_pose.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_radian_);
	robot_pose.twist.twist.linear.x = 0.0;
	robot_pose.twist.twist.linear.y = 0.0;
	robot_pose.twist.twist.angular.z = 0.0;
	odom_pub_.publish(robot_pose);
}



bool robot::lidarMotorCtrl(bool switch_)
{
	rplidar_ros::SetLidar ctrl_message;
	ctrl_message.request.switch_status = switch_;

	if (lidar_motor_cli_.call(ctrl_message))
	{
		ROS_INFO("\033[35m" "%s %d: Service response: %s" "\033[0m", __FUNCTION__, __LINE__, ctrl_message.response.message.c_str());
		return true;
	}
	return false;
}

bool robot::slamStart(void)
{
	std_srvs::Empty empty;
	ROS_INFO("%s %d: Call start slam service.", __FUNCTION__, __LINE__);
	return start_slam_cli_.call(empty);
}

bool robot::slamStop(void)
{
	std_srvs::Empty empty;
	return end_slam_cli_.call(empty);
}

void robot::initOdomPosition()
{
	// Reset the odom pose to (0, 0)
	boost::mutex::scoped_lock lock(odom_mutex);
	odom.setX(0);
	odom.setY(0);
}

void robot::updateRobotPose(tf::Vector3& odom, double odom_yaw)
{
	auto cal_scale = [](double val){
		double scale = fabs(val) > 0.05 ? 0.1 * fabs(val) / 0.05 : 0.03;
		scale = scale > 1.0 ? 1.0 : scale;
		return scale;
	};
	if (wheel.getLeftSpeedAfterPid() * wheel.getRightSpeedAfterPid() > 0)
	{
		auto diff = slam_correction_pos - robot_correction_pos;
		robot_correction_pos += {cal_scale(diff.x()) * diff.x(), cal_scale(diff.y()) * diff.y(), 0};
		double yaw = ranged_radian(slam_correction_yaw_ - robot_correction_yaw_);
		robot_correction_yaw_ += (yaw) * 0.8;
	}
//	ROS_INFO("%s %d: Odom_yaw:%f, robot_correction_yaw:%f.", __FUNCTION__, __LINE__, odom_yaw, robot_correction_yaw_);
	robot_pos = odom + robot_correction_pos;
	robot_radian_ = (ranged_radian(odom_yaw + robot_correction_yaw_));
}

void robot::resetCorrection()
{
	slam_correction_pos = {};
	slam_correction_yaw_ = 0;
	robot_correction_pos = {};
	robot_correction_yaw_ = 0;
	robot_pos = {};
	robot_radian_ = 0;
}

void robot::obsAdjustCount(int count)
{
#ifdef OBS_DYNAMIC
	OBS_adjust_count = count;
#endif
}

void robot::debugReceivedStream(const uint8_t *buf)
{
	ROS_INFO("%s %d: Received stream:", __FUNCTION__, __LINE__);
	for (int i = 0; i < RECEI_LEN; i++)
		printf("%02d ", i);
	printf("\n");

	for (int i = 0; i < RECEI_LEN; i++)
		printf("%02x ", buf[i]);
	printf("\n");
}

void robot::debugSendStream(const uint8_t *buf)
{
	ROS_INFO("%s %d: Send stream:", __FUNCTION__, __LINE__);
	for (int i = 0; i < SEND_LEN; i++)
		printf("%02d ", i);
	printf("\n");

	for (int i = 0; i < SEND_LEN; i++)
		printf("%02x ", buf[i]);
	printf("\n");
}

//--------------------
static float xCount{}, yCount{};

Point_t getPosition()
{
	return {xCount, yCount, robot::instance()->getWorldPoseRadian()};
}

float cellToCount(int16_t i) {
	return i * CELL_SIZE;
}

void setPosition(float x, float y) {
	xCount = x;
	yCount = y;
}

void resetPosition() {
	xCount = 0;
	yCount = 0;
}

bool isAny(Dir_t dir)
{
	return dir == MAP_ANY;
}

bool isPos(Dir_t dir)
{
	return dir == MAP_POS_X || dir == MAP_POS_Y;
}

bool isXAxis(Dir_t dir)
{
	return dir == MAP_POS_X || dir == MAP_NEG_X;
}

void updatePosition()
{
	auto pos_x = robot::instance()->getWorldPoseX()/* * 1000 * CELL_COUNT_MUL / CELL_SIZE*/;
	auto pos_y = robot::instance()->getWorldPoseY()/* * 1000 * CELL_COUNT_MUL / CELL_SIZE*/;
	setPosition(pos_x, pos_y);
//	ROS_INFO("%s %d:", __FUNCTION__, __LINE__);
}


Mode *getNextMode(int next_mode_i_)
{
	ROS_INFO("%s %d: next mode:%d", __FUNCTION__, __LINE__, next_mode_i_);
	switch (next_mode_i_)
	{
		case Mode::md_charge:
			return new ModeCharge();
		case Mode::md_sleep:
			return new ModeSleep();
		case Mode::md_go_to_charger:
			return new ModeGoToCharger();
		case Mode::md_remote:
			return new ModeRemote();

		case Mode::cm_navigation:
			return new CleanModeNav();
		case Mode::cm_wall_follow:
			return new CleanModeFollowWall();
		case Mode::cm_spot:
			return new CleanModeSpot();
		case Mode::cm_test:
			return new CleanModeTest();
		case Mode::cm_exploration:
			return new CleanModeExploration();
//		case Mode::cm_exploration:
//			return new CleanModeExploration();
		default:
		{
//			ROS_INFO("%s %d: next mode:%d", __FUNCTION__, __LINE__, next_mode_i_);
			return new ModeIdle();
		}
	}
}
