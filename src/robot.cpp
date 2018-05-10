#include "dev.h"

#include <stdio.h>
#include <time.h>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/SetBool.h>
#include <odom.h>
#include <event_manager.h>
#include <x900_functional_test.hpp>
#include "lidar.hpp"
#include "robot.hpp"
#include "slam.h"
#include "action.hpp"
#include "movement.hpp"
#include "move_type.hpp"
#include "state.hpp"
#include "mode.hpp"
#include "appointment.h"
#include "std_srvs/Empty.h"

using namespace SERIAL;

std::string consumable_file = "/opt/ros/indigo/share/pp/consumable_status";
std::string consumable_backup_file = "/opt/ros/indigo/share/pp/consumable_status_bk";

pthread_mutex_t recev_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  recev_cond = PTHREAD_COND_INITIALIZER;

pthread_mutex_t serial_data_ready_mtx = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t serial_data_ready_cond = PTHREAD_COND_INITIALIZER;

// For obs dynamic adjustment
int OBS_adjust_count = 50;

bool g_pp_shutdown = false;

bool robotbase_thread_enable = false;
bool send_thread_enable = false;
bool recei_thread_enable = false;

bool robotbase_thread_kill = false;
bool send_thread_kill = false;
bool recei_thread_kill = false;
bool event_manager_thread_kill = false;
bool event_handle_thread_kill = false;
bool core_thread_kill = false;

robot::robot()
{

	robotbase_thread_kill = false;
	send_thread_kill = false;
	recei_thread_kill = false;
	core_thread_kill = false;
	event_manager_thread_kill = false;
	event_handle_thread_kill = false;

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
	x900_ctrl_pub_ = robot_nh_.advertise<pp::x900ctrl>("/robot_ctrl_stream",1);

	resetCorrection();

	setBaselinkFrameType(ODOM_POSITION_ODOM_ANGLE);


	robot_nh_.param<double>("gyro_dynamic_run_time",gyro_dynamic_run_time_,0.5);
	robot_nh_.param<double>("gyro_dynamic_interval",gyro_dynamic_interval_,25);

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

	robot_nh_.param<std::string>("serial_port", serial_port_, "/dev/ttyS2");
	//robot_nh_.param<std::string>("wifi_port",wifi_port_,"dev/ttyS1");
	robot_nh_.param<int>("baud_rate", baud_rate_, 115200);

	robot_nh_.param<std::string>("lidar_bumper_file", lidar_bumper_dev_, "/dev/input/event1");

	loadConsumableStatus();

	while (!serial.isReady()) {
		// Init for serial.
		if (!serial.init(serial_port_, baud_rate_))
			ROS_ERROR("%s %d: Serial init failed!!", __FUNCTION__, __LINE__);
	}

	ROS_INFO("waiting robotbase awake ");
	auto serial_receive_routine = new boost::thread(boost::bind(&Serial::receive_routine_cb, &serial));
	auto speaker_play_routine = new boost::thread(boost::bind(&Speaker::playRoutine, &speaker));
	// Init for event manager.
	event_manager_init();
	auto event_manager_thread = new boost::thread(event_manager_thread_cb);
	auto event_handler_thread = new boost::thread(event_handler_thread_cb);
	auto core_thread = new boost::thread(boost::bind(&robot::core_thread_cb,this));

	auto wifi_send_thread = new boost::thread(boost::bind(&S_Wifi::wifi_send_routine,&s_wifi));

	obs.control(ON);
	ROS_WARN("%s %d: Robot x900(version 0000 r10) is online :)", __FUNCTION__, __LINE__);
}

robot::~robot()
{
	bumper.lidarBumperDeinit();
	recei_thread_kill = true;
	key_led.setMode(LED_STEADY, LED_OFF);
	serial.setSendData(CTL_BEEPER, 0x00);
	gyro.setOff();
	wheel.stop();
	brush.stop();
	vacuum.stop();
	s_wifi.deinit();
	wifi_led.set(false);
	water_tank.stop(WaterTank::operate_option::swing_motor_and_pump);
	serial.setWorkMode(WORK_MODE);
	usleep(40000);
//	while(ros::ok() && !g_pp_shutdown){
	printf("pp shutdown before waiting!\n");
	while(!g_pp_shutdown){
		usleep(2000);
	}
	printf("pp shutdown after waiting!\n");
	serial.close();
	pthread_mutex_destroy(&recev_lock);
	pthread_mutex_destroy(&serial_data_ready_mtx);

	pthread_cond_destroy(&recev_cond);
	pthread_cond_destroy(&serial_data_ready_cond);

	pthread_mutex_destroy(&new_event_mtx);
	pthread_mutex_destroy(&event_handler_mtx);
	pthread_cond_destroy(&new_event_cond);
	pthread_cond_destroy(&event_handler_cond);

//	delete robot_tf_;
	printf("pp shutdown!\n");
}

robot *robot::instance()
{
	extern robot* robot_instance;
	return robot_instance;
}

void robot::robotbase_routine_cb()
{
	ROS_INFO("robotbase,\033[32m%s\033[0m,%d is up.",__FUNCTION__,__LINE__);

	ros::Time	cur_time, last_time;
	uint8_t buf[REC_LEN];

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
	while (ros::ok() && !robotbase_thread_kill)
	{
		if (!robotbase_thread_enable)
		{
			usleep(10000);
			continue;
		}

		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock)!=0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond,&recev_lock)!=0, "robotbase pthread receive cond wait fail");
		memcpy(buf,serial.receive_stream,sizeof(uint8_t)*REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock)!=0, "robotbase pthread receive unlock fail");
//		serial.debugReceivedStream(buf);

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

		gyro.setAngleY(static_cast<float>(static_cast<int16_t>((buf[REC_ANGLE_H] << 8) | buf[REC_ANGLE_L]) / 100.0 * -1));
//		printf("angle:%f, angle_v:%f\n", gyro.getAngleY(), gyro.getAngleV());
		sensor.angle = gyro.getAngleY();
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

		gyro.error((buf[REC_OC]&0x80) ? true : false);

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
//		printf("%s %d: obs left:%d, front:%d, right:%d.\n", __FUNCTION__, __LINE__,
//			   (buf[REC_L_OBS_H] << 8) | buf[REC_L_OBS_L], (buf[REC_F_OBS_H] << 8) | buf[REC_F_OBS_L],
//			   (buf[REC_R_OBS_H] << 8) | buf[REC_R_OBS_L]);

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
		if (remote_signal != 0)
			remote.set(remote_signal);
		sensor.remote = remote.get();
		if (remote_signal > 0)
			ROS_INFO("%s %d: Remote received:%d", __FUNCTION__, __LINE__, remote_signal);

		// For rcon device.
		c_rcon.setStatus((buf[REC_RCON_CHARGER_4] << 24) | (buf[REC_RCON_CHARGER_3] << 16)
						 | (buf[REC_RCON_CHARGER_2] << 8) | buf[REC_RCON_CHARGER_1]);
		sensor.rcon = c_rcon.getStatus();
//		if (c_rcon.getStatus())
//			printf("rcon:%08x, ", c_rcon.getStatus());
//		printf("rcon:%08x\n", c_rcon.getStatus());

		// For virtual wall.
		sensor.virtual_wall = (buf[REC_VISUAL_WALL_H] << 8)| buf[REC_VISUAL_WALL_L];

		// For key device.
		key.eliminate_jitter((buf[REC_MIX_BYTE] & 0x01) != 0);
		sensor.key = key.getTriggerStatus();

		// For appointment status
		appmt_obj.setPlanStatus( buf[REC_MIX_BYTE] );
		sensor.plan = appmt_obj.getPlanStatus();
//		if (appmt_obj.getPlanStatus())
//			ROS_ERROR("RECEIVE PLAN(%d).", appmt_obj.getPlanStatus());

		// For water tank device.
		water_tank.setSwingMotorEquipmentStatus((buf[REC_MIX_BYTE] & 0x08) != 0);
//		ROS_INFO("mix:%x", buf[REC_MIX_BYTE]);
//		if (water_tank.getStatus())
//			ROS_INFO("Water tank~~~~~~~~~~~~~~~~~~ :D");
		sensor.water_tank = water_tank.getSwingMotorEquipmentStatus();

		// For charger device.
		charger.setChargeStatus((buf[REC_MIX_BYTE] >> 4) & 0x07);
		sensor.charge_status = charger.getChargeStatus();
//		printf("Charge status:%d.\n", charger.getChargeStatus());

		// For sleep status.
		serial.isMainBoardSleep((buf[REC_MIX_BYTE] & 0x80) == 0);
		sensor.main_board_sleep_status = serial.isMainBoardSleep();

		// For battery device.
		battery.setVoltage(buf[REC_BATTERY] * 10);
//		printf("Battery:%.1fv.\n", static_cast<float>(buf[REC_BATTERY] / 10.0));
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

		// For wheel encoder count.
		wheel.setLeftEncoderCnt(buf[REC_LEFT_WHEEL_ENCODER]);
		sensor.left_wheel_encoder = wheel.getLeftEncoderCnt();
		wheel.setRightEncoderCnt(buf[REC_RIGHT_WHEEL_ENCODER]);
		sensor.right_wheel_encoder = wheel.getRightEncoderCnt();


		// For appointment and set time

		if((buf[REC_APPOINTMENT_TIME] >= 0x80) && p_mode != nullptr && p_mode->allowRemoteUpdatePlan())
		{
			robot_timer.setRealTimeOffsetByRemote(buf[REC_REALTIME_H] << 8 | buf[REC_REALTIME_L]);
			appmt_obj.set(buf[REC_APPOINTMENT_TIME]);
		}
		sensor.realtime = buf[REC_REALTIME_H]<<8 | buf[REC_REALTIME_L];
		sensor.appointment = buf[REC_APPOINTMENT_TIME];
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
		boost::mutex::scoped_lock lock(odom_mutex_);
//		odom.setMovingSpeed(static_cast<float>((wheel.getLeftWheelActualSpeed() + wheel.getRightWheelActualSpeed()) / 2.0));
		odom.setOriginRadian(degree_to_radian(gyro.getAngleY()));
		odom.setAngleSpeed(gyro.getAngleV());
		cur_time = ros::Time::now();
		double angle_rad, dt;
		angle_rad = odom.getRadian();
		dt = (cur_time - last_time).toSec();
		last_time = cur_time;
//		gyro.setAngleR(gyro.calAngleR1OrderFilter(0.5, dt));//fusion of 1 order filter
		gyro.setAngleR(gyro.calAngleRKalmanFilter(dt));//fusion of kalman filter
		odom.setMovingSpeed(static_cast<float>((wheel.getLeftEncoderCnt() + wheel.getRightEncoderCnt()) *
											   WHEEL_ENCODER_TO_MILLIMETER / 1000 / 2.0 / dt));
		if(!lidar.isRobotSlip()){
//			odom.setOriginX(static_cast<float>(odom.getOriginX() + (odom.getMovingSpeed() * cos(angle_rad)) * dt));
//			odom.setOriginY(static_cast<float>(odom.getOriginY() + (odom.getMovingSpeed() * sin(angle_rad)) * dt));
			odom.setOriginX(static_cast<float>(odom.getOriginX() + ((wheel.getLeftEncoderCnt() + wheel.getRightEncoderCnt())
																												* WHEEL_ENCODER_TO_MILLIMETER * cos(angle_rad) / 1000 / 2)));
			odom.setOriginY(static_cast<float>(odom.getOriginY() + ((wheel.getLeftEncoderCnt() + wheel.getRightEncoderCnt())
																												* WHEEL_ENCODER_TO_MILLIMETER * sin(angle_rad) / 1000 / 2)));
			is_first_slip = true;
		} else if (is_first_slip){
			odom.setOriginX(static_cast<float>(odom.getOriginX() - (0.30 * cos(angle_rad)) * 0.8));
			odom.setOriginY(static_cast<float>(odom.getOriginY() - (0.30 * sin(angle_rad)) * 0.8));
			is_first_slip = false;
		}
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
		if (checkTilt()){
			gyro.setTiltCheckingStatus(1);
			beeper.debugBeep(VALID);
		} else {
			gyro.setTiltCheckingStatus(0);
		}

		// Check lidar stuck
		if (checkLidarStuck()) {
//			ROS_INFO("lidar stuck");
			ev.lidar_stuck = true;
		} else {
//			ROS_INFO("lidar good");
			ev.lidar_stuck = false;
		}
		// Dynamic adjust obs
		obs.DynamicAdjust(OBS_adjust_count);

		// Check for whether robot should publish this frame of scan.
		bool is_pub_;
		if (p_mode != nullptr)
		{
			if (p_mode->sp_action_ != nullptr && p_mode->isInitState())
				is_pub_ = 1;
			else
				is_pub_ =
						!(fabs(wheel.getLeftWheelActualSpeed() - wheel.getRightWheelActualSpeed()) > 0.1
						  || (wheel.getLeftWheelActualSpeed() * wheel.getRightWheelActualSpeed() < 0)
						  || bumper.getStatus()
						  /*|| gyro.getTiltCheckingStatus()*/
						  || abs(wheel.getLeftSpeedAfterPid() - wheel.getRightSpeedAfterPid()) > 100
						  || wheel.getLeftSpeedAfterPid() * wheel.getRightSpeedAfterPid() < 0);
		}
		else
		{
			is_pub_ = 1;
		}
		pubScanCtrl(is_pub_);
//		scan_ctrl_pub_.publish(scan_ctrl_);

	}//end while
	pthread_cond_broadcast(&serial_data_ready_cond);
	event_manager_thread_kill = true;
	printf("%s,%d,exit\n",__FUNCTION__,__LINE__);
}

void robot::core_thread_cb()
{
	recei_thread_enable = true;
	r16_work_mode_ = getTestMode();
//	r16_work_mode_ = LIFE_TEST_MODE;
	ROS_INFO("%s %d: work mode: %d", __FUNCTION__, __LINE__, r16_work_mode_);
	//s_wifi.taskPushBack(S_Wifi::ACT::ACT_SLEEP);
	switch (r16_work_mode_)
	{
		case FUNC_SERIAL_TEST_MODE:
		{
			recei_thread_enable = false;
			sleep(1); // Make sure recieve thread is hung up, this time interval should be the select timeout of serial.
			x900_functional_test(serial_port_, baud_rate_, lidar_bumper_dev_);
			break;
		}
		case DESK_TEST_CURRENT_MODE:
		case DESK_TEST_MOVEMENT_MODE:
		case DESK_TEST_WRITE_BASELINE_MODE:
		case GYRO_TEST_MODE:
		case LIFE_TEST_MODE:
		case WATER_TANK_TEST_MODE:
		case R16_AND_LIDAR_TEST_MODE:
		case BUMPER_TEST_MODE:
//		case WORK_MODE: // For debug
//		case NORMAL_SLEEP_MODE: // For debug
		{
			runTestMode();
			break;
		}
		default: //case WORK_MODE:
		{
			runWorkMode();
			break;
		}
	}

	printf("%s %d: core thread exit.\n", __FUNCTION__, __LINE__);
}

void robot::runTestMode()
{
	auto serial_send_routine = new boost::thread(boost::bind(&Serial::send_routine_cb, &serial));
	send_thread_enable = true;

	if (r16_work_mode_ == NORMAL_SLEEP_MODE || r16_work_mode_ == WORK_MODE ||
		r16_work_mode_ == WATER_TANK_TEST_MODE || r16_work_mode_ == BUMPER_TEST_MODE) // todo: for debug
	{
		auto robotbase_routine = new boost::thread(boost::bind(&robot::robotbase_routine_cb, this));
		robotbase_thread_enable = true;
	}

	if (r16_work_mode_ != R16_AND_LIDAR_TEST_MODE)
	{
		if (bumper.lidarBumperInit(lidar_bumper_dev_.c_str()) == -1)
			ROS_ERROR(" lidar bumper open fail!");
	}

	key_led.setMode(LED_STEADY, LED_ORANGE);
	p_mode.reset(new CleanModeTest(r16_work_mode_));
	p_mode->run();
	g_pp_shutdown = true;
	printf("%s %d: Exit.\n", __FUNCTION__, __LINE__);
}

void robot::runWorkMode()
{
	//s_wifi.taskPushBack(S_Wifi::ACT::ACT_RESUME);
	auto serial_send_routine = new boost::thread(boost::bind(&Serial::send_routine_cb, &serial));
	send_thread_enable = true;

	auto robotbase_routine = new boost::thread(boost::bind(&robot::robotbase_routine_cb, this));
	robotbase_thread_enable = true;

	ROS_INFO("Waiting for robot sensor ready.");
	while (!isSensorReady())
		usleep(1000);

	ROS_INFO("Robot sensor ready.");

	if (bumper.lidarBumperInit(lidar_bumper_dev_.c_str()) == -1)
		ROS_ERROR(" lidar bumper open fail!");

	if (charger.isOnStub() || charger.isDirected())
		p_mode.reset(new ModeCharge());
	else if (battery.isReadyToClean())
	{
		speaker.play(VOICE_PLEASE_START_CLEANING, false);
		p_mode.reset(new ModeIdle());
	}
	else
	{
		speaker.play(VOICE_BATTERY_LOW, false);
		p_mode.reset(new ModeIdle());
	}

	s_wifi.taskPushBack(S_Wifi::ACT::ACT_QUERY_NTP);

	while (ros::ok())
	{
//		ROS_INFO("%s %d: %x", __FUNCTION__, __LINE__, p_mode);
		p_mode->run();

		if (core_thread_kill)
			break;

		boost::mutex::scoped_lock lock(mode_mutex_);
		auto next_mode = p_mode->getNextMode();
		p_mode.reset();
		ROS_INFO("%s %d: Previous mode should finish destructing now?", __FUNCTION__, __LINE__);

		auto robot_up_hour = static_cast<uint16_t>(robot_timer.getRobotUpTime() / 3600);
		if (1 || robot_up_hour > robot_up_hour_)
		{
			robot_up_hour_ = robot_up_hour;
			updateConsumableStatus();
		}

		p_mode.reset(getNextMode(next_mode));
		robot_work_mode_ = next_mode;
	}
	g_pp_shutdown = true;
	printf("%s %d: Exit.\n", __FUNCTION__, __LINE__);
}

uint8_t robot::getTestMode(void)
{
	uint8_t buf[REC_LEN];

	/*--------data extrict from serial com--------*/
	ROS_ERROR_COND(pthread_mutex_lock(&recev_lock) != 0, "robotbase pthread receive lock fail");
	ROS_ERROR_COND(pthread_cond_wait(&recev_cond, &recev_lock) != 0, "robotbase pthread receive cond wait fail");
	memcpy(buf, serial.receive_stream, sizeof(uint8_t) * REC_LEN);
	ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock) != 0, "robotbase pthread receive unlock fail");
//	debugReceivedStream(buf);

	return buf[REC_WORK_MODE];
}

void robot::robotOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
	tf::StampedTransform transform{};
//	tf::Vector3 tmp_pos{odom.getOriginX(),odom.getOriginY(),0};
	tf::Vector3 tmp_pos(robot_pos);
	double	tmp_rad{odom.getRadian()};
	if (getBaselinkFrameType() == SLAM_POSITION_SLAM_ANGLE || getBaselinkFrameType() == SLAM_POSITION_ODOM_ANGLE) {
		if (slam.isMapReady()) {
			try {
				robot_tf_->lookupTransform("/map", "/base_link", ros::Time(0), transform);
//				ROS_ERROR("get transform!!!!!!!!!!!!!!!!!");
				slam_pos = transform.getOrigin();
				if(getBaselinkFrameType() == SLAM_POSITION_SLAM_ANGLE)
				{
					slam_rad  = tf::getYaw(transform.getRotation()) ;
//					auto diff = slam_rad - odom.getRadian();
//					if(diff > degree_to_radian(1))
//					{
//						odom.setRadianOffset(slam_rad - odom.getOriginRadian());
//						beeper.debugBeep();
//					}
				}
				else
					slam_rad = odom.getRadian();
				tmp_pos = slam_pos;
				tmp_rad = slam_rad;

			} catch (tf::TransformException e) {
				ROS_WARN("%s %d: Failed to compute map transform, skipping scan (%s)", __FUNCTION__, __LINE__, e.what());
			}

			if (!isTfReady()) {
				ROS_INFO("%s %d: TF ready.", __FUNCTION__, __LINE__);
				setTfReady(true);
			}
//			ROS_WARN("tmp_pos(%f,%f),tmp_rad(%f)", tmp_pos.x(), tmp_pos.y(), tmp_rad);
		}
	}else {
		tmp_pos = {odom.getX(), odom.getY(),0};
	}
//	ROS_INFO("tmp_pos(%f,%f),tmp_rad(%f)", tmp_pos.x(), tmp_pos.y(), tmp_rad);
	robot_pos = tmp_pos;
	robot_rad = tmp_rad;
	setRobotActualSpeed();
	odomPublish(robot_pos, robot_rad);
}

void robot::odomPublish(const tf::Vector3& robot_pos, double robot_radian_)
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
	boost::mutex::scoped_lock lock(odom_mutex_);
	odom.setOriginX(0);
	odom.setOriginY(0);
}
//

void robot::resetCorrection()
{
	slam_pos = {};
	robot_rad = {};
}

void robot::obsAdjustCount(int count)
{
#ifdef OBS_DYNAMIC
	OBS_adjust_count = count;
#endif
}

bool robot::pubScanCtrl(bool is_pub, bool is_force_pub) {
//	ROS_INFO("is_pub = %d", is_pub);
	if (!is_locked_scan_ctrl_ || is_force_pub) {
		scan_ctrl_.allow_publishing = is_pub;
		scan_ctrl_pub_.publish(scan_ctrl_);
		return true;
	} else {
//		INFO_YELLOW("scan_ctrl_ was locked, pub scan_ctrl failed!");
		return false;
	}
}

void robot::lockScanCtrl(void) {
	is_locked_scan_ctrl_ = true;
}

void robot::unlockScanCtrl(void) {
	is_locked_scan_ctrl_ = false;
}

void robot::publishCtrlStream(void)
{
	pp::x900ctrl ctrl_stream;

	ctrl_stream.left_wheel_speed = wheel.getLeftSpeedInStream();
	ctrl_stream.right_wheel_speed = wheel.getRightSpeedInStream();

	ctrl_stream.vacuum_PWM = serial.getSendData(CTL_VACCUM_PWR);
	ctrl_stream.left_brush_PWM = serial.getSendData(CTL_BRUSH_LEFT);
	ctrl_stream.right_brush_PWM = serial.getSendData(CTL_BRUSH_RIGHT);
	ctrl_stream.main_brush_PWM = serial.getSendData(CTL_BRUSH_MAIN);

	ctrl_stream.beeper_sound_code = serial.getSendData(CTL_BEEPER);
	ctrl_stream.main_board_mode = serial.getSendData(CTL_WORK_MODE);
	ctrl_stream.charge_control = serial.getSendData(CTL_CHARGER);

	ctrl_stream.led_red_brightness = serial.getSendData(CTL_LED_RED);
	ctrl_stream.led_green_brightness = serial.getSendData(CTL_LED_GREEN);
	ctrl_stream.wifi_led = static_cast<unsigned char>(serial.getSendData(CTL_MIX) & 0x01);

	ctrl_stream.vacuum_exception_ctrl = static_cast<unsigned char>((serial.getSendData(CTL_MIX) >> 1) & 0x01);

	ctrl_stream.gyro_dynamic_ctrl = static_cast<unsigned char>((serial.getSendData(CTL_MIX) >> 2) & 0x01);
	ctrl_stream.gyro_switch = static_cast<unsigned char>((serial.getSendData(CTL_MIX) >> 3) & 0x01);

	ctrl_stream.obs_switch = static_cast<unsigned char>((serial.getSendData(CTL_MIX) >> 4) & 0x01);

	ctrl_stream.swing_motor_pwm = static_cast<unsigned char>(serial.getSendData(CTL_WATER_TANK) & 0x7F);
	ctrl_stream.pump_switch = static_cast<unsigned char>((serial.getSendData(CTL_WATER_TANK) >> 7) & 0x01);

	ctrl_stream.infrared_display_switch = serial.getSendData(CTL_IR_CTRL) >> 6;
	ctrl_stream.infrared_display_step = static_cast<unsigned char>(serial.getSendData(CTL_IR_CTRL) & 0x3F);
	ctrl_stream.infrared_display_content = serial.getSendData(CTL_IR_CONTENT_H) << 8 | serial.getSendData(CTL_IR_CONTENT_L);
	ctrl_stream.infrared_display_error_code = serial.getSendData(CTL_IR_ERROR_CODE_H) << 8 | serial.getSendData(CTL_IR_ERROR_CODE_L);

	ctrl_stream.appointment_bytes = (serial.getSendData(CTL_APPOINTMENT_H) <<8) & serial.getSendData(CTL_APPOINTMENT_L);

	ctrl_stream.key_validation = serial.getSendData(CTL_KEY_VALIDATION);
	ctrl_stream.crc = serial.getSendData(CTL_CRC);

	x900_ctrl_pub_.publish(ctrl_stream);
}

void robot::updateRobotPositionForTest()
{
	robot_pos.setX(odom.getX());
	robot_pos.setY(odom.getY());
	robot_rad = ranged_radian(odom.getRadian());
}

bool robot::checkTilt() {
	if (!gyro.isTiltCheckingEnable()) {
		angle_tilt_time_ = 0;
		wheel_tilt_time_= 0;
		return false;
	}
	auto angle_triggered = gyro.getAngleR() > ANGLE_LIMIT;
	auto wheel_cliff_triggered = wheel.getLeftWheelCliffStatus() || wheel.getRightWheelCliffStatus();

	if(!angle_triggered)
		angle_tilt_time_ = 0;
	if(!wheel_cliff_triggered)
		wheel_tilt_time_= 0;
	if(!angle_triggered && !wheel_cliff_triggered)
		return false;

	//For angle triggered
	if(angle_triggered) {
		angle_tilt_time_ = angle_tilt_time_ == 0 ? ros::Time::now().toSec() : angle_tilt_time_;
		auto ret = ros::Time::now().toSec() - angle_tilt_time_ > ANGLE_TIME_LIMIT;
		ROS_WARN_COND(ret,"%s,%d,time_now:%lf,angle_tilt_time_:%lf",__FUNCTION__,__LINE__,ros::Time::now().toSec(),angle_tilt_time_);
		return ret;
	}
	//For wheel_cliff triggered
	if(wheel_cliff_triggered) {
		wheel_tilt_time_ = wheel_tilt_time_ == 0 ? ros::Time::now().toSec() : wheel_tilt_time_;
		auto ret = ros::Time::now().toSec() - wheel_tilt_time_ > WHEEL_CLIFF_TIME_LIMIT;
		ROS_WARN_COND(ret,"%s,%d,time_now:%lf,wheel_tilt_time_:%lf",__FUNCTION__,__LINE__,ros::Time::now().toSec(),wheel_tilt_time_);
		return ret;
	}
}

bool robot::checkTiltToSlip() {
	auto angle = std::fabs(gyro.getAngleR());
//	ROS_WARN("angle = %f", angle);
	if (angle < ANGLE_LIMIT_TO_SLIP) {
		is_first_tilt_to_slip_ = true;
		return false;
	}

	if (is_first_tilt_to_slip_) {
		is_first_tilt_to_slip_ = false;
		tilt_time_to_slip_ = ros::Time::now().toSec();
	}

	return (ros::Time::now().toSec() - tilt_time_to_slip_) > TIME_LIMIT_TO_SLIP ? true : false;
}

bool robot::checkLidarStuck() {
	if (!lidar.getLidarStuckCheckingEnable()) {
		lidar_is_covered_time_ = 0;
		return false;
	}
	auto is_stuck = !lidar.lidarCheckFresh(3,2);
	auto is_covered = lidar.checkLidarBeCovered();
//	ROS_INFO("is_stuck(%d), is_covered(%d)", is_stuck, is_covered);
	if (is_stuck) {
		lidar_is_covered_time_ = 0;
		return true;
	}
	if (!is_covered) {
		lidar_is_covered_time_ = 0;
		return false;
	}
	if (lidar_is_covered_time_ == 0) {
		lidar_is_covered_time_ = ros::Time::now().toSec();
	}
//	ROS_INFO("ros::Time::now().toSec() - lidar_is_covered_time_ = %lf", ros::Time::now().toSec() - lidar_is_covered_time_);
	if (ros::Time::now().toSec() - lidar_is_covered_time_ > 3) {
		return true;
	} else {
		return false;
	}
}

bool robot::getCleanMap(GridMap& map)
{
	bool ret = false;
	boost::mutex::scoped_lock lock(mode_mutex_);
	if (getRobotWorkMode() == Mode::cm_navigation)
	{
		auto mode = boost::dynamic_pointer_cast<ACleanMode>(p_mode);
		if (mode->isStateClean())
		{
			map = mode->clean_map_;
			ret = true;
		}
	}
	return ret;
}

void robot::wifiSetWaterTank()
{
	boost::mutex::scoped_lock lock(mode_mutex_);
	p_mode->wifiSetWaterTank();
}

void robot::wifiSetVacuum()
{
	boost::mutex::scoped_lock lock(mode_mutex_);
	p_mode->setVacuum();
}

void robot::loadConsumableStatus()
{ ROS_INFO("%s %d: Load consumable status.", __FUNCTION__, __LINE__);

	if (access(consumable_file.c_str(), F_OK) == -1)
	{
		// If file not exist, check if back up file exist.
		if (access(consumable_backup_file.c_str(), F_OK) == -1)
			return;
		else
		{
			//Restore from backup file.
			std::string cmd = "cp " + consumable_backup_file + " " + consumable_file;
			system(cmd.c_str());
			ROS_INFO("%s %d: Resume from backup file.", __FUNCTION__, __LINE__);
		}
	}

	bool file_error = false;
	FILE *f_read = fopen(consumable_file.c_str(), "r");
	if (f_read == nullptr)
	{
		ROS_ERROR("%s %d: Open %s error.", __FUNCTION__, __LINE__, consumable_file.c_str());
		file_error = true;
	}
	else
	{
		if (fscanf(f_read, "Side brush: %d\n", &side_brush_time_) != 1)
		{
			ROS_ERROR("%s %d: Side brush data error! Reset to 0.", __FUNCTION__, __LINE__);
			side_brush_time_ = 0;
			file_error = true;
		}
		ROS_INFO("%s %d: Read side brush: %d.", __FUNCTION__, __LINE__, side_brush_time_);
		if (fscanf(f_read, "Main brush: %d\n", &main_brush_time_) != 1)
		{
			ROS_ERROR("%s %d: Main brush data error! Reset to 0.", __FUNCTION__, __LINE__);
			main_brush_time_ = 0;
			file_error = true;
		}
		ROS_INFO("%s %d: Read main brush: %d.", __FUNCTION__, __LINE__, main_brush_time_);
		if (fscanf(f_read, "Filter: %d\n", &filter_time_) != 1)
		{
			ROS_ERROR("%s %d: Filter data error! Reset to 0.", __FUNCTION__, __LINE__);
			filter_time_ = 0;
			file_error = true;
		}
		ROS_INFO("%s %d: Read main brush: %d.", __FUNCTION__, __LINE__, filter_time_);
		fclose(f_read);
		ROS_INFO("%s %d: Read data succeeded.", __FUNCTION__, __LINE__);
	}

	if (file_error)
	{
		std::string cmd = "rm -f " + consumable_file + " " + consumable_backup_file;
		system(cmd.c_str());
		ROS_ERROR("%s %d: Delete consumable file due to file error.", __FUNCTION__, __LINE__);
	}
}

void robot::updateConsumableStatus()
{
	auto additional_side_brush_time_sec = brush.getSideBrushTime();
	ROS_INFO("%s %d: Additional side brush: %ds.", __FUNCTION__, __LINE__, additional_side_brush_time_sec);
	brush.resetSideBurshTime();
	auto side_brush_time = additional_side_brush_time_sec + side_brush_time_;

	auto additional_main_brush_time_sec = brush.getMainBrushTime();
	ROS_INFO("%s %d: Additional main brush: %ds.", __FUNCTION__, __LINE__, additional_main_brush_time_sec);
	brush.resetMainBrushTime();
	auto main_brush_time = additional_main_brush_time_sec + main_brush_time_;

	auto additional_filter_time_sec = vacuum.getFilterTime();
	ROS_INFO("%s %d: Additional filter: %ds.", __FUNCTION__, __LINE__, additional_filter_time_sec);
	vacuum.resetFilterTime();
	auto filter_time = additional_filter_time_sec + filter_time_;

	if (access(consumable_file.c_str(), F_OK) != -1)
	{
		// If file exist, make it a back up file.
		std::string cmd = "mv " + consumable_file + " " + consumable_backup_file;
		system(cmd.c_str());
		ROS_INFO("%s %d: Backup for %s.", __FUNCTION__, __LINE__, consumable_file.c_str());
	}

	FILE *f_write = fopen(consumable_file.c_str(), "w");
	if (f_write == nullptr)
		ROS_ERROR("%s %d: Open %s error.", __FUNCTION__, __LINE__, consumable_file.c_str());
	else
	{
		ROS_INFO("%s %d: Start writing data to %s.", __FUNCTION__, __LINE__, consumable_file.c_str());
		fprintf(f_write, "Side brush: %d\n", side_brush_time);
		ROS_INFO("%s %d: Write side brush: %d.", __FUNCTION__, __LINE__, side_brush_time);
		side_brush_time_ = side_brush_time;
		fprintf(f_write, "Main brush: %d\n", main_brush_time);
		ROS_INFO("%s %d: Main brush: %d.", __FUNCTION__, __LINE__, main_brush_time);
		main_brush_time_ = main_brush_time;
		fprintf(f_write, "Filter: %d\n", filter_time);
		ROS_INFO("%s %d: Filter: %d.", __FUNCTION__, __LINE__, filter_time);
		filter_time_ = filter_time;
		fclose(f_write);
		ROS_INFO("%s %d: Write data succeeded.", __FUNCTION__, __LINE__);
	}
}

void robot::updateCleanRecord(const uint32_t &time, const uint16_t &clean_time, const float &clean_area,
							  GridMap &clean_map)
{
	auto clean_area_int = static_cast<uint16_t>(clean_area + 1);
	boost::mutex::scoped_lock lock(last_clean_record_mutex_);
	last_clean_record_.time = time;
	last_clean_record_.clean_time = clean_time;
	last_clean_record_.clean_area = clean_area_int;
	last_clean_record_.clean_map.copy(clean_map);
}

void robot::getCleanRecord(uint32_t &time, uint16_t &clean_time, uint16_t &clean_area, GridMap &clean_map)
{
	boost::mutex::scoped_lock lock(last_clean_record_mutex_);
	time = last_clean_record_.time;
	clean_time = last_clean_record_.clean_time;
	clean_area = last_clean_record_.clean_area;
	clean_map.copy(last_clean_record_.clean_map);
}

void robot::setRobotActualSpeed() {
	static auto time = ros::Time::now().toSec();
	static auto x = robot_pos.x();
	static auto y = robot_pos.y();
	static int invalid_count = 1;
	auto dis = sqrt(pow(x - robot_pos.x(),2) + pow(y - robot_pos.y(),2));
	auto speed = dis / (ros::Time::now().toSec() - time);
	speed = speed < ROBOT_MAX_SPEED ? speed : ROBOT_MAX_SPEED;
	auto isvalid_speed = fabs(speed - robot_actual_speed_) > 0.03 * invalid_count;
	if(isvalid_speed)
	{
		invalid_count++;
		return;
	}
	robot_actual_speed_ = speed;
//	ROS_INFO("speed: %lf, dis:%lf, delta_time:%lf, invalid_count:%d",
//	robot_actual_speed_,dis,ros::Time::now().toSec() - time,invalid_count - 1);
	invalid_count = 1;
	x = robot_pos.x();
	y = robot_pos.y();
	time = ros::Time::now().toSec();
}


//--------------------
static float xCount{}, yCount{};

Point_t getPosition(Baselink_Frame_Type type)
{
	if(type == ODOM_POSITION_ODOM_ANGLE)
		return {odom.getX(),odom.getY(),odom.getRadian()};

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
/*
Dir_t getReDir(Dir_t dir)
{
	if(isAny(dir))
		return dir;
	if(isPos(dir))
	{
		return dir+1;
	}
	else
		return dir-1;
}*/
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
//		case Mode::cm_test:
//			return new CleanModeTest();
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
