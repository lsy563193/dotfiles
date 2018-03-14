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
#include "std_srvs/Empty.h"


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
	robot_nh_.param<int>("baud_rate", baud_rate_, 115200);

	robot_nh_.param<std::string>("lidar_bumper_file", lidar_bumper_dev_, "/dev/input/event1");

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

	ROS_INFO("%s %d: robot init done!", __FUNCTION__, __LINE__);
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
		boost::mutex::scoped_lock lock(odom_mutex_);
		odom.setMovingSpeed(static_cast<float>((wheel.getLeftWheelActualSpeed() + wheel.getRightWheelActualSpeed()) / 2.0));
		odom.setRadian(degree_to_radian(gyro.getAngle()));
		odom.setAngleSpeed(gyro.getAngleV());
		cur_time = ros::Time::now();
		double angle_rad, dt;
		angle_rad = odom.getRadian();
		dt = (cur_time - last_time).toSec();
		last_time = cur_time;
		if(!lidar.isRobotSlip()){
			odom.setX(static_cast<float>(odom.getX() + (odom.getMovingSpeed() * cos(angle_rad)) * dt));
			odom.setY(static_cast<float>(odom.getY() + (odom.getMovingSpeed() * sin(angle_rad)) * dt));
			is_first_slip = true;
		} else if (is_first_slip){
			odom.setX(static_cast<float>(odom.getX() - (0.30 * cos(angle_rad)) * 0.8));
			odom.setY(static_cast<float>(odom.getY() - (0.30 * sin(angle_rad)) * 0.8));
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
//		gyro.checkTilt();
#if GYRO_DYNAMIC_ADJUSTMENT
#endif
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
	ROS_ERROR("%s,%d,exit",__FUNCTION__,__LINE__);
}

void robot::core_thread_cb()
{
	recei_thread_enable = true;
	r16_work_mode_ = getTestMode();
	ROS_INFO("%s %d: work mode: %d", __FUNCTION__, __LINE__, r16_work_mode_);

	switch (r16_work_mode_)
	{
		case SERIAL_TEST_MODE:
		{
			recei_thread_enable = false;
			sleep(1); // Make sure recieve thread is hung up, this time interval should be the select timeout of serial.
			x900_functional_test(serial_port_, baud_rate_, lidar_bumper_dev_);
			break;
		}
		case DESK_TEST_CURRENT_MODE:
		case DESK_TEST_MOVEMENT_MODE:
		{
			auto serial_send_routine = new boost::thread(boost::bind(&Serial::send_routine_cb, &serial));
			send_thread_enable = true;
			if (bumper.lidarBumperInit(lidar_bumper_dev_.c_str()) == -1)
				ROS_ERROR(" lidar bumper open fail!");

			p_mode.reset(new CleanModeDeskTest());
			p_mode->run();
			break;
		}
		default: //case R16_NORMAL_MODE:
		{
			auto serial_send_routine = new boost::thread(boost::bind(&Serial::send_routine_cb, &serial));
			send_thread_enable = true;

			auto robotbase_routine = new boost::thread(boost::bind(&robot::robotbase_routine_cb, this));
			robotbase_thread_enable = true;

			ROS_INFO("Waiting for robot sensor ready.");
			while (!isSensorReady())
			{
				usleep(1000);
			}
			ROS_INFO("Robot sensor ready.");

			if (bumper.lidarBumperInit(lidar_bumper_dev_.c_str()) == -1)
				ROS_ERROR(" lidar bumper open fail!");

			if (charger.isOnStub() || charger.isDirected())
				p_mode.reset(new ModeCharge());
			else
			{
				speaker.play(VOICE_PLEASE_START_CLEANING, false);
				p_mode.reset(new ModeIdle());
			}

			while (ros::ok())
			{
//				ROS_INFO("%s %d: %x", __FUNCTION__, __LINE__, p_mode);
				p_mode->run();

				if (core_thread_kill)
					break;

				auto next_mode = p_mode->getNextMode();
				p_mode.reset();
//				ROS_INFO("%s %d: %x", __FUNCTION__, __LINE__, p_mode);
				p_mode.reset(getNextMode(next_mode));
//				ROS_INFO("%s %d: %x", __FUNCTION__, __LINE__, p_mode);
			}
			g_pp_shutdown = true;
			ROS_ERROR("%s,%d,exit", __FUNCTION__, __LINE__);
			break;
		}
	}

}

robot *robot::instance()
{
	extern robot* robot_instance;
	return robot_instance;
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

	return buf[REC_R16_WORK_MODE];
}

void robot::robotOdomCb(const nav_msgs::Odometry::ConstPtr &msg)
{
	tf::StampedTransform transform{};
//	tf::Vector3 tmp_pos{odom.getX(),odom.getY(),0};
	tf::Vector3 tmp_pos(robot_pos);
	double	tmp_rad{odom.getRadian()};
	if (getBaselinkFrameType() == SLAM_POSITION_SLAM_ANGLE || getBaselinkFrameType() == SLAM_POSITION_ODOM_ANGLE) {
		if (slam.isMapReady()) {
			try {
				robot_tf_->lookupTransform("/map", "/base_link", ros::Time(0), transform);
//				ROS_ERROR("get transform!!!!!!!!!!!!!!!!!");
				slam_pos = transform.getOrigin();
				slam_rad  = (getBaselinkFrameType() == SLAM_POSITION_SLAM_ANGLE) ? tf::getYaw(transform.getRotation()) : 0;
				tmp_pos = slam_pos;
				tmp_rad = slam_rad;
//				scaleCorrectionPos(tmp_pos, tmp_rad);
//				ROS_WARN("tmp_pos(%f,%f),tmp_rad(%f)", tmp_pos.x(), tmp_pos.y(), tmp_rad);
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
		tmp_pos = {odom.getX(),odom.getY(),0};
	}
//	ROS_INFO("tmp_pos(%f,%f),tmp_rad(%f)", tmp_pos.x(), tmp_pos.y(), tmp_rad);
	robot_pos = tmp_pos;
	robot_rad = tmp_rad;
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
	odom.setX(0);
	odom.setY(0);
}
//

void robot::resetCorrection()
{
	slam_pos = {};
	robot_rad = {};
	slam_pos = {};
	robot_rad = {};
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
	for (int i = 0; i < REC_LEN; i++)
		printf("%02d ", i);
	printf("\n");

	for (int i = 0; i < REC_LEN; i++)
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
	ctrl_stream.main_board_mode = serial.getSendData(CTL_MAIN_BOARD_MODE);
	ctrl_stream.charge_control = serial.getSendData(CTL_CHARGER);

	ctrl_stream.led_red_brightness = serial.getSendData(CTL_LED_RED);
	ctrl_stream.led_green_brightness = serial.getSendData(CTL_LED_GREEN);
	ctrl_stream.wifi_led = static_cast<unsigned char>(serial.getSendData(CTL_MIX) & 0x01);

	ctrl_stream.vacuum_exception_ctrl = static_cast<unsigned char>((serial.getSendData(CTL_MIX) >> 1) & 0x01);

	ctrl_stream.gyro_dynamic_ctrl = static_cast<unsigned char>((serial.getSendData(CTL_MIX) >> 2) & 0x01);
	ctrl_stream.gyro_switch = static_cast<unsigned char>((serial.getSendData(CTL_MIX) >> 3) & 0x01);

	ctrl_stream.key_validation = serial.getSendData(CTL_KEY_VALIDATION);
	ctrl_stream.crc = serial.getSendData(CTL_CRC);

	x900_ctrl_pub_.publish(ctrl_stream);
}

void robot::updateRobotPositionForDeskTest()
{
	robot_pos.setX(odom.getX());
	robot_pos.setY(odom.getY());
	robot_rad = ranged_radian(odom.getRadian());
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
		case Mode::cm_test:
			return new CleanModeDeskTest();
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
