#include "ros/ros.h"
#include "dev.h"
#include <std_msgs/String.h>
#include <pp/x900sensor.h>
#include <tf/transform_broadcaster.h>
#include "robotbase.h"


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
uint16_t robotbase_led_cnt_for_switch = 0;
uint16_t live_led_cnt_for_switch = 0;

// Lock for odom coordinate
boost::mutex odom_mutex;

void debug_received_stream()
{
	ROS_INFO("%s %d: Received stream:", __FUNCTION__, __LINE__);
	for (int i = 0; i < RECEI_LEN; i++)
		printf("%02d ", i);
	printf("\n");

	for (int i = 0; i < RECEI_LEN; i++)
		printf("%02x ", serial.receive_stream[i]);
	printf("\n");
}

void debug_send_stream(uint8_t *buf)
{
	ROS_INFO("%s %d: Send stream:", __FUNCTION__, __LINE__);
	for (int i = 0; i < SEND_LEN; i++)
		printf("%02x ", *(buf + i));
	printf("\n");
}

bool is_robotbase_stop(void)
{
	return robotbase_thread_stop ? true : false;
}

void robotbase_deinit(void)
{
	if (is_robotbase_init) {
		is_robotbase_init = false;
		robotbase_thread_stop = true;
		ROS_INFO("%s,%d,shutdown robotbase power",__FUNCTION__,__LINE__);
		led.set_mode(LED_STEADY, LED_OFF);
		serial.setSendData(CTL_BEEPER, 0x00);
		gyro.setOff();
		wheel.stop();
		brush.stop();
		vacuum.stop();
		serial.setCleanMode(POWER_DOWN);
		usleep(40000);
		send_stream_thread = false;
		usleep(40000);
		serial.close();
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
	boost::mutex::scoped_lock(g_send_stream_mutex);
	for (int i = 0; i < SEND_LEN; i++) {
		if (i != CTL_LED_GREEN)
			serial.setSendData(i, 0x00);
		else
			serial.setSendData(i, 0x64);
	}
	serial.setSendData(0, 0xaa);
	serial.setSendData(1, 0x55);
	serial.setSendData(SEND_LEN - 2, 0xcc);
	serial.setSendData(SEND_LEN - 1, 0x33);

	serial.setCleanMode(POWER_ACTIVE);
	uint8_t buf[SEND_LEN];
	memcpy(buf, serial.send_stream, sizeof(uint8_t) * SEND_LEN);
	uint8_t crc;
	crc = serial.calc_buf_crc8(buf, SEND_LEN - 3);
	serial.setSendData(SEND_LEN - 3, crc);
}


void robotbase_routine_cb()
{
	ROS_INFO("robotbase,\033[32m%s\033[0m,%d is up.",__FUNCTION__,__LINE__);

	ros::Rate	r(_RATE);
	ros::Time	cur_time, last_time;

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
		if(pthread_mutex_lock(&recev_lock)!=0)ROS_ERROR("robotbase pthread receive lock fail");
		if(pthread_cond_wait(&recev_cond,&recev_lock)!=0)ROS_ERROR("robotbase pthread receive cond wait fail");
		if(pthread_mutex_unlock(&recev_lock)!=0)ROS_WARN("robotbase pthread receive unlock fail");
		//ros::spinOnce();

		boost::mutex::scoped_lock(odom_mutex);

		pp::x900sensor sensor;

		// For wheel device.
		wheel.setLeftWheelActualSpeed(static_cast<float>(static_cast<int16_t>((serial.receive_stream[REC_WHEEL_L_SPEED_H] << 8) | serial.receive_stream[REC_WHEEL_L_SPEED_L]) / 1000.0));
		wheel.setRightWheelActualSpeed(static_cast<float>(static_cast<int16_t>((serial.receive_stream[REC_WHEEL_R_SPEED_H] << 8) | serial.receive_stream[REC_WHEEL_R_SPEED_L]) / 1000.0));
		sensor.left_wheel_speed = wheel.getLeftWheelActualSpeed();
		sensor.right_wheel_speed = wheel.getRightWheelActualSpeed();

		wheel.setLeftWheelCliffStatus((serial.receive_stream[REC_WHEEL_CLIFF] & 0x02) != 0);
		wheel.setRightWheelCliffStatus((serial.receive_stream[REC_WHEEL_CLIFF] & 0x01) != 0);
		sensor.left_wheel_cliff = wheel.getLeftWheelCliffStatus();
		sensor.right_wheel_cliff = wheel.getRightWheelCliffStatus();

		// For gyro device.
		gyro.setCalibration(serial.receive_stream[REC_GYRO_CALIBRATION] != 0);
		sensor.gyro_calibration = gyro.getCalibration();

		gyro.setAngle(static_cast<float>(static_cast<int16_t>((serial.receive_stream[REC_ANGLE_H] << 8) | serial.receive_stream[REC_ANGLE_L]) / 100.0 * -1));
		sensor.angle = gyro.getAngle();
		gyro.setAngleV(static_cast<float>(static_cast<int16_t>((serial.receive_stream[REC_ANGLE_V_H] << 8) | serial.receive_stream[REC_ANGLE_V_H]) / 100.0 * -1));
		sensor.angle_v = gyro.getAngleV();

		if (gyro.getXAcc() == -1000)
			gyro.setXAcc(static_cast<int16_t>((serial.receive_stream[REC_XACC_H] << 8) | serial.receive_stream[REC_XACC_L]));
		else
			gyro.setXAcc(static_cast<int16_t>((static_cast<int16_t>((serial.receive_stream[REC_XACC_H] << 8)|serial.receive_stream[REC_XACC_L]) + gyro.getXAcc()) / 2));
		if (gyro.getYAcc() == -1000)
			gyro.setYAcc(static_cast<int16_t>((serial.receive_stream[REC_YACC_H] << 8) | serial.receive_stream[REC_YACC_L]));
		else
			gyro.setYAcc(static_cast<int16_t>((static_cast<int16_t>((serial.receive_stream[REC_YACC_H] << 8)|serial.receive_stream[REC_YACC_L]) + gyro.getYAcc()) / 2));
		if (gyro.getZAcc() == -1000)
			gyro.setZAcc(static_cast<int16_t>((serial.receive_stream[REC_ZACC_H] << 8) | serial.receive_stream[REC_ZACC_L]));
		else
			gyro.setZAcc(static_cast<int16_t>((static_cast<int16_t>((serial.receive_stream[REC_ZACC_H] << 8)|serial.receive_stream[REC_ZACC_L]) + gyro.getZAcc()) / 2));

		sensor.x_acc = gyro.getXAcc();//in mG
		sensor.y_acc = gyro.getYAcc();//in mG
		sensor.z_acc = gyro.getZAcc();//in mG

		// For wall sensor device.
		wall.setLeft((serial.receive_stream[REC_L_WALL_H] << 8)| serial.receive_stream[REC_L_WALL_L]);
		sensor.left_wall = wall.getLeft();
		wall.setRight((serial.receive_stream[REC_R_WALL_H] << 8)| serial.receive_stream[REC_R_WALL_L]);
		sensor.right_wall = wall.getRight();

		// For obs sensor device.
		obs.setLeft((serial.receive_stream[REC_L_OBS_H] << 8) | serial.receive_stream[REC_L_OBS_L]);
		sensor.left_obs = obs.getLeft();
		obs.setFront((serial.receive_stream[REC_F_OBS_H] << 8) | serial.receive_stream[REC_F_OBS_L]);
		sensor.front_obs = obs.getFront();
		obs.setRight((serial.receive_stream[REC_R_OBS_H] << 8) | serial.receive_stream[REC_R_OBS_L]);
		sensor.right_obs = obs.getRight();

		// For bumper device.
		bumper.setLeft((serial.receive_stream[REC_BUMPER_AND_CLIFF] & 0x20) != 0);
		bumper.setRight((serial.receive_stream[REC_BUMPER_AND_CLIFF] & 0x10) != 0);
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
		cliff.setLeft((serial.receive_stream[REC_BUMPER_AND_CLIFF] & 0x04) != 0);
		cliff.setFront((serial.receive_stream[REC_BUMPER_AND_CLIFF] & 0x02) != 0);
		cliff.setRight((serial.receive_stream[REC_BUMPER_AND_CLIFF] & 0x01) != 0);
		sensor.right_cliff = cliff.getRight();
		sensor.front_cliff = cliff.getFront();
		sensor.left_cliff = cliff.getLeft();

		// For remote device.
		remote.set(serial.receive_stream[REC_REMOTE]);
		sensor.remote = remote.get();
		if (remote.get() > 0)
			ROS_INFO("%s %d: Remote received:%d", __FUNCTION__, __LINE__, remote.get());

		// For rcon device.
		c_rcon.setStatus((serial.receive_stream[REC_RCON_CHARGER_4] << 24) | (serial.receive_stream[REC_RCON_CHARGER_3] << 16)
						 | (serial.receive_stream[REC_RCON_CHARGER_2] << 8) | serial.receive_stream[REC_RCON_CHARGER_1]);
		sensor.rcon = c_rcon.getStatus();

		// For virtual wall.
		sensor.virtual_wall = (serial.receive_stream[REC_VISUAL_WALL_H] << 8)| serial.receive_stream[REC_VISUAL_WALL_L];

		// For key device.
		key.eliminate_jitter((serial.receive_stream[REC_MIX_BYTE] & 0x01) != 0);
		sensor.key = key.getTriggerStatus();

		// For timer device.
		robot_timer.setPlanStatus(static_cast<uint8_t>((serial.receive_stream[REC_MIX_BYTE] >> 1) & 0x03));
		sensor.plan = robot_timer.getPlanStatus();

		// For water tank device.
		sensor.water_tank = (serial.receive_stream[REC_MIX_BYTE] & 0x08) != 0;

		// For charger device.
		charger.setChargeStatus((serial.receive_stream[REC_MIX_BYTE] >> 4) & 0x07);
		sensor.charge_status = charger.getChargeStatus();

		// For battery device.
		battery.setVoltage(serial.receive_stream[REC_BATTERY] * 10);
		sensor.battery = static_cast<float>(battery.getVoltage() / 100.0);

		// For vacuum device.
		vacuum.setExceptionResumeStatus(serial.receive_stream[REC_VACUUM_EXCEPTION_RESUME]);
		sensor.vacuum_exception_resume = vacuum.getExceptionResumeStatus();

		// For over current checking.
		vacuum.setOc((serial.receive_stream[REC_OC] & 0x01) != 0);
		sensor.vacuum_oc = vacuum.getOc();
		brush.setRightOc((serial.receive_stream[REC_OC] & 0x02) != 0);
		sensor.right_brush_oc = brush.getRightOc();
		brush.setMainOc((serial.receive_stream[REC_OC] & 0x04) != 0);
		sensor.main_brush_oc = brush.getMainOc();
		brush.setLeftOc((serial.receive_stream[REC_OC] & 0x08) != 0);
		sensor.left_brush_oc = brush.getLeftOc();
		wheel.setRightWheelOc((serial.receive_stream[REC_OC] & 0x10) != 0);
		sensor.right_wheel_oc = wheel.getRightWheelOc();
		wheel.setLeftWheelOc((serial.receive_stream[REC_OC] & 0x20) != 0);
		sensor.left_wheel_oc = wheel.getLeftWheelOc();

//		debug_received_stream();
#if GYRO_DYNAMIC_ADJUSTMENT
		if (wheel.getLeftWheelActualSpeed() < 0.01 && wheel.getRightWheelActualSpeed() < 0.01)
			gyro.setDynamicOn();
		else
			gyro.setDynamicOff();
		gyro.checkTilt();
#endif
		/*---------extrict end-------*/

		pthread_mutex_lock(&serial_data_ready_mtx);
		pthread_cond_broadcast(&serial_data_ready_cond);
		pthread_mutex_unlock(&serial_data_ready_mtx);
		sensor_pub.publish(sensor);

		/*------------setting for odom and publish odom topic --------*/
		odom.setMovingSpeed(static_cast<float>((wheel.getLeftWheelActualSpeed() + wheel.getRightWheelActualSpeed()) / 2.0));
		odom.setAngle(gyro.getAngle());
		odom.setAngleSpeed(gyro.getAngleV());
		cur_time = ros::Time::now();
		double angle_rad, dt;
		angle_rad = deg_to_rad(odom.getAngle());
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

	}//end while
	ROS_INFO("\033[32m%s\033[0m,%d,robotbase thread exit",__FUNCTION__,__LINE__);
}

void process_beep()
{
	// This routine handles the speaker sounding logic
	// If temp_beeper_silence_time_count == 0, it is the end of loop of silence, so decrease the count and set sound in g_send_stream.
	if (temp_beeper_silence_time_count == 0){
		temp_beeper_silence_time_count--;
		temp_beeper_sound_time_count = robotbase_beeper_sound_time_count;
		serial.setSendData(CTL_BEEPER, robotbase_sound_code & 0xFF);
	}
	// If temp_beeper_sound_time_count == 0, it is the end of loop of sound, so decrease the count and set sound in g_send_stream.
	if (temp_beeper_sound_time_count == 0){
		temp_beeper_sound_time_count--;
		temp_beeper_silence_time_count = robotbase_beeper_silence_time_count;
		serial.setSendData(CTL_BEEPER, 0x00);
		// Decreace the speaker sound loop count because when it turns to silence this sound loop will be over when silence end, so we can decreace the sound loop count here.
		// If it is for constant beeper.play, the loop count will be less than 0, it will not decrease either.
		if (robotbase_beeper_sound_loop_count > 0){
			robotbase_beeper_sound_loop_count--;
		}
	}
	// If temp_beeper_silence_time_count == -1, it is in loop of sound, so decrease the count.
	if (temp_beeper_silence_time_count == -1){
		temp_beeper_sound_time_count--;
	}
	// If temp_beeper_sound_time_count == -1, it is in loop of silence, so decrease the count.
	if (temp_beeper_sound_time_count == -1){
		temp_beeper_silence_time_count--;
	}
}

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
			if (live_led_cnt_for_switch > robotbase_led_cnt_for_switch / 2)
				led_brightness = 0;
			break;
		}
		case LED_BREATH:
		{
			if (live_led_cnt_for_switch > robotbase_led_cnt_for_switch / 2)
				led_brightness = led_brightness * (2 * (float)live_led_cnt_for_switch / (float)robotbase_led_cnt_for_switch - 1.0);
			else
				led_brightness = led_brightness * (1.0 - 2 * (float)live_led_cnt_for_switch / (float)robotbase_led_cnt_for_switch);
			break;
		}
	}

	if (live_led_cnt_for_switch++ > robotbase_led_cnt_for_switch)
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

void robotbase_reset_odom_pose(void)
{
	// Reset the odom pose to (0, 0)
	boost::mutex::scoped_lock(odom_mutex);
	odom.setX(0);
	odom.setY(0);
}


