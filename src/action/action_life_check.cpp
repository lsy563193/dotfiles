//
// Created by austin on 18-3-15.
//

#include <serial.h>
#include <robot.hpp>
#include <brush.h>
#include <wheel.hpp>
#include <vacuum.h>
#include <water_tank.hpp>
#include <battery.h>
#include <key_led.h>
#include <beeper.h>
#include <error.h>
#include "action.hpp"

ActionLifeCheck::ActionLifeCheck()
{
	ROS_INFO("%s %d: Enter life check.", __FUNCTION__, __LINE__);

	auto life_test_routine = new boost::thread(boost::bind(&ActionLifeCheck::lifeTestRoutineThread, this));

	start_time_stamp_ = ros::Time::now().toSec();

	wheel_current_ref_ = static_cast<uint16_t>(100.0 /*mA*/ / (330.0 /*3.3v reference voltage*/ / 0.1 /*Sampling resistance*/ / 4096.0 /*Accuracy*/));
	side_brush_current_ref_ = static_cast<uint16_t>(110.0 /*mA*/ / (330.0 /*3.3v reference voltage*/ / 0.1 /*Sampling resistance*/ / 4096.0 /*Accuracy*/));
	main_brush_current_ref_ = static_cast<uint16_t>(300.0 /*mA*/ / (330.0 /*3.3v reference voltage*/ / 0.1 /*Sampling resistance*/ / 4096.0 /*Accuracy*/));
	vacuum_current_ref_ = static_cast<uint16_t>(1100.0 /*mA*/ / (330.0 /*3.3v reference voltage*/ / 0.1 /*Sampling resistance*/ / 4096.0 /*Accuracy*/));
	water_tank_current_ref_ = static_cast<uint16_t>(1100.0 /*mA*/ / (330.0 /*3.3v reference voltage*/ / 0.1 /*Sampling resistance*/ / 4096.0 /*Accuracy*/));

}

void ActionLifeCheck::lifeTestRoutineThread()
{
	ROS_INFO("\033[32m%s\033[0m,%d is up.",__FUNCTION__,__LINE__);

	uint8_t buf[REC_LEN];
	ros::Time cur_time, last_time;

	while (ros::ok())
	{
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock) != 0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond, &recev_lock) != 0, "robotbase pthread receive cond wait fail");
		memcpy(buf, serial.receive_stream, sizeof(uint8_t) * REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock) != 0, "robotbase pthread receive unlock fail");
//		robot::instance()->debugReceivedStream(buf);
		dataExtract(buf);
	}
}

bool ActionLifeCheck::dataExtract(const uint8_t *buf)
{
//	robot::instance()->debugReceivedStream(buf);

	// work mode
	auto current_work_mode = buf[REC_WORK_MODE];
	if (current_work_mode != serial.getSendData(CTL_WORK_MODE))
		return false;

	/*// For wheel device.
	wheel.setLeftWheelActualSpeed(
			static_cast<float>(static_cast<int16_t>((buf[REC_WHEEL_L_SPEED_H] << 8) | buf[REC_WHEEL_L_SPEED_L]) /
							   1000.0));
	wheel.setRightWheelActualSpeed(
			static_cast<float>(static_cast<int16_t>((buf[REC_WHEEL_R_SPEED_H] << 8) | buf[REC_WHEEL_R_SPEED_L]) /
							   1000.0));

	// For gyro device.
	gyro.setCalibration(buf[REC_GYRO_CALIBRATION] != 0);

	gyro.setAngle(static_cast<float>(static_cast<int16_t>((buf[REC_ANGLE_H] << 8) | buf[REC_ANGLE_L]) / 100.0 * -1));
	gyro.setAngleV(
			static_cast<float>(static_cast<int16_t>((buf[REC_ANGLE_V_H] << 8) | buf[REC_ANGLE_V_L]) / 100.0 * -1));

	// For bumper device.
	bumper.setLeft((buf[REC_BUMPER_AND_CLIFF] & 0x20) != 0);
	bumper.setRight((buf[REC_BUMPER_AND_CLIFF] & 0x10) != 0);

	bumper.setLidarBumperStatus();

	// For obs sensor device.
	obs.setLeft(((buf[REC_L_OBS_H] << 8) | buf[REC_L_OBS_L]) + obs.getLeftBaseline());
	obs.setFront(((buf[REC_F_OBS_H] << 8) | buf[REC_F_OBS_L]) + obs.getFrontBaseline());
	obs.setRight(((buf[REC_R_OBS_H] << 8) | buf[REC_R_OBS_L]) + obs.getRightBaseline());
//	printf("obs left:%d, front:%d, right:%d.\n", obs.getLeft(), obs.getFront(), obs.getRight());

	// For key device.
	key.eliminate_jitter((buf[REC_MIX_BYTE] & 0x01) != 0);

	// For charger device.
	charger.setChargeStatus(static_cast<uint8_t>((buf[REC_MIX_BYTE] >> 4) & 0x07));*/

	// For battery device.
	battery.setVoltage(static_cast<uint16_t>(buf[REC_BATTERY] * 10));

	brush.setLeftCurrent((buf[REC_L_BRUSH_CUNT_H] << 8) | buf[REC_L_BRUSH_CUNT_L]);
	brush.setRightCurrent((buf[REC_R_BRUSH_CUNT_H] << 8) | buf[REC_R_BRUSH_CUNT_L]);
	brush.setMainCurrent((buf[REC_M_BRUSH_CUNT_H] << 8) | buf[REC_M_BRUSH_CUNT_L]);

	wheel.setLeftCurrent((buf[REC_L_WHEEL_CUNT_H] << 8) | buf[REC_L_WHEEL_CUNT_L]);
	wheel.setRightCurrent((buf[REC_R_WHEEL_CUNT_H] << 8) | buf[REC_R_WHEEL_CUNT_L]);

	vacuum.setCurrent((buf[REC_VACUUM_CURRENT_H] << 8) | buf[REC_VACUUM_CURRENT_L]);

	water_tank.setCurrent((buf[REC_WATER_PUMP_CURRENT_H] << 8) | buf[REC_WATER_PUMP_CURRENT_L]);

	return true;
}

void ActionLifeCheck::run()
{
	switch (test_stage_)
	{
		case 0:
		{
			if (ros::Time::now().toSec() - start_time_stamp_ > 0.2)
			{
				brush.normalOperate();
				vacuum.setMode(Vac_Normal);
				wheel.setPidTargetSpeed(LINEAR_MAX_SPEED, LINEAR_MAX_SPEED);
				key_led.setMode(LED_FLASH, LED_ORANGE, 600);
				test_stage_++;
				left_brush_current_baseline_ /= sum_cnt_;
				right_brush_current_baseline_ /= sum_cnt_;
				main_brush_current_baseline_ /= sum_cnt_;
				left_wheel_current_baseline_ /= sum_cnt_;
				right_wheel_current_baseline_ /= sum_cnt_;
				ROS_INFO("%s %d: left_brush_current_baseline_:%d.", __FUNCTION__, __LINE__,
						 left_brush_current_baseline_);
				ROS_INFO("%s %d: right_brush_current_baseline_:%d.", __FUNCTION__, __LINE__,
						 right_brush_current_baseline_);
				ROS_INFO("%s %d: main_brush_current_baseline_:%d.", __FUNCTION__, __LINE__,
						 main_brush_current_baseline_);
				ROS_INFO("%s %d: left_wheel_current_baseline_:%d.", __FUNCTION__, __LINE__,
						 left_wheel_current_baseline_);
				ROS_INFO("%s %d: right_wheel_current_baseline_:%d.", __FUNCTION__, __LINE__,
						 right_wheel_current_baseline_);
				ROS_INFO("%s %d: vacuum_current_baseline_:%d.", __FUNCTION__, __LINE__, vacuum_current_baseline_);
				ROS_INFO("%s %d: water_tank_current_baseline_:%d.", __FUNCTION__, __LINE__,
						 water_tank_current_baseline_);

			}
			else
			{
				left_brush_current_baseline_ += brush.getLeftCurrent();
				right_brush_current_baseline_ += brush.getRightCurrent();
				main_brush_current_baseline_ += brush.getMainCurrent();
				left_wheel_current_baseline_ += wheel.getLeftCurrent();
				right_wheel_current_baseline_ += wheel.getRightCurrent();
				vacuum_current_baseline_ += vacuum.getCurrent();
				sum_cnt_++;
			}
			break;
		}
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
		{
			left_brush_current_max_ = (brush.getLeftCurrent() > left_brush_current_max_) ? brush.getLeftCurrent()
																						 : left_brush_current_max_;
			right_brush_current_max_ = (brush.getRightCurrent() > right_brush_current_max_) ? brush.getRightCurrent()
																							: right_brush_current_max_;
			main_brush_current_max_ = (brush.getMainCurrent() > main_brush_current_max_) ? brush.getMainCurrent()
																						 : main_brush_current_max_;
			vacuum_current_max_ = (vacuum.getCurrent() > vacuum_current_max_) ? vacuum.getCurrent() :
								  vacuum_current_max_;
			if (test_stage_ % 2 == 1)
			{
				left_wheel_forward_current_max_ = (wheel.getLeftCurrent() > left_wheel_forward_current_max_) ?
												  wheel.getLeftCurrent() : left_wheel_forward_current_max_;
				right_wheel_forward_current_max_ = (wheel.getRightCurrent() > right_wheel_forward_current_max_) ?
												   wheel.getRightCurrent() : right_wheel_forward_current_max_;
			} else
			{
				left_wheel_backward_current_max_ = (wheel.getLeftCurrent() > left_wheel_backward_current_max_) ?
												  wheel.getLeftCurrent() : left_wheel_backward_current_max_;
				right_wheel_backward_current_max_ = (wheel.getRightCurrent() > right_wheel_backward_current_max_) ?
												   wheel.getRightCurrent() : right_wheel_backward_current_max_;
			}

			if (!checkCurrent())
				test_stage_ = 99;
			else if ((test_stage_ % 2 == 1) && ros::Time::now().toSec() - start_time_stamp_ > 5)
			{
				test_stage_++;
				wheel.setPidTargetSpeed(-LINEAR_MAX_SPEED, -LINEAR_MAX_SPEED);
				ROS_INFO("%s %d: Switch to backward, switch stage to %d.", __FUNCTION__, __LINE__, test_stage_);
				start_time_stamp_ = ros::Time::now().toSec();
			}
			else if ((test_stage_ % 2 == 0) && ros::Time::now().toSec() - start_time_stamp_ > 5)
			{
				test_stage_++;
				wheel.setPidTargetSpeed(LINEAR_MAX_SPEED, LINEAR_MAX_SPEED);
				ROS_INFO("%s %d: Switch to backward, switch stage to %d.", __FUNCTION__, __LINE__, test_stage_);
				start_time_stamp_ = ros::Time::now().toSec();
			}
			break;
		}
		case 99: // For error.
		{
			ROS_ERROR("%s %d: error code:%d", __FUNCTION__, __LINE__, error_code_);
			key_led.setMode(LED_STEADY, LED_RED);
			wheel.stop();
			brush.stop();
			vacuum.stop();
			sleep(5);
			break;
		}
		default: // case 10:
		{
			ROS_INFO("%s %d: Test finish.", __FUNCTION__, __LINE__);
			key_led.setMode(LED_STEADY, LED_GREEN);
			beeper.beep(2, 40, 40, 3);
			wheel.stop();
			brush.stop();
			vacuum.stop();
			sleep(5);
			break;
		}
	}

}

bool ActionLifeCheck::checkCurrent()
{

	if (left_brush_current_max_ - left_brush_current_baseline_ > side_brush_current_ref_)
		error_code_ = LEFT_BRUSH_CURRENT_ERROR;
	else if (right_brush_current_max_ - right_brush_current_baseline_> side_brush_current_ref_)
		error_code_ = RIGHT_BRUSH_CURRENT_ERROR;
	else if (main_brush_current_max_ - main_brush_current_baseline_ > main_brush_current_ref_)
		error_code_ = MAIN_BRUSH_CURRENT_ERROR;
	else if (left_wheel_forward_current_max_ - left_wheel_current_baseline_ > wheel_current_ref_)
		error_code_ = LEFT_WHEEL_FORWARD_CURRENT_ERROR;
	else if (left_wheel_backward_current_max_ - left_wheel_current_baseline_> wheel_current_ref_)
		error_code_ = LEFT_WHEEL_BACKWARD_CURRENT_ERROR;
	else if (right_wheel_forward_current_max_ - right_wheel_current_baseline_ > wheel_current_ref_)
		error_code_ = RIGHT_WHEEL_FORWARD_CURRENT_ERROR;
	else if (right_wheel_backward_current_max_ - right_wheel_current_baseline_ > wheel_current_ref_)
		error_code_ = RIGHT_WHEEL_BACKWARD_CURRENT_ERROR;
	else if (vacuum_current_max_ - vacuum_current_baseline_ > vacuum_current_ref_)
		error_code_ = VACUUM_CURRENT_ERROR;
	else if (water_tank_current_max_ - water_tank_current_baseline_ > water_tank_current_ref_)
		error_code_ = VACUUM_CURRENT_ERROR; // todo:

	return error_code_ == 0;
}
