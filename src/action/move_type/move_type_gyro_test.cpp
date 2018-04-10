//
// Created by austin on 18-3-14.
//

#include <cmath>
#include <robot.hpp>
#include <brush.h>
#include <error.h>
#include <key_led.h>
#include <wheel.hpp>
#include <vacuum.h>
#include <beeper.h>
#include <gyro.h>
#include <infrared_display.hpp>
#include <speaker.h>
#include "move_type.hpp"
#include "error.h"

#define ROUTATE_SPEED 20

MoveTypeGyroTest::MoveTypeGyroTest()
{
	ROS_INFO("%s,%d: Enter move type gyro test.", __FUNCTION__, __LINE__);

	auto gyro_test_routine = new boost::thread(boost::bind(&MoveTypeGyroTest::gyroTestRoutineThread, this));

	p_movement_.reset(new ActionOpenGyro());
	last_time_stamp_ = ros::Time::now().toSec();
	infrared_display.displayNormalMsg(test_stage_, 0);
}

MoveTypeGyroTest::~MoveTypeGyroTest()
{
	ROS_INFO("%s,%d: Exit move type gyro test.", __FUNCTION__, __LINE__);
}

void MoveTypeGyroTest::gyroTestRoutineThread()
{
	ROS_INFO("\033[32m%s\033[0m,%d is up.",__FUNCTION__,__LINE__);

	uint8_t buf[REC_LEN];
	ros::Time cur_time, last_time;

	while (ros::ok() && !recei_thread_kill)
	{
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock) != 0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond, &recev_lock) != 0, "robotbase pthread receive cond wait fail");
		memcpy(buf, serial.receive_stream, sizeof(uint8_t) * REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock) != 0, "robotbase pthread receive unlock fail");
//		serial.debugReceivedStream(buf);
		if (dataExtract(buf))
		{
			if (test_stage_ > 0)
			{
				wheel_mileage_ +=
						(std::abs(wheel.getLeftEncoderCnt()) + std::abs(wheel.getRightEncoderCnt())) *
						WHEEL_ENCODER_TO_MILLIMETER / 1000;
				count_sum += std::abs(wheel.getLeftEncoderCnt());
			}
			odom.setOriginRadian(degree_to_radian(gyro.getAngleY()));
			odom.setAngleSpeed(gyro.getAngleV());
			cur_time = ros::Time::now();
			double angle_rad, dt;
			angle_rad = odom.getRadian();
			dt = (cur_time - last_time).toSec();
			last_time = cur_time;
			odom.setMovingSpeed(static_cast<float>((wheel.getLeftEncoderCnt() + wheel.getRightEncoderCnt()) *
												   WHEEL_ENCODER_TO_MILLIMETER / 1000 / 2.0 / dt));
			odom.setOriginX(static_cast<float>(odom.getOriginX() + ((wheel.getLeftEncoderCnt() + wheel.getRightEncoderCnt())
																												* WHEEL_ENCODER_TO_MILLIMETER * cos(angle_rad) / 1000 / 2)));
			odom.setOriginY(static_cast<float>(odom.getOriginY() + ((wheel.getLeftEncoderCnt() + wheel.getRightEncoderCnt())
																												* WHEEL_ENCODER_TO_MILLIMETER * sin(angle_rad) / 1000 / 2)));
//			odom.setOriginX(static_cast<float>(odom.getOriginX() + (odom.getMovingSpeed() * cos(angle_rad)) * dt));
//			odom.setOriginY(static_cast<float>(odom.getOriginY() + (odom.getMovingSpeed() * sin(angle_rad)) * dt));
			robot::instance()->updateRobotPositionForTest();
			updatePosition();
		}
	}
	pthread_cond_broadcast(&serial_data_ready_cond);
	event_manager_thread_kill = true;
	ROS_ERROR("%s,%d,exit",__FUNCTION__,__LINE__);
}

bool MoveTypeGyroTest::dataExtract(const uint8_t *buf)
{
//	robot::instance()->debugReceivedStream(buf);

	// For wheel encoder count.
	wheel.setLeftEncoderCnt(buf[REC_LEFT_WHEEL_ENCODER]);
	wheel.setRightEncoderCnt(buf[REC_RIGHT_WHEEL_ENCODER]);

	// For wheel device.
	wheel.setLeftWheelActualSpeed(
			static_cast<float>(static_cast<int16_t>((buf[REC_WHEEL_L_SPEED_H] << 8) | buf[REC_WHEEL_L_SPEED_L]) /
							   1000.0));
	wheel.setRightWheelActualSpeed(
			static_cast<float>(static_cast<int16_t>((buf[REC_WHEEL_R_SPEED_H] << 8) | buf[REC_WHEEL_R_SPEED_L]) /
							   1000.0));

	// For gyro device.
	gyro.setCalibration(buf[REC_GYRO_CALIBRATION] != 0);

	gyro.setAngleY(static_cast<float>(static_cast<int16_t>((buf[REC_ANGLE_H] << 8) | buf[REC_ANGLE_L]) / 100.0 * -1));
	gyro.setAngleV(
			static_cast<float>(static_cast<int16_t>((buf[REC_ANGLE_V_H] << 8) | buf[REC_ANGLE_V_L]) / 100.0 * -1));

	return true;
}

void MoveTypeGyroTest::run()
{
	switch (test_stage_)
	{
		case 1:
		{
			if (p_movement_->isFinish())
			{
				test_stage_++;
				infrared_display.displayNormalMsg(test_stage_, 0);
				p_movement_.reset();
				p_movement_.reset(new MovementTurn(getPosition().th + degree_to_radian(-90), ROUTATE_SPEED));
				saved_gyro_turn_angle_ = gyro.getAngleY();
				saved_wheel_mileage_ = wheel_mileage_;
				brush.normalOperate();
			}
			else
				p_movement_->run();
			break;
		}
		// First round.
		case 2:
		case 3:
		case 4:
		case 5:
		// Second round.
		case 6:
		case 7:
		case 8:
		case 9:
		// Third round.
		case 10:
		case 11:
		case 12:
		case 13:
		// Forth round.
		case 14:
		case 15:
		case 16:
		case 17:
		// Fifth round.
		case 18:
		case 19:
		case 20:
		case 21:
		{
			/*wheel.pidSetLeftSpeed(-5);
			wheel.pidSetRightSpeed(5);
			wheel_turn_angle_ = radian_to_degree((wheel_mileage_ - saved_wheel_mileage_) / 2 / WHEEL_TO_CENTER_DISTANCE);
			ROS_INFO("%s %d: Turn for %d times, wheel_turn_angle_:%f(%f in angle).",
					 __FUNCTION__, __LINE__, test_stage_, degree_to_radian(wheel_turn_angle_), wheel_turn_angle_);

			auto current_angle = gyro.getAngleY();
			auto gyro_diff = current_angle - saved_gyro_turn_angle_;
			ROS_INFO("%s %d: gyro_diff:%f(%f in angle).", __FUNCTION__, __LINE__, degree_to_radian(gyro_diff),
					 gyro_diff);
			double current_time = ros::Time::now().toSec();
			double dt = current_time - last_time_stamp_;
//			wheel_mileage_ +=
//					(std::fabs(wheel.getLeftWheelActualSpeed()) + std::fabs(wheel.getRightWheelActualSpeed())) * dt;
			last_time_stamp_ = current_time;*/

			if (p_movement_->isFinish())
			{
				wheel_turn_angle_ = radian_to_degree((wheel_mileage_ - saved_wheel_mileage_) / 2 / WHEEL_TO_CENTER_DISTANCE);
				ROS_INFO("count sum:%d.", count_sum);
				ROS_INFO("%s %d: Turn for %d times, wheel_turn_angle_:%f(%f in angle).",
						 __FUNCTION__, __LINE__, test_stage_ - 1, degree_to_radian(wheel_turn_angle_), wheel_turn_angle_);

				auto current_angle = gyro.getAngleY();
				auto gyro_diff = fabs(ranged_degree(current_angle - saved_gyro_turn_angle_));
				ROS_INFO("%s %d: gyro_diff:%f(%f in angle).", __FUNCTION__, __LINE__, degree_to_radian(gyro_diff), gyro_diff);

				auto diff = fabs(ranged_degree(wheel_turn_angle_ - gyro_diff));
				ROS_INFO("%s %d: diff = %f", __FUNCTION__, __LINE__, diff);
				if (diff > 6)
				{
					error_code_ = GYRO_ERROR;
					error_stage_ = test_stage_;
					error_content_ = static_cast<uint16_t>(diff * 10);
					test_stage_ = 99;
				}
				else
				{
					p_movement_.reset();
					wheel.stop();
					usleep(500000);
					if (test_stage_ == 21)
						p_movement_.reset();
					else
						p_movement_.reset(new MovementTurn(getPosition().th + degree_to_radian(-90), ROUTATE_SPEED));
					test_stage_++;
					infrared_display.displayNormalMsg(test_stage_, 0);
					current_angle = gyro.getAngleY();
					saved_gyro_turn_angle_ = current_angle;
					saved_wheel_mileage_ = wheel_mileage_;
				}
			}
			else
			{
				double current_time = ros::Time::now().toSec();
				double dt = current_time - last_time_stamp_;
//				wheel_mileage_ +=
//						(std::fabs(wheel.getLeftWheelActualSpeed()) + std::fabs(wheel.getRightWheelActualSpeed())) * dt;

				// Clear the encoder data.
				wheel.setLeftEncoderCnt(0);
				wheel.setRightEncoderCnt(0);

				last_time_stamp_ = current_time;
				p_movement_->run();
			}
			break;
		}
		case 99: // For error.
		{
			ROS_ERROR("%s %d: error code:%d", __FUNCTION__, __LINE__, error_code_);
			key_led.setMode(LED_STEADY, LED_RED);
			speaker.play(VOICE_TEST_FAIL);
			infrared_display.displayErrorMsg(error_stage_, error_content_, error_code_);
			wheel.stop();
			brush.stop();
			vacuum.stop();
			sleep(5);
			break;
		}
		default: // For finish.
		{
			ROS_INFO("%s %d: Test finish.", __FUNCTION__, __LINE__);
			key_led.setMode(LED_STEADY, LED_GREEN);
			infrared_display.displayNormalMsg(0, 9999);
			speaker.play(VOICE_TEST_SUCCESS);
//			beeper.beep(2, 40, 40, 3);
			wheel.stop();
			brush.stop();
			vacuum.stop();
			sleep(5);
			break;
		}
	}
}
