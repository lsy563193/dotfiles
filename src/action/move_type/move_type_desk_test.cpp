//
// Created by austin on 18-3-7.
//

#include <serial.h>
#include <wheel.hpp>
#include <brush.h>
#include <vacuum.h>
#include <key_led.h>
#include <lidar.hpp>
#include <gyro.h>
#include <bumper.h>
#include <obs.h>
#include <cliff.h>
#include <mode.hpp>
#include <battery.h>
#include <map.h>
#include <beeper.h>
#include <water_tank.hpp>
#include <charger.h>
#include <key.h>
#include <infrared_display.hpp>
#include <speaker.h>

#include "move_type.hpp"

#include "error.h"

#define RCON_ROTATE_SPEED (ROTATE_TOP_SPEED * 2 / 3)

// Sequence for baseline setting.
#define CTL_L_OBS_BL_H 2
#define CTL_L_OBS_BL_L 3
#define CTL_F_OBS_BL_H 4
#define CTL_F_OBS_BL_L 5
#define CTL_R_OBS_BL_H 6
#define CTL_R_OBS_BL_L 7
#define CTL_L_CLIFF_BL_H 8
#define CTL_L_CLIFF_BL_L 9
#define CTL_F_CLIFF_BL_H 10
#define CTL_F_CLIFF_BL_L 12
#define CTL_R_CLIFF_BL_H 15
#define CTL_R_CLIFF_BL_L 16

MoveTypeDeskTest::MoveTypeDeskTest()
{
	ROS_INFO("%s,%d: Enter move type desk test.", __FUNCTION__, __LINE__);

	auto desk_test_routine = new boost::thread(boost::bind(&MoveTypeDeskTest::deskTestRoutineThread, this));

	// Make sure lidar has been inited and stopped, and the voice has finished playing.
	usleep(3500000);
	serial.setSendData(CTL_WORK_MODE, DESK_TEST_CURRENT_MODE);
	wheel.stop();
	brush.stop();
	vacuum.stop();
//	water_pump.stop();
	infrared_display.displayNormalMsg(test_stage_, 0);
}

MoveTypeDeskTest::~MoveTypeDeskTest()
{
	ROS_INFO("%s,%d: Exit move type desk test.", __FUNCTION__, __LINE__);
}

void MoveTypeDeskTest::run()
{
	switch (test_stage_)
	{
		case 1:
		{
			if (checkStage1Finish())
			{
				test_stage_++;
				test_step_ = 0;
				// Switch to next stage.
				infrared_display.displayNormalMsg(test_stage_, 0);
				serial.setSendData(CTL_WORK_MODE, DESK_TEST_MOVEMENT_MODE);
				p_movement_.reset();
				p_movement_.reset(new ActionOpenGyro());
				ROS_INFO("%s %d: Stage 1 finished, next stage: %d.", __FUNCTION__, __LINE__, test_stage_);
			}
			break;
		}
		case 2:
		{
			if (checkStage2Finish())
			{
				test_stage_++;
				test_step_ = 0;
				// Switch to next stage.
				infrared_display.displayNormalMsg(test_stage_, 0);
				p_movement_.reset();
				p_movement_.reset(new MovementDirectGo(false));
				ROS_INFO("%s %d: Stage 2 finished, next stage: %d.", __FUNCTION__, __LINE__, test_stage_);
				ROS_INFO("%s %d: Start checking for left bumper.", __FUNCTION__, __LINE__);
			}
			break;
		}
		case 3:
		{
			if (checkStage3Finish())
			{
				serial.setSendData(CTL_WORK_MODE, DESK_TEST_CURRENT_MODE);
				test_stage_++;
				sum_cnt_ = 0;
				check_current_start_time_ = ros::Time::now().toSec();
				test_step_ = 0;
				// Switch to next stage.
				infrared_display.displayNormalMsg(test_stage_, 0);
				sp_mode_->action_i_ = sp_mode_->ac_follow_wall_left;
				Points points_{};
				points_.push_front({0, 0, 0});
				p_movement_.reset();
				p_movement_.reset(new MoveTypeFollowWall(points_, true));

				ROS_INFO("%s %d: Stage 3 finished, next stage: %d.", __FUNCTION__, __LINE__, test_stage_);
			}
			break;
		}
		case 4:
		{
			if (checkStage4Finish())
			{
				serial.setSendData(CTL_WORK_MODE, DESK_TEST_MOVEMENT_MODE);
				test_stage_++;
				test_step_ = 0;
				// Switch to next stage.
				infrared_display.displayNormalMsg(test_stage_, 0);
				p_movement_.reset();
				p_movement_.reset(new MovementStay(0.2));
				/*brush.fullOperate();
				vacuum.setMode(Vac_Max);*/
				// todo: for water tank
				ROS_INFO("%s %d: Stage 4 finished, next stage: %d.", __FUNCTION__, __LINE__, test_stage_);
			}
			break;
		}
		case 5:
		{
			if (checkStage5Finish())
			{
				test_stage_++;
				test_step_ = 0;
				// Switch to next stage.
				infrared_display.displayNormalMsg(test_stage_, 0);
				c_rcon.resetStatus();
				p_movement_.reset();
				p_movement_.reset(new MovementTurn(getPosition().th + degree_to_radian(-179), ROTATE_TOP_SPEED * 2 / 3));
				brush.normalOperate();
				vacuum.setMode(Vac_Normal);
				// todo: for water tank
				ROS_INFO("%s %d: Stage 5 finished, next stage: %d.", __FUNCTION__, __LINE__, test_stage_);
			}
			break;
		}
		case 6:
		{
			if (checkStage6Finish())
			{
				test_stage_++;
				test_step_ = 0;
				// Switch to next stage.
				infrared_display.displayNormalMsg(test_stage_, 0);
				c_rcon.resetStatus();
				p_movement_.reset();
				p_movement_.reset(new MoveTypeGoToCharger());
				brush.slowOperate();
				ROS_INFO("%s %d: Stage 6 finished, next stage: %d.", __FUNCTION__, __LINE__, test_stage_);
			}
			break;
		}
		case 7:
		{
			if (checkStage7Finish())
			{
				test_stage_++;
				wheel.stop();
				brush.stop();
				vacuum.stop();
				test_step_ = 0;
				// Switch to next stage.
				p_movement_.reset();
				wheel.stop();
				brush.stop();
				vacuum.stop();
				lidar.motorCtrl(OFF);
				usleep(50000);
				ROS_INFO("%s %d: Stage 7 finished.", __FUNCTION__, __LINE__);
			}
			break;
		}
		case 99: // For error.
		{
			ROS_ERROR("%s %d: error code: %d", __FUNCTION__, __LINE__, error_code_);
			key_led.setMode(LED_STEADY, LED_RED);
			wheel.stop();
			brush.stop();
			vacuum.stop();
			lidar.motorCtrl(OFF);
			infrared_display.displayErrorMsg(error_step_, error_content_, error_code_);
			serial.debugSendStream(serial.send_stream);
			speaker.play(VOICE_TEST_FAIL);
			sleep(5);
			break;
		}
		default: // For finish.
		{
			ROS_INFO("%s %d: Test finish.", __FUNCTION__, __LINE__);
			infrared_display.displayNormalMsg(0, 0);
			key_led.setMode(LED_STEADY, LED_GREEN);
//			beeper.beep(2, 40, 40, 3);
			serial.setSendData(CTL_WORK_MODE, DESK_TEST_WRITE_BASELINE_MODE);
			serial.setSendData(CTL_L_OBS_BL_H, static_cast<uint8_t>(left_obs_baseline_ >> 8));
			serial.setSendData(CTL_L_OBS_BL_L, static_cast<uint8_t>(left_obs_baseline_));
			serial.setSendData(CTL_F_OBS_BL_H, static_cast<uint8_t>(front_obs_baseline_ >> 8));
			serial.setSendData(CTL_F_OBS_BL_L, static_cast<uint8_t>(front_obs_baseline_));
			serial.setSendData(CTL_R_OBS_BL_H, static_cast<uint8_t>(right_obs_baseline_ >> 8));
			serial.setSendData(CTL_R_OBS_BL_L, static_cast<uint8_t>(right_obs_baseline_));
			serial.setSendData(CTL_L_CLIFF_BL_H, static_cast<uint8_t>(left_cliff_baseline_ >> 8));
			serial.setSendData(CTL_L_CLIFF_BL_L, static_cast<uint8_t>(left_cliff_baseline_));
			serial.setSendData(CTL_F_CLIFF_BL_H, static_cast<uint8_t>(front_cliff_baseline_ >> 8));
			serial.setSendData(CTL_F_CLIFF_BL_L, static_cast<uint8_t>(front_cliff_baseline_));
			serial.setSendData(CTL_R_CLIFF_BL_H, static_cast<uint8_t>(right_cliff_baseline_ >> 8));
			serial.setSendData(CTL_R_CLIFF_BL_L, static_cast<uint8_t>(right_cliff_baseline_));
			serial.debugSendStream(serial.send_stream);
			speaker.play(VOICE_TEST_SUCCESS);
			sleep(5);
			serial.debugReceivedStream(serial.receive_stream);
			break;
		}
	}
}

void MoveTypeDeskTest::deskTestRoutineThread()
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
//		robot::instance()->debugReceivedStream(buf);
		if (dataExtract(buf))
		{
			odom.setMovingSpeed(
					static_cast<float>((wheel.getLeftWheelActualSpeed() + wheel.getRightWheelActualSpeed()) / 2.0));
			odom.setOriginRadian(degree_to_radian(gyro.getAngleY()));
			odom.setAngleSpeed(gyro.getAngleV());
			cur_time = ros::Time::now();
			double angle_rad, dt;
			angle_rad = odom.getRadian();
			dt = (cur_time - last_time).toSec();
			last_time = cur_time;
			odom.setOriginX(static_cast<float>(odom.getOriginX() + (odom.getMovingSpeed() * cos(angle_rad)) * dt));
			odom.setOriginY(static_cast<float>(odom.getOriginY() + (odom.getMovingSpeed() * sin(angle_rad)) * dt));
			robot::instance()->updateRobotPositionForTest();
			updatePosition();
		}
	}
	pthread_cond_broadcast(&serial_data_ready_cond);
	event_manager_thread_kill = true;
	ROS_ERROR("%s,%d,exit",__FUNCTION__,__LINE__);
}

bool MoveTypeDeskTest::dataExtract(const uint8_t *buf)
{
//	robot::instance()->debugReceivedStream(buf);

	// work mode
	auto current_work_mode = buf[REC_WORK_MODE];
	if (current_work_mode != serial.getSendData(CTL_WORK_MODE))
		return false;

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

	// For bumper device.
	bumper.setLeft((buf[REC_BUMPER_AND_CLIFF] & 0x20) != 0);
	bumper.setRight((buf[REC_BUMPER_AND_CLIFF] & 0x10) != 0);

	bumper.setLidarBumperStatus();

	// For obs sensor device.
	obs.setLeft(((buf[REC_L_OBS_H] << 8) | buf[REC_L_OBS_L]) + obs.getLeftBaseline());
	obs.setFront(((buf[REC_F_OBS_H] << 8) | buf[REC_F_OBS_L]) + obs.getFrontBaseline());
	obs.setRight(((buf[REC_R_OBS_H] << 8) | buf[REC_R_OBS_L]) + obs.getRightBaseline());
//	printf("obs left:%d, front:%d, right:%d.\n", obs.getLeft(), obs.getFront(), obs.getRight());

	// For battery device.
	battery.setVoltage(static_cast<uint16_t>(buf[REC_BATTERY] * 10));

	// For key device.
	key.eliminate_jitter((buf[REC_MIX_BYTE] & 0x01) != 0);

	// For charger device.
	charger.setChargeStatus(static_cast<uint8_t>((buf[REC_MIX_BYTE] >> 4) & 0x07));

	// For cliff device.
	cliff.setLeftValue((buf[REC_L_CLIFF_H] << 8) | buf[REC_L_CLIFF_L]);
	cliff.setFrontValue((buf[REC_F_CLIFF_H] << 8) | buf[REC_F_CLIFF_L]);
	cliff.setRightValue((buf[REC_R_CLIFF_H] << 8) | buf[REC_R_CLIFF_L]);
//	printf("cliff left:%d, front:%d, right:%d.\n", cliff.getLeftValue(), cliff.getFrontValue(), cliff.getRightValue());

	if (current_work_mode == DESK_TEST_CURRENT_MODE)
	{
		brush.setLeftCurrent((buf[REC_L_BRUSH_CUNT_H] << 8) | buf[REC_L_BRUSH_CUNT_L]);
		brush.setRightCurrent((buf[REC_R_BRUSH_CUNT_H] << 8) | buf[REC_R_BRUSH_CUNT_L]);
		brush.setMainCurrent((buf[REC_M_BRUSH_CUNT_H] << 8) | buf[REC_M_BRUSH_CUNT_L]);

		wheel.setLeftCurrent((buf[REC_L_WHEEL_CUNT_H] << 8) | buf[REC_L_WHEEL_CUNT_L]);
		wheel.setRightCurrent((buf[REC_R_WHEEL_CUNT_H] << 8) | buf[REC_R_WHEEL_CUNT_L]);

		vacuum.setCurrent((buf[REC_VACUUM_CURRENT_H] << 8) | buf[REC_VACUUM_CURRENT_L]);

//		water_tank.setCurrent((buf[REC_WATER_PUMP_CURRENT_H] << 8) | buf[REC_WATER_PUMP_CURRENT_L]);
		water_tank.setCurrent(0);

		robot::instance()->setCurrent((buf[REC_ROBOT_CUNT_H] << 8) | buf[REC_ROBOT_CUNT_L]);

//		printf("brush(l%d, r%d, m%d), wheel(l%d, r%d), vacuum(%d), robot(%d).\n",
//			   brush.getLeftCurrent(), brush.getRightCurrent(), brush.getMainCurrent(),
//			   wheel.getLeftCurrent(), wheel.getRightCurrent(), vacuum.getCurrent(),
//			   robot::instance()->getCurrent());

	} else if (current_work_mode == DESK_TEST_MOVEMENT_MODE)
	{

		// For rcon device.
		c_rcon.setStatus((buf[REC_RCON_CHARGER_4] << 24) | (buf[REC_RCON_CHARGER_3] << 16)
						 | (buf[REC_RCON_CHARGER_2] << 8) | buf[REC_RCON_CHARGER_1]);
	}

	return true;
}

bool MoveTypeDeskTest::checkStage1Finish()
{
	if (sum_cnt_ >= 50)
	{
		left_brush_current_baseline_ /= sum_cnt_;
		ROS_INFO("%s %d: left_brush_current_baseline_:%d.", __FUNCTION__, __LINE__, left_brush_current_baseline_);
		right_brush_current_baseline_ /= sum_cnt_;
		ROS_INFO("%s %d: right_brush_current_baseline_:%d.", __FUNCTION__, __LINE__, right_brush_current_baseline_);
		main_brush_current_baseline_ /= sum_cnt_;
		ROS_INFO("%s %d: main_brush_current_baseline_:%d.", __FUNCTION__, __LINE__, main_brush_current_baseline_);
		left_wheel_current_baseline_ /= sum_cnt_;
		ROS_INFO("%s %d: left_wheel_current_baseline_:%d.", __FUNCTION__, __LINE__, left_wheel_current_baseline_);
		right_wheel_current_baseline_ /= sum_cnt_;
		ROS_INFO("%s %d: right_wheel_current_baseline_:%d.", __FUNCTION__, __LINE__, right_wheel_current_baseline_);
		vacuum_current_baseline_ /= sum_cnt_;
		ROS_INFO("%s %d: vacuum_current_baseline_:%d.", __FUNCTION__, __LINE__, vacuum_current_baseline_);
		water_tank_current_baseline_ /= sum_cnt_;
		ROS_INFO("%s %d: water_tank_current_baseline_:%d.", __FUNCTION__, __LINE__, water_tank_current_baseline_);
		robot_current_baseline_ /= sum_cnt_;
		ROS_INFO("%s %d: robot_current_baseline_:%d.", __FUNCTION__, __LINE__, robot_current_baseline_);

		int32_t side_brush_current_baseline_ref_{1620};
		int32_t main_brush_current_baseline_ref_{1620};
		int32_t wheel_current_baseline_ref_{1620};
		int32_t vacuum_current_baseline_ref_{1620};
		int32_t water_tank_current_baseline_ref_{0};
		int32_t robot_current_baseline_ref_{1775};

		if (left_brush_current_baseline_ > side_brush_current_baseline_ref_ * 1.2 ||
			left_brush_current_baseline_ < side_brush_current_baseline_ref_ * 0.8)
		{
			error_code_ = LEFT_BRUSH_CURRENT_ERROR;
			error_content_ = static_cast<uint16_t>(left_brush_current_baseline_);
		}
		else if (right_brush_current_baseline_ > side_brush_current_baseline_ref_ * 1.2 ||
				 right_brush_current_baseline_ < side_brush_current_baseline_ref_ * 0.8)
		{
			error_code_ = RIGHT_BRUSH_CURRENT_ERROR;
			error_content_ = static_cast<uint16_t>(right_brush_current_baseline_);
		}
		else if (main_brush_current_baseline_ > main_brush_current_baseline_ref_ * 1.2 ||
				 main_brush_current_baseline_ < main_brush_current_baseline_ref_ * 0.8)
		{
			error_code_ = MAIN_BRUSH_CURRENT_ERROR;
			error_content_ = static_cast<uint16_t>(main_brush_current_baseline_);
		}
		else if (vacuum_current_baseline_ > vacuum_current_baseline_ref_ * 1.2 ||
				 vacuum_current_baseline_ < vacuum_current_baseline_ref_ * 0.8)
		{
			error_code_ = VACUUM_CURRENT_ERROR;
			error_content_ = static_cast<uint16_t>(vacuum_current_baseline_);
		}
		/*else if (water_tank_current_baseline_ > water_tank_current_baseline_ref_ * 1.2 ||
				 water_tank_current_baseline_ < water_tank_current_baseline_ref_ * 0.8)
		{
			error_code_ = VACUUM_CURRENT_ERROR; // todo:
			error_content_ = static_cast<uint16_t>(water_tank_current_baseline_);
		}*/
		else if (left_wheel_current_baseline_ > wheel_current_baseline_ref_ * 1.2 ||
				 left_wheel_current_baseline_ < wheel_current_baseline_ref_ * 0.8)
		{
			error_code_ = LEFT_WHEEL_FORWARD_CURRENT_ERROR;
			error_content_ = static_cast<uint16_t>(left_wheel_current_baseline_);
		}
		else if (right_wheel_current_baseline_ > wheel_current_baseline_ref_ * 1.2 ||
				 right_wheel_current_baseline_ < wheel_current_baseline_ref_ * 0.8)
		{
			error_code_ = RIGHT_WHEEL_FORWARD_CURRENT_ERROR;
			error_content_ = static_cast<uint16_t>(right_wheel_current_baseline_);
		}
		else if (robot_current_baseline_ > robot_current_baseline_ref_ * 1.2 ||
				 robot_current_baseline_ < robot_current_baseline_ref_ * 0.8)
		{
			error_code_ = BASELINE_CURRENT_ERROR;
			error_content_ = static_cast<uint16_t>(robot_current_baseline_);
		}

		if (error_code_ == 0)
		{
			sum_cnt_ = 0;
			return true;
		} else
		{
			error_step_ = test_stage_;
			test_stage_ = 99;
		}
	}
	else
	{
		left_brush_current_baseline_ += brush.getLeftCurrent();
		right_brush_current_baseline_ += brush.getRightCurrent();
		main_brush_current_baseline_ += brush.getMainCurrent();
		left_wheel_current_baseline_ += wheel.getLeftCurrent();
		right_wheel_current_baseline_ += wheel.getRightCurrent();
		vacuum_current_baseline_ += vacuum.getCurrent();
		water_tank_current_baseline_ += water_tank.getCurrent();
		robot_current_baseline_ += robot::instance()->getCurrent();
		sum_cnt_++;
	}

	return false;
}

bool MoveTypeDeskTest::checkStage2Finish()
{
	if (test_step_ <= 5)
	{
		left_obs_sum_ += obs.getLeft();
		front_obs_sum_ += obs.getFront();
		right_obs_sum_ += obs.getRight();
		sum_cnt_++;
	}

	switch (test_step_)
	{
		case 0: // For opening gyro.
		case 1: // For opening lidar.
		case 3: // Turn for receiving rcon.
		case 5: // Turn for receiving rcon.
		{
			if (p_movement_->isFinish())
			{
				if (test_step_ == 0)
				{
					p_movement_.reset();
					p_movement_.reset(new ActionOpenLidar());
					brush.normalOperate();
					vacuum.setMode(Vac_Normal);
				}
				else if (test_step_ == 1)
					c_rcon.resetStatus();
				test_step_++;
			}
			else
				p_movement_->run();

//			ROS_INFO("%s %d: angle:%f", __FUNCTION__, __LINE__, gyro.getAngleY());
			break;
		}
		case 2:
		{
			ROS_INFO("%s %d: Enter lidar checking 1.", __FUNCTION__, __LINE__);
			wheel.stop();
			// Stop for checking lidar data.
			while (ros::ok() && lidar_check_cnt_ < 5)
			{
				auto lidar_data = lidar.getLidarScanDataOriginal();
				if (lidar_check_seq_ != lidar_data.header.seq)
				{
					lidar_check_cnt_++;
					// todo:handler for lidar.
				}
				usleep(100000);
			}
			test_step_++;
			lidar_check_cnt_ = 0;
			p_movement_.reset();
			p_movement_.reset(new MovementTurn(getPosition().th + degree_to_radian(-178), RCON_ROTATE_SPEED));
			ROS_INFO("%s %d: Enter rcon turning 1.", __FUNCTION__, __LINE__);
			break;
		}
		case 4:
		{
			ROS_INFO("%s %d: Enter lidar checking 2.", __FUNCTION__, __LINE__);
			wheel.stop();
			// Stop for checking lidar data.
			while (ros::ok() && lidar_check_cnt_ < 5)
			{
				auto lidar_data = lidar.getLidarScanDataOriginal();
				if (lidar_check_seq_ != lidar_data.header.seq)
				{
					lidar_check_cnt_++;
					// todo:handler for lidar.
				}
				usleep(100000);
			}
			test_step_++;
			p_movement_.reset();
			p_movement_.reset(new MovementTurn(getPosition().th + degree_to_radian(-178), RCON_ROTATE_SPEED));
			break;
		}
		default://case 6:
		{
			// Check for rcon.
			if (!(c_rcon.getStatus() & (RconBL_HomeT)))
			{
				error_code_ = BLRCON_ERROR;
				error_content_ = static_cast<uint16_t>((c_rcon.getStatus() & (RconBL_HomeT | RconBL_HomeL | RconBL_HomeR)) >> 28);
			}
			else if (!(c_rcon.getStatus() & (RconL_HomeT)))
			{
				error_code_ = LRCON_ERROR;
				error_content_ = static_cast<uint16_t>((c_rcon.getStatus() & (RconL_HomeT | RconL_HomeL | RconL_HomeR)) >> 24);
			}
			else if (!(c_rcon.getStatus() & (RconFL2_HomeT)))
			{
				error_code_ = FL2RCON_ERROR;
				error_content_ = static_cast<uint16_t>((c_rcon.getStatus() & (RconFL2_HomeT | RconFL2_HomeL | RconFL2_HomeR)) >> 20);
			}
			else if (!(c_rcon.getStatus() & (RconFL_HomeT)))
			{
				error_code_ = FLRCON_ERROR;
				error_content_ = static_cast<uint16_t>((c_rcon.getStatus() & (RconFL_HomeT | RconFL_HomeL | RconFL_HomeR)) >> 16);
			}
			else if (!(c_rcon.getStatus() & (RconFR_HomeT)))
			{
				error_code_ = FRRCON_ERROR;
				error_content_ = static_cast<uint16_t>((c_rcon.getStatus() & (RconFR_HomeT | RconFR_HomeL | RconFR_HomeR)) >> 12);
			}
			else if (!(c_rcon.getStatus() & (RconFR2_HomeT)))
			{
				error_code_ = FR2RCON_ERROR;
				error_content_ = static_cast<uint16_t>((c_rcon.getStatus() & (RconFR2_HomeT | RconFR2_HomeL | RconFR2_HomeR)) >> 8);
			}
			else if (!(c_rcon.getStatus() & (RconR_HomeT)))
			{
				error_code_ = RRCON_ERROR;
				error_content_ = static_cast<uint16_t>((c_rcon.getStatus() & (RconR_HomeT | RconR_HomeL | RconR_HomeR)) >> 4);
			}
			else if (!(c_rcon.getStatus() & (RconBR_HomeT)))
			{
				error_code_ = BRRCON_ERROR;
				error_content_ = static_cast<uint16_t>((c_rcon.getStatus() & (RconBR_HomeT | RconBR_HomeL | RconBR_HomeR)));
			}

			if (error_code_ == 0)
			{
				left_obs_baseline_ = static_cast<uint16_t>(left_obs_sum_ / sum_cnt_);
				ROS_INFO("%s %d: left_obs_baseline_:%d.", __FUNCTION__, __LINE__, left_obs_baseline_);
				front_obs_baseline_ = static_cast<uint16_t>(front_obs_sum_ / sum_cnt_);
				ROS_INFO("%s %d: front_obs_baseline_:%d.", __FUNCTION__, __LINE__, front_obs_baseline_);
				right_obs_baseline_ = static_cast<uint16_t>(right_obs_sum_ / sum_cnt_);
				ROS_INFO("%s %d: right_obs_baseline_:%d.", __FUNCTION__, __LINE__, right_obs_baseline_);
				sum_cnt_ = 0;
				return true;
			} else
			{
				error_step_ = test_stage_;
				test_stage_ = 99;
				ROS_ERROR("%s %d: rcon status:%x.", __FUNCTION__, __LINE__, c_rcon.getStatus());
			}
		}
	}
	return false;
}

bool MoveTypeDeskTest::checkStage3Finish()
{
	left_obs_max_ = (obs.getLeft() > left_obs_max_) ? obs.getLeft() : left_obs_max_;
	front_obs_max_ = (obs.getFront() > front_obs_max_) ? obs.getFront() : front_obs_max_;
	right_obs_max_ = (obs.getRight() > right_obs_max_) ? obs.getRight() : right_obs_max_;
	switch (test_step_)
	{
		case 0: // For going near the wall.
		{
			if (bumper.getStatus())
			{
				p_movement_.reset();
				p_movement_.reset(new MovementBack(0.01, BACK_MAX_SPEED));
				test_step_++;
			} else
				p_movement_->run();
			break;
		}
		case 1:
		{
			if (p_movement_->isFinish())
			{
				p_movement_.reset();
				p_movement_.reset(new MovementTurn(getPosition().th - degree_to_radian(45), ROTATE_TOP_SPEED));
				test_step_++;
			} else
				p_movement_->run();
			break;
		}
		case 2:
		{
			if (p_movement_->isFinish())
			{
				p_movement_.reset();
				p_movement_.reset(new MovementDirectGo(false, 1));
				test_step_++;
			} else
				p_movement_->run();
			break;
		}
		case 3: // For going towards the wall and test for left bumper.
		{
			if (bumper.getStatus() & BLOCK_LEFT)
			{
				test_step_++;
				p_movement_.reset();
				p_movement_.reset(new MovementBack(0.01, BACK_MAX_SPEED));
				ROS_INFO("%s %d: Left bumper checked.", __FUNCTION__, __LINE__);
			}
			else if (p_movement_->isTimeUp())
			{
				error_step_ = test_stage_;
				test_stage_ = 99;
				error_code_ = LEFT_BUMPER_ERROR;
			}
			else
				p_movement_->run();
			break;
		}
		case 4: // For moving back.
		{
			if (p_movement_->isFinish())
			{
				test_step_++;
				p_movement_.reset();
				p_movement_.reset(new MovementTurn(getPosition().th + degree_to_radian(90), ROTATE_TOP_SPEED));
			}
			else
				p_movement_->run();
			break;
		}
		case 5: // For turning.
		{
			if (p_movement_->isFinish())
			{
				test_step_++;
				p_movement_.reset();
				p_movement_.reset(new MovementDirectGo(false));
				ROS_INFO("%s %d: Start checking for right bumper.", __FUNCTION__, __LINE__);
			}
			else
				p_movement_->run();
			break;
		}
		case 6: // For going towards the wall and test for right bumper.
		{
			if (p_movement_->isFinish())
			{
				test_step_++;
				p_movement_.reset();
				p_movement_.reset(new MovementBack(0.01, BACK_MAX_SPEED));
				ROS_INFO("%s %d: Right bumper checked.", __FUNCTION__, __LINE__);
			}
			else if (p_movement_->isTimeUp())
			{
				error_step_ = test_stage_;
				test_stage_ = 99;
				error_code_ = RIGHT_BUMPER_ERROR;
			}
			else
				p_movement_->run();
			break;
		}
		case 7: // For moving back.
		{
			if (p_movement_->isFinish())
			{
				test_step_++;
				p_movement_.reset();
				p_movement_.reset(new MovementTurn(getPosition().th + degree_to_radian(-130), ROTATE_TOP_SPEED));
			}
			else
				p_movement_->run();
			break;
		}
		case 8: // For turning to follow wall and check for OBS.
		{
			if (p_movement_->isFinish())
			{
				// Checking for OBS value.
				ROS_INFO("%s %d: left_obs_max:%d, front_obs_max:%d, right_obs_max:%d.",
						 __FUNCTION__, __LINE__, left_obs_max_, front_obs_max_, right_obs_max_);
				if (left_obs_max_ - left_obs_baseline_ < obs_ref_)
				{
					error_code_ = LEFT_OBS_ERROR;
					error_content_ = static_cast<uint16_t>(left_obs_max_ - left_obs_baseline_);
				}
				else if (front_obs_max_ - front_obs_baseline_ < obs_ref_)
				{
					error_code_ = FRONT_OBS_ERROR;
					error_content_ = static_cast<uint16_t>(front_obs_max_ - front_obs_baseline_);
				}
				else if (right_obs_max_ - right_obs_baseline_ < obs_ref_)
				{
					error_code_ = RIGHT_OBS_ERROR;
					error_content_ = static_cast<uint16_t>(right_obs_max_ - right_obs_baseline_);
				}

				if (error_code_ != 0)
				{
					error_step_ = test_stage_;
					test_stage_ = 99;
				}
				else
					return true;
			}
			else
				p_movement_->run();
			break;
		}

		default:
			break;
	}

	return false;
}

bool MoveTypeDeskTest::checkStage4Finish()
{
	switch (test_step_)
	{
		case 0: // Follow wall to the cliff.
		{
			if (bumper.getLidarBumperStatus())
				lidar_bumper_valid_ = true;

			if (ros::Time::now().toSec() - check_current_start_time_ > 3)
			{
				if (!checkCurrent())
				{
					error_step_ = test_stage_;
					test_stage_ = 99;
				}
				else
				{
					check_current_start_time_ = ros::Time::now().toSec();
					left_brush_current_ = 0;
					right_brush_current_ = 0;
					main_brush_current_ = 0;
					vacuum_current_ = 0;
					left_wheel_current_ = 0;
					right_wheel_current_ = 0;
					robot_current_ = 0;
				}
			}else
			{
				left_brush_current_ += brush.getLeftCurrent();
				right_brush_current_ += brush.getRightCurrent();
				main_brush_current_ += brush.getMainCurrent();
				vacuum_current_ += vacuum.getCurrent();
				if (wheel.getLeftWheelActualSpeed() > 0.15)
				{
					left_wheel_current_ += wheel.getLeftCurrent();
					left_wheel_current_cnt_++;
				}
				if (wheel.getRightWheelActualSpeed() > 0.15)
				{
					right_wheel_current_ += wheel.getRightCurrent();
					right_wheel_current_cnt_++;
				}
				robot_current_ += robot::instance()->getCurrent();
				sum_cnt_++;
			}

			left_cliff_max_ = (cliff.getLeftValue() > left_cliff_max_) ? cliff.getLeftValue() : left_cliff_max_;
			front_cliff_max_ = (cliff.getFrontValue() > front_cliff_max_) ? cliff.getFrontValue() : front_cliff_max_;
			right_cliff_max_ = (cliff.getRightValue() > right_cliff_max_) ? cliff.getRightValue() : right_cliff_max_;

			if (cliff.getFrontValue() < cliff_min_ref_)
			{
				if (!lidar_bumper_valid_)
					error_code_ = LIDAR_BUMPER_ERROR;

				if (error_code_ != 0)
				{
					error_step_ = test_stage_;
					test_stage_ = 99;
				} else
				{
					sum_cnt_ = 0;
					sp_mode_->action_i_ = sp_mode_->ac_desk_test;
					return true;
				}
			}
			else
			{
				p_movement_->isFinish(); // For calculating targets.
				p_movement_->run();
			}
			break;
		}
	}

	return false;
}

bool MoveTypeDeskTest::checkCurrent()
{

	left_brush_current_ /= sum_cnt_;
	ROS_INFO("%s %d: Left brush current: %d.", __FUNCTION__, __LINE__, left_brush_current_ - left_brush_current_baseline_);
	right_brush_current_ /= sum_cnt_;
	ROS_INFO("%s %d: Right brush current: %d.", __FUNCTION__, __LINE__, right_brush_current_ - right_brush_current_baseline_);
	main_brush_current_ /= sum_cnt_;
	ROS_INFO("%s %d: Main brush current: %d.", __FUNCTION__, __LINE__, main_brush_current_ - main_brush_current_baseline_);
	left_wheel_current_ /= left_wheel_current_cnt_;
	ROS_INFO("%s %d: Left wheel current: %d.", __FUNCTION__, __LINE__, left_wheel_current_ - left_wheel_current_baseline_);
	right_wheel_current_ /= right_wheel_current_cnt_;
	ROS_INFO("%s %d: Right wheel current: %d.", __FUNCTION__, __LINE__, right_wheel_current_ - right_wheel_current_baseline_);
	vacuum_current_ /= sum_cnt_;
	ROS_INFO("%s %d: Vacuum current: %d.", __FUNCTION__, __LINE__, vacuum_current_ - vacuum_current_baseline_);
	water_tank_current_ /= sum_cnt_;
	ROS_INFO("%s %d: Water tank current: %d.", __FUNCTION__, __LINE__, water_tank_current_ - water_tank_current_baseline_);
	robot_current_ /= sum_cnt_;
	ROS_INFO("%s %d: Robot current: %d.", __FUNCTION__, __LINE__, robot_current_ - robot_current_baseline_);

	// During follow wall
	uint16_t side_brush_current_ref_{1675 - 1620}; // 55
	uint16_t main_brush_current_ref_{1785 - 1620}; // 165
	uint16_t wheel_current_ref_{1685 - 1620}; // 65
	uint16_t vacuum_current_ref_{1970 - 1620}; // 350
	uint16_t water_tank_current_ref_{0};
	uint16_t robot_current_ref_{2250 - 1775}; // 475

	if (left_brush_current_ - left_brush_current_baseline_ > side_brush_current_ref_ * 1.5 /* 82 */||
		left_brush_current_ - left_brush_current_baseline_ < side_brush_current_ref_ * 0.6 /* 33 */)
	{
		error_code_ = LEFT_BRUSH_CURRENT_ERROR;
		error_content_ = static_cast<uint16_t>(left_brush_current_ - left_brush_current_baseline_);
	}
	else if (right_brush_current_ - right_brush_current_baseline_ > side_brush_current_ref_ * 1.4 /* 77 */||
			 right_brush_current_ - right_brush_current_baseline_ < side_brush_current_ref_ * 0.6 /* 33 */)
	{
		error_code_ = RIGHT_BRUSH_CURRENT_ERROR;
		error_content_ = static_cast<uint16_t>(right_brush_current_ - right_brush_current_baseline_);
	}
	else if (main_brush_current_ - main_brush_current_baseline_ > main_brush_current_ref_ * 1.2 ||
			 main_brush_current_ - main_brush_current_baseline_ < main_brush_current_ref_ * 0.8)
	{
		error_code_ = MAIN_BRUSH_CURRENT_ERROR;
		error_content_ = static_cast<uint16_t>(main_brush_current_ - main_brush_current_baseline_);
	}
	else if (vacuum_current_ - vacuum_current_baseline_ > vacuum_current_ref_ * 1.2 ||
			 vacuum_current_ - vacuum_current_baseline_ < vacuum_current_ref_ * 0.8)
	{
		error_code_ = VACUUM_CURRENT_ERROR;
		error_content_ = static_cast<uint16_t>(vacuum_current_ - vacuum_current_baseline_);
	}
	/*else if (water_tank_current_ - water_tank_current_baseline_ > water_tank_current_ref_ * 1.2 ||
			 water_tank_current_ - water_tank_current_baseline_ < water_tank_current_ref_ * 0.8)
	{
		error_code_ = VACUUM_CURRENT_ERROR; // todo:
		error_content_ = static_cast<uint16_t>(water_tank_current_ - water_tank_current_baseline_);
	}*/
	else if (left_wheel_current_cnt_ != 0 && (left_wheel_current_ - left_wheel_current_baseline_ > wheel_current_ref_ * 1.3 ||
			 left_wheel_current_ - left_wheel_current_baseline_ < wheel_current_ref_ * 0.6))
	{
		error_code_ = LEFT_WHEEL_FORWARD_CURRENT_ERROR;
		error_content_ = static_cast<uint16_t>(left_wheel_current_ - left_wheel_current_baseline_);
	}
	else if (right_wheel_current_cnt_ != 0 && (right_wheel_current_ - right_wheel_current_baseline_ > wheel_current_ref_ * 1.3 ||
			 right_wheel_current_ - right_wheel_current_baseline_ < wheel_current_ref_ * 0.6))
	{
		error_code_ = RIGHT_WHEEL_FORWARD_CURRENT_ERROR;
		error_content_ = static_cast<uint16_t>(right_wheel_current_ - right_wheel_current_baseline_);
	}
	else if (robot_current_ - robot_current_baseline_ > robot_current_ref_ * 1.2 ||
			 robot_current_ - robot_current_baseline_ < robot_current_ref_ * 0.8)
	{
		error_code_ = BASELINE_CURRENT_ERROR;
		error_content_ = static_cast<uint16_t>(robot_current_ - robot_current_baseline_);
	}

	sum_cnt_ = 0;
	left_wheel_current_cnt_ = 0;
	right_wheel_current_cnt_ = 0;

	return error_code_ == 0;
}

bool MoveTypeDeskTest::checkStage5Finish()
{
	/*left_brush_current_max_ = (brush.getLeftCurrent() > left_brush_current_max_) ?
							  brush.getLeftCurrent() : left_brush_current_max_;
	right_brush_current_max_ = (brush.getRightCurrent() > right_brush_current_max_) ?
							   brush.getRightCurrent() : right_brush_current_max_;
	main_brush_current_max_ = (brush.getMainCurrent() > main_brush_current_max_) ?
							  brush.getMainCurrent() : main_brush_current_max_;
	vacuum_current_max_ = (vacuum.getCurrent() > vacuum_current_max_) ?
						  vacuum.getCurrent() : vacuum_current_max_;
	water_tank_current_max_ = (water_tank.getCurrent() > water_tank_current_max_) ?
							  water_tank.getCurrent() : water_tank_current_max_;*/

	switch(test_step_)
	{
		case 0:
		{
			ROS_INFO("%s %d: cliff left:%d, front:%d, right:%d.", __FUNCTION__, __LINE__, left_cliff_max_,
					 front_cliff_max_, right_cliff_max_);

			if (left_cliff_max_ < cliff_max_ref_)
			{
				error_code_ = LEFT_CLIFF_ERROR;
				error_content_ = static_cast<uint16_t>(left_cliff_max_);
			} else if (front_cliff_max_ < cliff_max_ref_)
			{
				error_code_ = FRONT_CLIFF_ERROR;
				error_content_ = static_cast<uint16_t>(front_cliff_max_);
			} else if (right_cliff_max_ < cliff_max_ref_)
			{
				error_code_ = RIGHT_CLIFF_ERROR;
				error_content_ = static_cast<uint16_t>(right_cliff_max_);
			}

			if (error_code_ != 0)
			{
				error_step_ = test_stage_;
				test_stage_ = 99;
			} else
				test_step_++;
			break;
		}
		case 1: // Getting baseline for front cliff.
		{
			if (p_movement_->isFinish())
			{
				p_movement_.reset();
				p_movement_.reset(new MovementTurn(getPosition().th + degree_to_radian(90), ROTATE_TOP_SPEED));
				front_cliff_baseline_ = static_cast<uint16_t>(front_cliff_sum_ / sum_cnt_);
				ROS_INFO("%s %d: front_cliff_baseline_:%d.", __FUNCTION__, __LINE__, front_cliff_baseline_);
				test_step_++;
			} else{
				front_cliff_sum_ += cliff.getFrontValue();
				sum_cnt_++;
				p_movement_->run();
			}
			break;
		}
		case 2: // Turning left to check for right cliff.
		{
			if (cliff.getRightValue() < cliff_min_ref_)
			{
				p_movement_.reset();
				p_movement_.reset(new MovementStay(0.2));
				sum_cnt_ = 0;
				test_step_++;
			}
			else if (p_movement_->isFinish())
			{
				error_code_ = RIGHT_CLIFF_ERROR;
				error_content_ = static_cast<uint16_t>(cliff.getRightValue());
			}
			else
				p_movement_->run();
			break;
		}
		case 3: // Getting baseline for right cliff.
		{
			if (p_movement_->isFinish())
			{
				p_movement_.reset();
				p_movement_.reset(new MovementBack(0.1, BACK_MAX_SPEED));
				movement_i_= mm_back;
				right_cliff_baseline_ = static_cast<uint16_t>(right_cliff_sum_ / sum_cnt_);
				ROS_INFO("%s %d: right_cliff_baseline_:%d.", __FUNCTION__, __LINE__, right_cliff_baseline_);
				test_step_++;
			} else{
				right_cliff_sum_ += cliff.getRightValue();
				sum_cnt_++;
				p_movement_->run();
			}
			break;
		}
		case 4: // Moving back and turn and go for testing left cliff.
		{
			if (movement_i_ == mm_straight)
			{
				if (cliff.getLeftValue() < cliff_min_ref_)
				{
					p_movement_.reset();
					p_movement_.reset(new MovementStay(0.2));
					sum_cnt_ = 0;
					test_step_++;
				}
				else
					p_movement_->run();
			}
			else if (p_movement_->isFinish())
			{
				if (movement_i_ == mm_back)
				{
					movement_i_ = mm_turn;
					p_movement_.reset();
					p_movement_.reset(new MovementTurn(getPosition().th + degree_to_radian(-120), ROTATE_TOP_SPEED));
				}
				else if (movement_i_ == mm_turn)
				{
					movement_i_ = mm_straight;
					p_movement_.reset();
					p_movement_.reset(new MovementDirectGo(false));
				}
			}
			else
				p_movement_->run();
			break;
		}
		case 5: // Getting baseline for left cliff.
		{
			if (p_movement_->isFinish())
			{
				p_movement_.reset();
				p_movement_.reset(new MovementBack(0.1, BACK_MAX_SPEED));
				movement_i_= mm_back;
				left_cliff_baseline_ = static_cast<uint16_t>(left_cliff_sum_ / sum_cnt_);
				ROS_INFO("%s %d: left_cliff_baseline_:%d.", __FUNCTION__, __LINE__, left_cliff_baseline_);
				test_step_++;
			} else{
				left_cliff_sum_ += cliff.getLeftValue();
				sum_cnt_++;
				p_movement_->run();
			}
			break;
		}
		case 6: // Moving back.
		{
			if (p_movement_->isFinish())
			{
				p_movement_.reset();
				p_movement_.reset(new MovementTurn(getPosition().th + degree_to_radian(-90), ROTATE_TOP_SPEED));
				test_step_++;
			}
			else{
				left_cliff_sum_ += cliff.getLeftValue();
				sum_cnt_++;
				p_movement_->run();
			}
			break;
		}
		case 7:
		{
			if (p_movement_->isFinish())
			{
				p_movement_.reset();
				p_movement_.reset(new MovementDirectGo(false, 1));
				test_step_++;
			} else
				p_movement_->run();
			break;
		}
		default: // case 8:
		{
			if (p_movement_->isFinish())
				return true;
			else
				p_movement_->run();
			break;
		}
	}

	return false;
}

bool MoveTypeDeskTest::checkStage6Finish()
{
	switch(test_step_)
	{
		case 0:
		{
			if (p_movement_->isFinish())
			{
				p_movement_.reset();
				p_movement_.reset(
						new MovementTurn(getPosition().th + degree_to_radian(-179), ROTATE_TOP_SPEED * 2 / 3));
				test_step_++;
			} else
				p_movement_->run();
			break;
		}
		case 1:
		{
			if (p_movement_->isFinish())
			{
				// Check for rcon.
				if (!(c_rcon.getStatus() & (RconBL_HomeL | RconBL_HomeR)))
				{
					error_code_ = BLRCON_ERROR;
					error_content_ = static_cast<uint16_t>((c_rcon.getStatus() & (RconBL_HomeT | RconBL_HomeL | RconBL_HomeR)) >> 28);
				}
				else if (!(c_rcon.getStatus() & (RconL_HomeL | RconL_HomeR)))
				{
					error_code_ = LRCON_ERROR;
					error_content_ = static_cast<uint16_t>((c_rcon.getStatus() & (RconL_HomeT | RconL_HomeL | RconL_HomeR)) >> 24);
				}
				else if (!(c_rcon.getStatus() & (RconFL2_HomeL | RconFL2_HomeR)))
				{
					error_code_ = FL2RCON_ERROR;
					error_content_ = static_cast<uint16_t>((c_rcon.getStatus() & (RconFL2_HomeT | RconFL2_HomeL | RconFL2_HomeR)) >> 20);
				}
				else if (!(c_rcon.getStatus() & (RconFL_HomeL | RconFL_HomeR)))
				{
					error_code_ = FLRCON_ERROR;
					error_content_ = static_cast<uint16_t>((c_rcon.getStatus() & (RconFL_HomeT | RconFL_HomeL | RconFL_HomeR)) >> 16);
				}
				else if (!(c_rcon.getStatus() & (RconFR_HomeL | RconFR_HomeR)))
				{
					error_code_ = FRRCON_ERROR;
					error_content_ = static_cast<uint16_t>((c_rcon.getStatus() & (RconFR_HomeT | RconFR_HomeL | RconFR_HomeR)) >> 12);
				}
				else if (!(c_rcon.getStatus() & (RconFR2_HomeL | RconFR2_HomeR)))
				{
					error_code_ = FR2RCON_ERROR;
					error_content_ = static_cast<uint16_t>((c_rcon.getStatus() & (RconFR2_HomeT | RconFR2_HomeL | RconFR2_HomeR)) >> 8);
				}
				else if (!(c_rcon.getStatus() & (RconR_HomeL | RconR_HomeR)))
				{
					error_code_ = RRCON_ERROR;
					error_content_ = static_cast<uint16_t>((c_rcon.getStatus() & (RconR_HomeT | RconR_HomeL | RconR_HomeR)) >> 4);
				}
				else if (!(c_rcon.getStatus() & (RconBR_HomeL | RconBR_HomeR)))
				{
					error_code_ = BRRCON_ERROR;
					error_content_ = static_cast<uint16_t>((c_rcon.getStatus() & (RconBR_HomeT | RconBR_HomeL | RconBR_HomeR)));
				}

				if (error_code_ != 0)
				{
					error_step_ = test_stage_;
					test_stage_ = 99;
				}
				else
				{
					p_movement_.reset();
					p_movement_.reset(
							new MovementTurn(getPosition().th + degree_to_radian(45), ROTATE_TOP_SPEED * 2 / 3));
					test_step_++;
				}
			} else
				p_movement_->run();
			break;
		}
		case 2:
		{
			if (p_movement_->isFinish())
				return true;
			else
				p_movement_->run();
			break;
		}
		default:
			break;
	}

	return false;
}

bool MoveTypeDeskTest::checkStage7Finish()
{
	switch(test_step_)
	{
		case 0:
		{
			if (p_movement_->isFinish())
			{
				p_movement_.reset();
				vacuum.stop();
				brush.stop();
				lidar.motorCtrl(OFF);
				p_movement_.reset(new ActionIdle());
				key_led.setMode(LED_STEADY, LED_ORANGE);
				test_step_++;
			} else
				p_movement_->run();
			break;
		}
		case 1:
		{
			if (key.getPressStatus())
			{
				beeper.beepForCommand(VALID);
				p_movement_.reset();
				p_movement_.reset(new ActionBackFromCharger());
				test_step_++;
			} else
				p_movement_->run();
			break;
		}
		case 2:
		{
			if (p_movement_->isFinish())
				return true;
			else
				p_movement_->run();
		}
		default:
			break;
	}

	return false;
}
