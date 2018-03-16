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

#include "move_type.hpp"

#include "error.h"

MoveTypeDeskTest::MoveTypeDeskTest()
{
	ROS_INFO("%s,%d: Enter move type desk test.", __FUNCTION__, __LINE__);

	auto desk_test_routine = new boost::thread(boost::bind(&MoveTypeDeskTest::deskTestRoutineThread, this));
}

MoveTypeDeskTest::~MoveTypeDeskTest()
{
	ROS_INFO("%s,%d: Exit move type desk test.", __FUNCTION__, __LINE__);
}

void MoveTypeDeskTest::run()
{
	switch (test_stage)
	{
		case 0:
		{
			serial.setSendData(CTL_WORK_MODE, DESK_TEST_CURRENT_MODE);
			test_stage++;
			wheel.stop();
			brush.stop();
			vacuum.stop();
//			water_pump.stop();
			break;
		}
		case 1:
		{
			if (check_stage_1_finish())
			{
				test_stage++;
				test_step_ = 0;
				// Switch to next stage.
				serial.setSendData(CTL_WORK_MODE, DESK_TEST_MOVEMENT_MODE);
				p_movement_.reset();
				p_movement_.reset(new ActionOpenGyro());
				ROS_INFO("%s %d: Stage 1 finished, next stage: %d.", __FUNCTION__, __LINE__, test_stage);
			}
			break;
		}
		case 2:
		{
			if (check_stage_2_finish())
			{
				test_stage++;
				test_step_ = 0;
				// Switch to next stage.
				p_movement_.reset();
				p_movement_.reset(new MovementDirectGo(false));
				ROS_INFO("%s %d: Stage 2 finished, next stage: %d.", __FUNCTION__, __LINE__, test_stage);
				ROS_INFO("%s %d: Start checking for left bumper.", __FUNCTION__, __LINE__);
			}
			break;
		}
		case 3:
		{
			if (check_stage_3_finish())
			{
				test_stage++;
				test_step_ = 0;
				// Switch to next stage.
				sp_mode_->action_i_ = sp_mode_->ac_follow_wall_left;
				Points points_{};
				points_.push_front({0, 0, 0});
				p_movement_.reset();
				p_movement_.reset(new MoveTypeFollowWall(points_, true));

				ROS_INFO("%s %d: Stage 3 finished, next stage: %d.", __FUNCTION__, __LINE__, test_stage);
			}
			break;
		}
		case 4:
		{
			if (check_stage_4_finish())
			{
				test_stage++;
				test_step_ = 0;
				// Switch to next stage.
				p_movement_.reset();
				p_movement_.reset(new MovementTurn(getPosition().th + degree_to_radian(-150), LINEAR_MAX_SPEED));
				brush.fullOperate();
				vacuum.setMode(Vac_Max);
				// todo: for water tank
				ROS_INFO("%s %d: Stage 4 finished, next stage: %d.", __FUNCTION__, __LINE__, test_stage);
			}
			break;
		}
		case 5:
		{
			if (check_stage_5_finish())
			{
				test_stage++;
				test_step_ = 0;
				// Switch to next stage.
				c_rcon.resetStatus();
				p_movement_.reset();
				p_movement_.reset(new MovementTurn(getPosition().th + degree_to_radian(-179), ROTATE_TOP_SPEED * 2 / 3));
				brush.normalOperate();
				vacuum.setMode(Vac_Normal);
				// todo: for water tank
				ROS_INFO("%s %d: Stage 5 finished, next stage: %d.", __FUNCTION__, __LINE__, test_stage);
			}
			break;
		}
		case 6:
		{
			if (check_stage_6_finish())
			{
				test_stage++;
				test_step_ = 0;
				// Switch to next stage.
				c_rcon.resetStatus();
				p_movement_.reset();
				p_movement_.reset(new MoveTypeGoToCharger());
				brush.slowOperate();
				key_led.setMode(LED_ORANGE, LED_BREATH);
				ROS_INFO("%s %d: Stage 6 finished, next stage: %d.", __FUNCTION__, __LINE__, test_stage);
			}
			break;
		}
		case 7:
		{
			if (check_stage_7_finish())
			{
				test_stage++;
				wheel.stop();
				brush.stop();
				vacuum.stop();
				test_step_ = 0;
				// Switch to next stage.
				p_movement_.reset();
				ROS_INFO("%s %d: Stage 4 finished.", __FUNCTION__, __LINE__);
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
		default: // For finish.
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

void MoveTypeDeskTest::deskTestRoutineThread()
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
		if (dataExtract(buf))
		{
			odom.setMovingSpeed(
					static_cast<float>((wheel.getLeftWheelActualSpeed() + wheel.getRightWheelActualSpeed()) / 2.0));
			odom.setRadian(degree_to_radian(gyro.getAngle()));
			odom.setAngleSpeed(gyro.getAngleV());
			cur_time = ros::Time::now();
			double angle_rad, dt;
			angle_rad = odom.getRadian();
			dt = (cur_time - last_time).toSec();
			last_time = cur_time;
			odom.setX(static_cast<float>(odom.getX() + (odom.getMovingSpeed() * cos(angle_rad)) * dt));
			odom.setY(static_cast<float>(odom.getY() + (odom.getMovingSpeed() * sin(angle_rad)) * dt));
			robot::instance()->updateRobotPositionForTest();
			updatePosition();
		}
	}
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

	// For battery device.
	battery.setVoltage(static_cast<uint16_t>(buf[REC_BATTERY] * 10));

	// For key device.
	key.eliminate_jitter((buf[REC_MIX_BYTE] & 0x01) != 0);

	// For charger device.
	charger.setChargeStatus(static_cast<uint8_t>((buf[REC_MIX_BYTE] >> 4) & 0x07));

	if (current_work_mode == DESK_TEST_CURRENT_MODE)
	{
		brush.setLeftCurrent((buf[REC_L_BRUSH_CUNT_H] << 8) | buf[REC_L_BRUSH_CUNT_L]);
		brush.setRightCurrent((buf[REC_R_BRUSH_CUNT_H] << 8) | buf[REC_R_BRUSH_CUNT_L]);
		brush.setMainCurrent((buf[REC_M_BRUSH_CUNT_H] << 8) | buf[REC_M_BRUSH_CUNT_L]);

		wheel.setLeftCurrent((buf[REC_L_WHEEL_CUNT_H] << 8) | buf[REC_L_WHEEL_CUNT_L]);
		wheel.setRightCurrent((buf[REC_R_WHEEL_CUNT_H] << 8) | buf[REC_R_WHEEL_CUNT_L]);

		vacuum.setCurrent((buf[REC_VACUUM_CURRENT_H] << 8) | buf[REC_VACUUM_CURRENT_L]);

		water_tank.setCurrent((buf[REC_WATER_PUMP_CURRENT_H] << 8) | buf[REC_WATER_PUMP_CURRENT_L]);
	} else if (current_work_mode == DESK_TEST_MOVEMENT_MODE)
	{
		// For cliff device.
		cliff.setLeftValue((buf[REC_L_CLIFF_H] << 8) | buf[REC_L_CLIFF_L]);
		cliff.setFrontValue((buf[REC_F_CLIFF_H] << 8) | buf[REC_F_CLIFF_L]);
		cliff.setRightValue((buf[REC_R_CLIFF_H] << 8) | buf[REC_R_CLIFF_L]);

//		printf("cliff left:%d, front:%d, right:%d.\n", cliff.getLeftValue(), cliff.getFrontValue(), cliff.getRightValue());
		// For rcon device.
		c_rcon.setStatus((buf[REC_RCON_CHARGER_4] << 24) | (buf[REC_RCON_CHARGER_3] << 16)
						 | (buf[REC_RCON_CHARGER_2] << 8) | buf[REC_RCON_CHARGER_1]);
	}

	return true;
}

bool MoveTypeDeskTest::check_stage_1_finish()
{
	if (sum_cnt_ >= 50)
	{
		left_brush_current_baseline_ = static_cast<int16_t>(left_brush_current_baseline_sum_ / sum_cnt_);
		ROS_INFO("%s %d: left_brush_current_baseline_:%d.", __FUNCTION__, __LINE__, left_brush_current_baseline_);
		right_brush_current_baseline_ = static_cast<int16_t>(right_brush_current_baseline_sum_ / sum_cnt_);
		ROS_INFO("%s %d: right_brush_current_baseline_:%d.", __FUNCTION__, __LINE__, right_brush_current_baseline_);
		main_brush_current_baseline_ = static_cast<int16_t>(main_brush_current_baseline_sum_ / sum_cnt_);
		ROS_INFO("%s %d: main_brush_current_baseline_:%d.", __FUNCTION__, __LINE__, main_brush_current_baseline_);
		left_wheel_current_baseline_ = static_cast<int16_t>(left_wheel_current_baseline_sum_ / sum_cnt_);
		ROS_INFO("%s %d: left_wheel_current_baseline_:%d.", __FUNCTION__, __LINE__, left_wheel_current_baseline_);
		right_wheel_current_baseline_ = static_cast<int16_t>(right_wheel_current_baseline_sum_ / sum_cnt_);
		ROS_INFO("%s %d: right_wheel_current_baseline_:%d.", __FUNCTION__, __LINE__, right_wheel_current_baseline_);
		vacuum_current_baseline_ = static_cast<int16_t>(vacuum_current_baseline_sum_ / sum_cnt_);
		ROS_INFO("%s %d: vacuum_current_baseline_:%d.", __FUNCTION__, __LINE__, vacuum_current_baseline_);
		water_tank_current_baseline_ = static_cast<int16_t>(water_tank_current_baseline_sum_ / sum_cnt_);
		ROS_INFO("%s %d: water_tank_current_baseline_:%d.", __FUNCTION__, __LINE__, water_tank_current_baseline_);

		sum_cnt_ = 0;
		return true;
	}
	else
	{
		left_brush_current_baseline_sum_ += brush.getLeftCurrent();
		right_brush_current_baseline_sum_ += brush.getRightCurrent();
		main_brush_current_baseline_sum_ += brush.getMainCurrent();
		left_wheel_current_baseline_sum_ += wheel.getLeftCurrent();
		right_wheel_current_baseline_sum_ += wheel.getRightCurrent();
		vacuum_current_baseline_sum_ += vacuum.getCurrent();
		water_tank_current_baseline_sum_ += water_tank.getCurrent();
		sum_cnt_++;
	}

	return false;
}

bool MoveTypeDeskTest::check_stage_2_finish()
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
				test_step_++;
			}
			else
				p_movement_->run();

//			ROS_INFO("%s %d: angle:%f", __FUNCTION__, __LINE__, gyro.getAngle());
			break;
		}
		case 2:
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
			lidar_check_cnt_ = 0;
			p_movement_.reset();
			p_movement_.reset(new MovementTurn(getPosition().th + degree_to_radian(-179), ROTATE_TOP_SPEED * 2 / 3));
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
			p_movement_.reset(new MovementTurn(getPosition().th + degree_to_radian(-179), ROTATE_TOP_SPEED * 2 / 3));
			break;
		}
		default://case 6:
		{
			// Check for rcon.
			if (!(c_rcon.getStatus() & (RconBL_HomeT)))
				error_code_ = BLRCON_ERROR;
			if (error_code_ == 0 && (!(c_rcon.getStatus() & (RconL_HomeT))))
				error_code_ = LRCON_ERROR;
			if (error_code_ == 0 && (!(c_rcon.getStatus() & (RconFL2_HomeT))))
				error_code_ = FL2RCON_ERROR;
			if (error_code_ == 0 && (!(c_rcon.getStatus() & (RconFL_HomeT))))
				error_code_ = FLRCON_ERROR;
			if (error_code_ == 0 && (!(c_rcon.getStatus() & (RconFR_HomeT))))
				error_code_ = FRRCON_ERROR;
			if (error_code_ == 0 && (!(c_rcon.getStatus() & (RconFR2_HomeT))))
				error_code_ = FR2RCON_ERROR;
			if (error_code_ == 0 && (!(c_rcon.getStatus() & (RconR_HomeT))))
				error_code_ = RRCON_ERROR;
			if (error_code_ == 0 && (!(c_rcon.getStatus() & (RconBR_HomeT))))
				error_code_ = BRRCON_ERROR;

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
				test_stage = 99;
		}
	}
	return false;
}

bool MoveTypeDeskTest::check_stage_3_finish()
{
	left_obs_max_ = (obs.getLeft() > left_obs_max_) ? obs.getLeft() : left_obs_max_;
	front_obs_max_ = (obs.getFront() > front_obs_max_) ? obs.getFront() : front_obs_max_;
	right_obs_max_ = (obs.getRight() > right_obs_max_) ? obs.getRight() : right_obs_max_;
	switch (test_step_)
	{
		case 0: // For going towards the wall and test for left bumper.
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
				test_stage = 99;
				error_code_ = LEFT_BUMPER_ERROR;
			}
			else
				p_movement_->run();
			break;
		}
		case 1: // For moving back.
		{
			if (p_movement_->isFinish())
			{
				test_step_++;
				p_movement_.reset();
				p_movement_.reset(new MovementTurn(getPosition().th + degree_to_radian(70), ROTATE_TOP_SPEED));
			}
			else
				p_movement_->run();
			break;
		}
		case 2: // For turning.
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
		case 3: // For going towards the wall and test for right bumper.
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
				test_stage = 99;
				error_code_ = RIGHT_BUMPER_ERROR;
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
				p_movement_.reset(new MovementTurn(getPosition().th + degree_to_radian(-100), ROTATE_TOP_SPEED));
			}
			else
				p_movement_->run();
			break;
		}
		case 5: // For turning to follow wall and check for OBS.
		{
			if (p_movement_->isFinish())
			{
				// Checking for OBS value.
				ROS_INFO("%s %d: left_obs_max:%d, front_obs_max:%d, right_obs_max:%d.",
						 __FUNCTION__, __LINE__, left_obs_max_, front_obs_max_, right_obs_max_);
				if (left_obs_max_ - left_obs_baseline_ < obs_ref_)
					error_code_ = LEFT_OBS_ERROR;
				if (error_code_ != 0 && front_obs_max_ - front_obs_baseline_ < obs_ref_)
					error_code_ = FRONT_OBS_ERROR;
				if (error_code_ != 0 && right_obs_max_ - right_obs_baseline_ < obs_ref_)
					error_code_ = RIGHT_OBS_ERROR;

				if (error_code_ != 0)
					test_stage = 99;
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

bool MoveTypeDeskTest::check_stage_4_finish()
{
	switch (test_step_)
	{
		case 0: // Follow wall to the cliff.
		{
			left_cliff_max_ = (cliff.getLeftValue() > left_cliff_max_) ? cliff.getLeftValue() : left_cliff_max_;
			front_cliff_max_ = (cliff.getFrontValue() > front_cliff_max_) ? cliff.getFrontValue() : front_cliff_max_;
			right_cliff_max_ = (cliff.getRightValue() > right_cliff_max_) ? cliff.getRightValue() : right_cliff_max_;

			if (cliff.getFrontValue() < cliff_min_ref_)
			{
				printf("cliff left:%d, front:%d, right:%d.\n", left_cliff_max_, front_cliff_max_, right_cliff_max_);

				if (left_cliff_max_ < cliff_max_ref_)
					error_code_ = LEFT_CLIFF_ERROR;
				if (error_code_ != 0 && front_cliff_max_ < cliff_max_ref_)
					error_code_ = FRONT_CLIFF_ERROR;
				if (error_code_ != 0 && right_cliff_max_ < cliff_max_ref_)
					error_code_ = RIGHT_CLIFF_ERROR;

				if (error_code_ != 0)
					test_stage = 99;
				else
				{
					test_step_++;
					p_movement_.reset();
					p_movement_.reset(new MovementStay(0.2));
					sum_cnt_ = 0;
					sp_mode_->action_i_ = sp_mode_->ac_desk_test;
				}
			}
			else
			{
				p_movement_->isFinish(); // For calculating targets.
				p_movement_->run();
			}
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
				error_code_ = RIGHT_CLIFF_ERROR;
			else
				p_movement_->run();
			break;
		}
		case 3: // Getting baseline for right cliff.
		{
			if (p_movement_->isFinish())
			{
				p_movement_.reset();
				p_movement_.reset(new MovementBack(0.05, BACK_MAX_SPEED));
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
		default: //case 7: // Moving back.
		{
			if (p_movement_->isFinish())
				return true;
			else{
				left_cliff_sum_ += cliff.getLeftValue();
				sum_cnt_++;
				p_movement_->run();
			}
			break;
		}
	}

	return false;
}

bool MoveTypeDeskTest::check_stage_5_finish()
{
	left_brush_current_max_ = (brush.getLeftCurrent() > left_brush_current_max_) ?
							  brush.getLeftCurrent() : left_brush_current_max_;
	right_brush_current_max_ = (brush.getRightCurrent() > right_brush_current_max_) ?
							   brush.getRightCurrent() : right_brush_current_max_;
	main_brush_current_max_ = (brush.getMainCurrent() > main_brush_current_max_) ?
							  brush.getMainCurrent() : main_brush_current_max_;
	vacuum_current_max_ = (vacuum.getCurrent() > vacuum_current_max_) ?
						  vacuum.getCurrent() : vacuum_current_max_;
	water_tank_current_max_ = (water_tank.getCurrent() > water_tank_current_max_) ?
							  water_tank.getCurrent() : water_tank_current_max_;

	switch(test_step_)
	{
		case 0:
		{
			if (p_movement_->isFinish())
			{
				p_movement_.reset();
				p_movement_.reset(new MovementTurn(getPosition().th + degree_to_radian(60), LINEAR_MAX_SPEED));
				test_step_++;
			} else
			{
				left_wheel_forward_current_max_ = (wheel.getLeftCurrent() > left_wheel_forward_current_max_) ?
												  wheel.getLeftCurrent() : left_wheel_forward_current_max_;
				right_wheel_backward_current_max_ = (wheel.getRightCurrent() > right_wheel_backward_current_max_) ?
												   wheel.getRightCurrent() : right_wheel_backward_current_max_;
				p_movement_->run();
			}
			break;
		}
		case 1:
		{
			if (p_movement_->isFinish())
			{
				p_movement_.reset();
				p_movement_.reset(new MovementDirectGo(false, 1));
				test_step_++;
			} else
			{
				left_wheel_backward_current_max_ = (wheel.getLeftCurrent() > left_wheel_backward_current_max_) ?
												  wheel.getLeftCurrent() : left_wheel_backward_current_max_;
				right_wheel_forward_current_max_ = (wheel.getRightCurrent() > right_wheel_forward_current_max_) ?
												   wheel.getRightCurrent() : right_wheel_forward_current_max_;
				p_movement_->run();
			}
			break;
		}
		case 2:
		{
			if (p_movement_->isFinish())
			{
				/*uint16_t side_brush_current_ref_{0}; // todo:
				uint16_t main_brush_current_ref_{0};
				uint16_t wheel_current_ref_{0};
				uint16_t vacuum_current_ref_{0};
				uint16_t water_tank_current_ref_{0};

				if (left_brush_current_max_ - left_brush_current_baseline_ > side_brush_current_ref_)
					error_code_ = LEFT_BRUSH_CURRENT_ERROR;
				else if (right_brush_current_max_ - right_brush_current_baseline_ > side_brush_current_ref_)
					error_code_ = RIGHT_BRUSH_CURRENT_ERROR;
				else if (main_brush_current_max_ - main_brush_current_baseline_ > main_brush_current_ref_)
					error_code_ = MAIN_BRUSH_CURRENT_ERROR;
				else if (left_wheel_forward_current_max_ - left_wheel_current_baseline_ > wheel_current_ref_)
					error_code_ = LEFT_WHEEL_FORWARD_CURRENT_ERROR;
				else if (left_wheel_backward_current_max_ - left_wheel_current_baseline_ > wheel_current_ref_)
					error_code_ = LEFT_WHEEL_BACKWARD_CURRENT_ERROR;
				else if (right_wheel_forward_current_max_ - right_wheel_current_baseline_ > wheel_current_ref_)
					error_code_ = RIGHT_WHEEL_FORWARD_CURRENT_ERROR;
				else if (right_wheel_backward_current_max_ - right_wheel_current_baseline_ > wheel_current_ref_)
					error_code_ = RIGHT_WHEEL_BACKWARD_CURRENT_ERROR;
				else if (vacuum_current_max_ - vacuum_current_baseline_ > vacuum_current_ref_)
					error_code_ = VACUUM_CURRENT_ERROR;
				else if (water_tank_current_max_ - water_tank_current_baseline_ > water_tank_current_ref_)
					error_code_ = VACUUM_CURRENT_ERROR; // todo:*/

				if (error_code_ == 0)
				{
					sum_cnt_ = 0;
					return true;
				} else
					test_stage = 99;
			} else
				p_movement_->run();
			break;
		}
		default:
			break;
	}

	return false;
}

bool MoveTypeDeskTest::check_stage_6_finish()
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
					error_code_ = BLRCON_ERROR;
				if (error_code_ == 0 && (!(c_rcon.getStatus() & (RconL_HomeL | RconL_HomeR))))
					error_code_ = LRCON_ERROR;
				if (error_code_ == 0 && (!(c_rcon.getStatus() & (RconFL2_HomeL | RconFL2_HomeR))))
					error_code_ = FL2RCON_ERROR;
				if (error_code_ == 0 && (!(c_rcon.getStatus() & (RconFL_HomeL | RconFL_HomeR))))
					error_code_ = FLRCON_ERROR;
				if (error_code_ == 0 && (!(c_rcon.getStatus() & (RconFR_HomeL | RconFR_HomeR))))
					error_code_ = FRRCON_ERROR;
				if (error_code_ == 0 && (!(c_rcon.getStatus() & (RconFR2_HomeL | RconFR2_HomeR))))
					error_code_ = FR2RCON_ERROR;
				if (error_code_ == 0 && (!(c_rcon.getStatus() & (RconR_HomeL | RconR_HomeR))))
					error_code_ = RRCON_ERROR;
				if (error_code_ == 0 && (!(c_rcon.getStatus() & (RconBR_HomeL | RconBR_HomeR))))
					error_code_ = BRRCON_ERROR;

				if (error_code_ != 0)
					test_stage = 99;
				else
					return true;
			} else
				p_movement_->run();
			break;
		}
		default:
			break;
	}

	return false;
}

bool MoveTypeDeskTest::check_stage_7_finish()
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
