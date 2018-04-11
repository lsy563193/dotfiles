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
#include <infrared_display.hpp>
#include <speaker.h>
#include "action.hpp"

ActionLifeCheck::ActionLifeCheck()
{
	ROS_INFO("%s %d: Enter life check.", __FUNCTION__, __LINE__);

	auto life_test_routine = new boost::thread(boost::bind(&ActionLifeCheck::lifeTestRoutineThread, this));

	timeout_interval_ = 300;

	// Make sure lidar has been inited and stopped, and the voice has finished playing.
	usleep(3500000);

	start_time_stamp_ = ros::Time::now().toSec();
	infrared_display.displayNormalMsg(test_stage_, 0);
}

void ActionLifeCheck::lifeTestRoutineThread()
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
		dataExtract(buf);
	}

	pthread_cond_broadcast(&serial_data_ready_cond);
	event_manager_thread_kill = true;
	printf("%s,%d,exit\n",__FUNCTION__,__LINE__);
}

bool ActionLifeCheck::dataExtract(const uint8_t *buf)
{
//	serial.debugReceivedStream(buf);

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

	robot::instance()->setCurrent((buf[REC_ROBOT_CUNT_H] << 8) | buf[REC_ROBOT_CUNT_L]);
//	printf("brush(l%d, r%d, m%d), wheel(l%d, r%d), vacuum(%d), robot(%d).\n",
//		   brush.getLeftCurrent(), brush.getRightCurrent(), brush.getMainCurrent(),
//		   wheel.getLeftCurrent(), wheel.getRightCurrent(), vacuum.getCurrent(),
//		   robot::instance()->getCurrent());

	return true;
}

void ActionLifeCheck::run()
{
	switch (test_stage_)
	{
		case 1:
		{
			if (ros::Time::now().toSec() - start_time_stamp_ > 0.3)
			{
				brush.normalOperate();
				vacuum.isMaxInClean(false);
				vacuum.setCleanState();
				if(water_tank.checkEquipment(false))
					water_tank.open(WaterTank::tank_pump);
				wheel.setPidTargetSpeed(LINEAR_MAX_SPEED, LINEAR_MAX_SPEED);
				left_brush_current_baseline_ /= sum_cnt_;
				ROS_INFO("%s %d: left_brush_current_baseline_:%d.", __FUNCTION__, __LINE__,
						 left_brush_current_baseline_);
				right_brush_current_baseline_ /= sum_cnt_;
				ROS_INFO("%s %d: right_brush_current_baseline_:%d.", __FUNCTION__, __LINE__,
						 right_brush_current_baseline_);
				main_brush_current_baseline_ /= sum_cnt_;
				ROS_INFO("%s %d: main_brush_current_baseline_:%d.", __FUNCTION__, __LINE__,
						 main_brush_current_baseline_);
				left_wheel_current_baseline_ /= sum_cnt_;
				ROS_INFO("%s %d: left_wheel_current_baseline_:%d.", __FUNCTION__, __LINE__,
						 left_wheel_current_baseline_);
				right_wheel_current_baseline_ /= sum_cnt_;
				ROS_INFO("%s %d: right_wheel_current_baseline_:%d.", __FUNCTION__, __LINE__,
						 right_wheel_current_baseline_);
				vacuum_current_baseline_ /= sum_cnt_;
				ROS_INFO("%s %d: vacuum_current_baseline_:%d.", __FUNCTION__, __LINE__, vacuum_current_baseline_);
				water_tank_current_baseline_ /= sum_cnt_;
				ROS_INFO("%s %d: water_tank_current_baseline_:%d.", __FUNCTION__, __LINE__,
						 water_tank_current_baseline_);
				robot_current_baseline_ /= sum_cnt_;
				ROS_INFO("%s %d: robot_current_baseline_:%d.", __FUNCTION__, __LINE__, robot_current_baseline_);
				sum_cnt_ = 0;

				int32_t side_brush_current_baseline_ref_{1620};
				int32_t main_brush_current_baseline_ref_{1620};
				int32_t wheel_current_baseline_ref_{1620};
				int32_t vacuum_current_baseline_ref_{1620};
				int32_t water_tank_current_baseline_ref_{0};
				int32_t robot_current_baseline_ref_{1775}; // todo:


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

				if (error_code_ != 0)
				{
					error_step_ = test_stage_;
					test_stage_ = 99;
				}
				else
				{
					check_wheel_time_ = ros::Time::now().toSec();
					test_stage_++;
					infrared_display.displayNormalMsg(test_stage_, 0);
				}
				/*start_time_stamp_ = ros::Time::now().toSec();
				left_brush_current_baseline_ = 0;
				right_brush_current_baseline_ = 0;
				main_brush_current_baseline_ = 0;
				left_wheel_current_baseline_ = 0;
				right_wheel_current_baseline_ = 0;
				vacuum_current_baseline_ = 0;
				robot_current_baseline_ = 0;*/
			}
			else
			{
				left_brush_current_baseline_ += brush.getLeftCurrent();
				right_brush_current_baseline_ += brush.getRightCurrent();
				main_brush_current_baseline_ += brush.getMainCurrent();
				left_wheel_current_baseline_ += wheel.getLeftCurrent();
				right_wheel_current_baseline_ += wheel.getRightCurrent();
				vacuum_current_baseline_ += vacuum.getCurrent();
				robot_current_baseline_ += robot::instance()->getCurrent();

//					printf("brush(l%d, r%d, m%d), wheel(l%d, r%d), vacuum(%d), robot(%d).\n",
//						   brush.getLeftCurrent(), brush.getRightCurrent(), brush.getMainCurrent(),
//						   wheel.getLeftCurrent(), wheel.getRightCurrent(), vacuum.getCurrent(),
//						   robot::instance()->getCurrent());
				sum_cnt_++;
			}
			break;
		}
		case 2:
		{
			infrared_display.displayNormalMsg(test_stage_,
											  static_cast<uint16_t>(ros::Time::now().toSec() - start_time_stamp_));
			if (ros::Time::now().toSec() - start_time_stamp_ > timeout_interval_ - 60)
			{
				// Last one minute will check for current. For wheels, check the forward current first.
				ROS_INFO("Start checking.");
				test_stage_++;
				infrared_display.displayNormalMsg(test_stage_, 0);
				brush.normalOperate(); // Update for battery.
				wheel.setPidTargetSpeed(LINEAR_MAX_SPEED, LINEAR_MAX_SPEED);
				check_wheel_time_ = ros::Time::now().toSec();
				wheel_forward_ = true;
			}
			else if (wheel_forward_ && ros::Time::now().toSec() - check_wheel_time_ > 30)
			{
				brush.normalOperate(); // Update for battery.
				wheel.setPidTargetSpeed(-LINEAR_MAX_SPEED, -LINEAR_MAX_SPEED);
				ROS_INFO("%s %d: Switch to backward.", __FUNCTION__, __LINE__);
				check_wheel_time_ = ros::Time::now().toSec();
				wheel_forward_ = false;
			} else if (!wheel_forward_ && ros::Time::now().toSec() - check_wheel_time_ > 30)
			{
				brush.normalOperate(); // Update for battery.
				wheel.setPidTargetSpeed(LINEAR_MAX_SPEED, LINEAR_MAX_SPEED);
				ROS_INFO("%s %d: Switch to forward.", __FUNCTION__, __LINE__);
				check_wheel_time_ = ros::Time::now().toSec();
				wheel_forward_ = true;
			}
			break;
		}
		case 3:
		{
			/*left_brush_current_max_ = (brush.getLeftCurrent() > left_brush_current_max_) ? brush.getLeftCurrent()
																						 : left_brush_current_max_;
			right_brush_current_max_ = (brush.getRightCurrent() > right_brush_current_max_) ? brush.getRightCurrent()
																							: right_brush_current_max_;
			main_brush_current_max_ = (brush.getMainCurrent() > main_brush_current_max_) ? brush.getMainCurrent()
																						 : main_brush_current_max_;
			vacuum_current_max_ = (vacuum.getCurrent() > vacuum_current_max_) ? vacuum.getCurrent() :
								  vacuum_current_max_;*/
			infrared_display.displayNormalMsg(test_stage_,
											  static_cast<uint16_t>(ros::Time::now().toSec() - start_time_stamp_));
			left_brush_current_ += brush.getLeftCurrent();
			right_brush_current_ += brush.getRightCurrent();
			main_brush_current_ += brush.getMainCurrent();
			vacuum_current_ += vacuum.getCurrent();
			robot_current_ += robot::instance()->getCurrent();
			sum_cnt_++;
			if (wheel_forward_)
			{
				/*left_wheel_forward_current_max_ = (wheel.getLeftCurrent() > left_wheel_forward_current_max_) ?
												  wheel.getLeftCurrent() : left_wheel_forward_current_max_;
				right_wheel_forward_current_max_ = (wheel.getRightCurrent() > right_wheel_forward_current_max_) ?
												   wheel.getRightCurrent() : right_wheel_forward_current_max_;*/
				left_wheel_forward_current_ += wheel.getLeftCurrent();
				right_wheel_forward_current_ += wheel.getRightCurrent();
				wheel_forward_current_cnt_++;
			} else
			{
				/*left_wheel_backward_current_max_ = (wheel.getLeftCurrent() > left_wheel_backward_current_max_) ?
												  wheel.getLeftCurrent() : left_wheel_backward_current_max_;
				right_wheel_backward_current_max_ = (wheel.getRightCurrent() > right_wheel_backward_current_max_) ?
												   wheel.getRightCurrent() : right_wheel_backward_current_max_;*/
				left_wheel_backward_current_ += wheel.getLeftCurrent();
				right_wheel_backward_current_ += wheel.getRightCurrent();
				wheel_backward_current_cnt_++;
			}

			if (wheel_forward_ && ros::Time::now().toSec() - check_wheel_time_ > 30)
			{
				// Change to check the wheel backward current.
				ROS_INFO("Change to check the wheel backward current.");
				brush.normalOperate(); // Update for battery.
				wheel.setPidTargetSpeed(-LINEAR_MAX_SPEED, -LINEAR_MAX_SPEED);
				wheel_forward_ = false;
			}
			else if (!wheel_forward_ && ros::Time::now().toSec() - start_time_stamp_ > timeout_interval_)
			{
				if (!checkCurrent())
				{
					error_step_ = test_stage_;
					test_stage_ = 99;
				}
				else
				{
					/*start_time_stamp_ = ros::Time::now().toSec();
					left_brush_current_baseline_ = 0;
					right_brush_current_baseline_ = 0;
					main_brush_current_baseline_ = 0;
					left_wheel_current_baseline_ = 0;
					right_wheel_current_baseline_ = 0;
					vacuum_current_baseline_ = 0;
					robot_current_baseline_ = 0;
					start_time_stamp_ += 5;*/
					test_stage_++;
				}
			}

			break;
		}
		case 99: // For error.
		{
			ROS_ERROR("%s %d: error code:%d", __FUNCTION__, __LINE__, error_code_);
			infrared_display.displayErrorMsg(error_step_, error_content_, error_code_);
			speaker.play(VOICE_TEST_FAIL);
			key_led.setMode(LED_STEADY, LED_RED);
			wheel.stop();
			brush.stop();
			vacuum.stop();
			water_tank.stop(WaterTank::tank_pump);
			sleep(5);
			break;
		}
		default: // case 4:
		{
			ROS_INFO("%s %d: Test finish.", __FUNCTION__, __LINE__);
			infrared_display.displayNormalMsg(0, 9999);
			speaker.play(VOICE_TEST_SUCCESS);
			key_led.setMode(LED_STEADY, LED_GREEN);
//			beeper.beep(2, 40, 40, 3);
			wheel.stop();
			brush.stop();
			vacuum.stop();
			water_tank.stop(WaterTank::tank_pump);
			sleep(5);
			break;
		}
	}

}

bool ActionLifeCheck::checkCurrent()
{

	/*if (left_brush_current_max_ - left_brush_current_baseline_ > side_brush_current_ref_)
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
		error_code_ = VACUUM_CURRENT_ERROR; // todo:*/

	uint16_t side_brush_current_ref_{1675 - 1620}; // 55
	uint16_t main_brush_current_ref_{1820 - 1620}; // 200
	uint16_t wheel_current_ref_{1685 - 1620}; // 65
	uint16_t vacuum_current_ref_{2285 - 1620}; // 665
	uint16_t water_tank_current_ref_{0};
	uint16_t robot_current_ref_{2385 - 1775}; // 610

//	wheel_current_ref_ = static_cast<uint16_t>(100.0 /*mA*/ / (330.0 /*3.3v reference voltage*/ / 0.1 /*Sampling resistance*/ / 4096.0 /*Accuracy*/));
	ROS_INFO("%s %d: Wheel current reference: %d.", __FUNCTION__, __LINE__, wheel_current_ref_);
//	side_brush_current_ref_ = static_cast<uint16_t>(110.0 /*mA*/ / (330.0 /*3.3v reference voltage*/ / 0.1 /*Sampling resistance*/ / 4096.0 /*Accuracy*/));
	ROS_INFO("%s %d: Side brush current reference: %d.", __FUNCTION__, __LINE__, side_brush_current_ref_);
//	main_brush_current_ref_ = static_cast<uint16_t>(300.0 /*mA*/ / (330.0 /*3.3v reference voltage*/ / 0.1 /*Sampling resistance*/ / 4096.0 /*Accuracy*/));
	ROS_INFO("%s %d: Main brush current reference: %d.", __FUNCTION__, __LINE__, main_brush_current_ref_);
//	vacuum_current_ref_ = static_cast<uint16_t>(1100.0 /*mA*/ / (330.0 /*3.3v reference voltage*/ / 0.1 /*Sampling resistance*/ / 4096.0 /*Accuracy*/));
	ROS_INFO("%s %d: Vacuum current reference: %d.", __FUNCTION__, __LINE__, vacuum_current_ref_);
//	water_tank_current_ref_ = static_cast<uint16_t>(1100.0 /*mA*/ / (330.0 /*3.3v reference voltage*/ / 0.1 /*Sampling resistance*/ / 4096.0 /*Accuracy*/));
	ROS_INFO("%s %d: Water tank current reference: %d.", __FUNCTION__, __LINE__, water_tank_current_ref_);
//	robot_current_ref_ = static_cast<uint16_t>(1100.0 /*mA*/ / (330.0 /*3.3v reference voltage*/ / 0.1 /*Sampling resistance*/ / 4096.0 /*Accuracy*/));
	ROS_INFO("%s %d: Robot current reference: %d.", __FUNCTION__, __LINE__, robot_current_ref_);

	left_brush_current_ /= sum_cnt_;
	ROS_INFO("%s %d: Left brush current: %d.", __FUNCTION__, __LINE__, left_brush_current_ - left_brush_current_baseline_);
	right_brush_current_ /= sum_cnt_;
	ROS_INFO("%s %d: Right brush current: %d.", __FUNCTION__, __LINE__, right_brush_current_ - right_brush_current_baseline_);
	main_brush_current_ /= sum_cnt_;
	ROS_INFO("%s %d: Main brush current: %d.", __FUNCTION__, __LINE__, main_brush_current_ - main_brush_current_baseline_);
	left_wheel_forward_current_ /= wheel_forward_current_cnt_;
	ROS_INFO("%s %d: Left wheel forward current: %d.", __FUNCTION__, __LINE__, left_wheel_forward_current_ - left_wheel_current_baseline_);
	right_wheel_forward_current_ /= wheel_forward_current_cnt_;
	ROS_INFO("%s %d: Right wheel forward current: %d.", __FUNCTION__, __LINE__, right_wheel_forward_current_ - right_wheel_current_baseline_);
	left_wheel_backward_current_ /= wheel_backward_current_cnt_;
	ROS_INFO("%s %d: Left wheel backward current: %d.", __FUNCTION__, __LINE__, left_wheel_backward_current_ - left_wheel_current_baseline_);
	right_wheel_backward_current_ /= wheel_backward_current_cnt_;
	ROS_INFO("%s %d: Right wheel backward current: %d.", __FUNCTION__, __LINE__, right_wheel_backward_current_ - right_wheel_current_baseline_);
	vacuum_current_ /= sum_cnt_;
	ROS_INFO("%s %d: Vacuum current: %d.", __FUNCTION__, __LINE__, vacuum_current_ - vacuum_current_baseline_);
	water_tank_current_ /= sum_cnt_;
	ROS_INFO("%s %d: Water tank current: %d.", __FUNCTION__, __LINE__, water_tank_current_ - water_tank_current_baseline_);
	robot_current_ /= sum_cnt_;
	ROS_INFO("%s %d: Robot current: %d.", __FUNCTION__, __LINE__, robot_current_ - robot_current_baseline_);

	if (left_brush_current_ - left_brush_current_baseline_ > side_brush_current_ref_ * 1.5 ||
			 left_brush_current_ - left_brush_current_baseline_ < side_brush_current_ref_ * 0.5)
	{
		error_code_ = LEFT_BRUSH_CURRENT_ERROR;
		error_content_ = static_cast<uint16_t>(left_brush_current_ - left_brush_current_baseline_);
	}
	else if (right_brush_current_ - right_brush_current_baseline_ > side_brush_current_ref_ * 1.5 ||
		right_brush_current_ - right_brush_current_baseline_ < side_brush_current_ref_ * 0.5)
	{
		error_code_ = RIGHT_BRUSH_CURRENT_ERROR;
		error_content_ = static_cast<uint16_t>(right_brush_current_ - right_brush_current_baseline_);
	}
	else if (main_brush_current_ - main_brush_current_baseline_ > main_brush_current_ref_ * 1.5 ||
			 main_brush_current_ - main_brush_current_baseline_ < main_brush_current_ref_ * 0.5)
	{
		error_code_ = MAIN_BRUSH_CURRENT_ERROR;
		error_content_ = static_cast<uint16_t>(main_brush_current_ - main_brush_current_baseline_);
	}
	else if (vacuum_current_ - vacuum_current_baseline_ > vacuum_current_ref_ * 1.5 ||
			 vacuum_current_ - vacuum_current_baseline_ < vacuum_current_ref_ * 0.2)
	{
		error_code_ = VACUUM_CURRENT_ERROR;
		error_content_ = static_cast<uint16_t>(vacuum_current_ - vacuum_current_baseline_);
	}
	/*else if (water_tank_current_ - water_tank_current_baseline_ > water_tank_current_ref_ * 1.5 ||
			 water_tank_current_ - water_tank_current_baseline_ < water_tank_current_ref_ * 0.5)
	{
		error_code_ = VACUUM_CURRENT_ERROR; // todo:
		error_content_ = static_cast<uint16_t>(water_tank_current_ - water_tank_current_baseline_);
	}*/
	else if (left_wheel_forward_current_ - left_wheel_current_baseline_ > wheel_current_ref_ * 1.5 ||
		left_wheel_forward_current_ - left_wheel_current_baseline_ < wheel_current_ref_ * 0.5)
	{
		error_code_ = LEFT_WHEEL_FORWARD_CURRENT_ERROR;
		error_content_ = static_cast<uint16_t>(left_wheel_forward_current_ - left_wheel_current_baseline_);
	}
	else if (right_wheel_forward_current_ - right_wheel_current_baseline_ > wheel_current_ref_ * 1.5 ||
			 right_wheel_forward_current_ - right_wheel_current_baseline_ < wheel_current_ref_ * 0.5)
	{
		error_code_ = RIGHT_WHEEL_FORWARD_CURRENT_ERROR;
		error_content_ = static_cast<uint16_t>(right_wheel_forward_current_ - right_wheel_current_baseline_);
	}
	else if (left_wheel_backward_current_ - left_wheel_current_baseline_ > wheel_current_ref_ * 1.5 ||
		left_wheel_backward_current_ - left_wheel_current_baseline_ < wheel_current_ref_ * 0.5)
	{
		error_code_ = LEFT_WHEEL_BACKWARD_CURRENT_ERROR;
		error_content_ = static_cast<uint16_t>(left_wheel_backward_current_ - left_wheel_current_baseline_);
	}
	else if (right_wheel_backward_current_ - right_wheel_current_baseline_ > wheel_current_ref_ * 1.5 ||
			 right_wheel_backward_current_ - right_wheel_current_baseline_ < wheel_current_ref_ * 0.5)
	{
		error_code_ = RIGHT_WHEEL_BACKWARD_CURRENT_ERROR;
		error_content_ = static_cast<uint16_t>(right_wheel_backward_current_ - right_wheel_current_baseline_);
	}
	else if (robot_current_ - robot_current_baseline_ > robot_current_ref_ * 1.5 ||
			robot_current_ - robot_current_baseline_ < robot_current_ref_ * 0.5)
	{
		error_code_ = BASELINE_CURRENT_ERROR;
		error_content_ = static_cast<uint16_t>(robot_current_ - robot_current_baseline_);
	}

	return error_code_ == 0;
}
