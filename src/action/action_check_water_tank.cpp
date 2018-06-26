//
// Created by austin on 18-3-14.
//

#include <water_tank.hpp>
#include <robot.hpp>
#include <infrared_display.hpp>
#include <error.h>
#include <speaker.h>
#include <key_led.h>

ActionCheckWaterTank::ActionCheckWaterTank()
{
	ROS_WARN("%s %d: Start." , __FUNCTION__, __LINE__);

	auto swing_motor_test_routine = new boost::thread(boost::bind(&ActionCheckWaterTank::waterTankTestRoutineThread, this));

	infrared_display.displayNormalMsg(test_stage_, 0);
	check_time_ = ros::Time::now().toSec();
	timeout_interval_ = 16;
}

void ActionCheckWaterTank::waterTankTestRoutineThread()
{
	ROS_WARN("\033[32m%s\033[0m,%d is up.",__FUNCTION__,__LINE__);

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

bool ActionCheckWaterTank::dataExtract(const uint8_t *buf)
{
	//	serial.debugReceivedStream(buf);

	// work mode
	auto current_work_mode = buf[REC_WORK_MODE];
	if (current_work_mode != serial.getSendData(CTL_WORK_MODE))
		return false;

	water_tank.setSwingMotorCurrent((buf[REC_WATER_PUMP_CURRENT_H] << 8) | buf[REC_WATER_PUMP_CURRENT_L]);
	printf("swing_motor(%d).\n", water_tank.getSwingMotorCurrent());

	return true;
}

void ActionCheckWaterTank::run()
{
	switch (test_stage_)
	{
		case 1:
		{
			if (ros::Time::now().toSec() - check_time_ > 1)
			{
				int32_t swing_motor_current_baseline_ref_{1610};
				auto average_current = swing_motor_current_baseline_ / sum_cnt_;
				ROS_INFO("%s %d: Swing motor average current baseline: %d.", __FUNCTION__, __LINE__, average_current);
				if (average_current > swing_motor_current_baseline_ref_ * 1.4
						|| average_current < swing_motor_current_baseline_ref_ * 0.8)
				{
					error_code_ = SWING_MOTOR_ERROR;
					error_content_ = static_cast<uint16_t>(average_current);
					error_step_ = test_stage_;
					test_stage_ = 99;
				}
				else
				{
					water_tank.open(WaterTank::operate_option::pump);
					check_time_ = ros::Time::now().toSec();
					sum_cnt_ = 0;
					swing_motor_current_baseline_ = average_current;
					test_stage_++;
					infrared_display.displayNormalMsg(test_stage_, static_cast<uint16_t>(average_current));
				}
			} else
			{
				swing_motor_current_baseline_ += water_tank.getSwingMotorCurrent();
				sum_cnt_++;
			}
			break;
		}
		case 2:
		{
			if (ros::Time::now().toSec() - check_time_ > 5)
			{
				water_tank.stop(WaterTank::operate_option::pump);
				water_tank.open(WaterTank::operate_option::swing_motor);
				check_time_ = ros::Time::now().toSec();
				test_stage_++;
				infrared_display.displayNormalMsg(test_stage_, 0);
			}
			break;
		}
		case 3:
		{
			if (ros::Time::now().toSec() - check_time_ > 5)
			{
				int32_t swing_motor_current_ref_{1500};
				auto average_current = swing_motor_current_ / sum_cnt_;
				ROS_INFO("%s %d: Swing motor average current: %d.", __FUNCTION__, __LINE__, average_current);
				if (average_current > swing_motor_current_ref_ * 1.3
						|| average_current < swing_motor_current_ref_ * 0.55) // 825
				{
					error_code_ = SWING_MOTOR_ERROR;
					error_content_ = static_cast<uint16_t>(average_current);
					error_step_ = test_stage_;
					test_stage_ = 99;
				}
				else
				{
					check_time_ = ros::Time::now().toSec();
					swing_motor_current_ = 0;
					sum_cnt_ = 0;
				}
			}
			else if (isTimeUp())
				test_stage_++;
			else
			{
				swing_motor_current_ += (water_tank.getSwingMotorCurrent() - swing_motor_current_baseline_);
				sum_cnt_++;
//				printf("swing motor current sum:%d, sum_cnt:%d.\n", swing_motor_current_, sum_cnt_);
			}
			break;
		}
		case 99: // For error.
		{
			ROS_ERROR("%s %d: error code:%d", __FUNCTION__, __LINE__, error_code_);
			infrared_display.displayErrorMsg(error_step_, error_content_, error_code_);
			speaker.play(VOICE_TEST_FAIL);
			key_led.setMode(LED_STEADY, LED_RED);
			water_tank.stop(WaterTank::operate_option::swing_motor_and_pump);
			sleep(5);
			break;
		}
		default: // case 4:
		{
			ROS_WARN("%s %d: Test finish.", __FUNCTION__, __LINE__);
			infrared_display.displayNormalMsg(0, 9999);
			speaker.play(VOICE_TEST_SUCCESS);
			key_led.setMode(LED_STEADY, LED_GREEN);
//			beeper.beep(2, 40, 40, 3);
			water_tank.stop(WaterTank::operate_option::swing_motor_and_pump);
			sleep(5);
			break;
		}

	}

}
