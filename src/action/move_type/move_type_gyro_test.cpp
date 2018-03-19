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
#include "move_type.hpp"

MoveTypeGyroTest::MoveTypeGyroTest()
{
	ROS_INFO("%s,%d: Enter move type gyro test.", __FUNCTION__, __LINE__);

	key_led.setMode(LED_FLASH, LED_GREEN);
	p_movement_.reset(new ActionOpenGyro());
	last_time_stamp_ = ros::Time::now().toSec();
}

MoveTypeGyroTest::~MoveTypeGyroTest()
{
	ROS_INFO("%s,%d: Exit move type gyro test.", __FUNCTION__, __LINE__);
}

void MoveTypeGyroTest::run()
{
	switch (test_stage_)
	{
		case 0:
		{
			if (p_movement_->isFinish())
			{
				test_stage_++;
				p_movement_.reset();
				p_movement_.reset(new MovementTurn(getPosition().th + degree_to_radian(90), 10));
				saved_gyro_turn_angle_ = gyro.getAngleY();
				wheel_mileage_ = 0;
				brush.slowOperate();
			}
			else
				p_movement_->run();
			break;
		}
		case 1:
		case 2:
		case 3:
		case 4:
		{
			/*wheel.pidSetLeftSpeed(-5);
			wheel.pidSetRightSpeed(5);
			wheel_turn_angle_ = radian_to_degree(ranged_radian(wheel_mileage_ / 2 / WHEEL_TO_CENTER_DISTANCE));
			ROS_INFO("%s %d: Turn for %d times, wheel_turn_angle_:%f(%f in angle).",
					 __FUNCTION__, __LINE__, test_stage_, degree_to_radian(wheel_turn_angle_), wheel_turn_angle_);

			auto current_angle = gyro.getAngleY();
			auto gyro_diff = current_angle - saved_gyro_turn_angle_;
			ROS_INFO("%s %d: gyro_diff:%f(%f in angle).", __FUNCTION__, __LINE__, degree_to_radian(gyro_diff),
					 gyro_diff);
			double current_time = ros::Time::now().toSec();
			double dt = current_time - last_time_stamp_;
			wheel_mileage_ +=
					(std::fabs(wheel.getLeftWheelActualSpeed()) + std::fabs(wheel.getRightWheelActualSpeed())) * dt;
			last_time_stamp_ = current_time;*/

			if (p_movement_->isFinish())
			{
				wheel_turn_angle_ = radian_to_degree(wheel_mileage_ / 2 / WHEEL_TO_CENTER_DISTANCE);
				ROS_INFO("%s %d: Turn for %d times, wheel_turn_angle_:%f(%f in angle).",
						 __FUNCTION__, __LINE__, test_stage_, degree_to_radian(wheel_turn_angle_), wheel_turn_angle_);

				auto current_angle = gyro.getAngleY();
				auto gyro_diff =  current_angle - saved_gyro_turn_angle_;
				ROS_INFO("%s %d: gyro_diff:%f(%f in angle).", __FUNCTION__, __LINE__, degree_to_radian(gyro_diff), gyro_diff);

				if (fabs(ranged_radian(wheel_turn_angle_ - gyro_diff)) > 1)
				{
					error_code_ = GYRO_ERROR;
					test_stage_ = 99;
				}
				else
				{
					wheel_mileage_ = 0;
					p_movement_.reset();
					if (test_stage_ == 4)
						p_movement_.reset();
					else
						p_movement_.reset(new MovementTurn(getPosition().th + degree_to_radian(90), 10));
					test_stage_++;
					current_angle = gyro.getAngleY();
					saved_gyro_turn_angle_ = current_angle;
				}
			}
			else
			{
				double current_time = ros::Time::now().toSec();
				double dt = current_time - last_time_stamp_;
				wheel_mileage_ +=
						(std::fabs(wheel.getLeftWheelActualSpeed()) + std::fabs(wheel.getRightWheelActualSpeed())) * dt;

				last_time_stamp_ = current_time;
				p_movement_->run();
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
