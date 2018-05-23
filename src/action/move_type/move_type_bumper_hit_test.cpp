//
// Created by austin on 17-12-21.
//

#include <robot.hpp>
#include <move_type.hpp>
#include <bumper.h>
#include <key_led.h>
#include <wifi_led.hpp>
#include <wheel.hpp>

MoveTypeBumperHitTest::MoveTypeBumperHitTest()
{
	ROS_WARN("%s,%d: Move type is bumper hit test.", __FUNCTION__, __LINE__);
	p_direct_go_movement_.reset(new MovementDirectGo(false));
	p_back_movement_.reset();
	p_turn_movement_.reset();

}

bool MoveTypeBumperHitTest::isFinish()
{
	return false;
}

void MoveTypeBumperHitTest::run()
{
	if (bumper_error)
	{
		if (bumper.getStatus() == BLOCK_LEFT)
			key_led.setMode(LED_STEADY, LED_GREEN);
		else if (bumper.getStatus() == BLOCK_RIGHT)
			key_led.setMode(LED_STEADY, LED_RED);
		else if (bumper.getStatus() == BLOCK_ALL)
			key_led.setMode(LED_STEADY, LED_ORANGE);
		else
			key_led.setMode(LED_STEADY, LED_OFF);

		if (bumper.getStatus() == BLOCK_LIDAR_BUMPER)
			wifi_led.setMode(LED_STEADY, WifiLed::state::on);
		else
			wifi_led.setMode(LED_STEADY, WifiLed::state::off);
	}

	else if (p_back_movement_ != nullptr)
	{
		if (ev.bumper_jam || ev.lidar_bumper_jam)
		{
			bumper_error = true;
			wheel.stop();
			return;
		}

		if (p_back_movement_->isFinish())
		{
			p_back_movement_.reset();
			turn_time_stamp_ = ros::Time::now().toSec();
		}
		else
			p_back_movement_->run();
	}
	else if (p_turn_movement_ != nullptr)
	{
		if (ros::Time::now().toSec() - turn_time_stamp_ > 0.6)
			p_turn_movement_.reset();
		else
			p_turn_movement_->run();
	}
	else if (bumper.getStatus())
	{
		float back_distance = 0.08;
		p_back_movement_.reset(new MovementBack(back_distance, 40));
		if (turn_left_)
			turn_target_angle_ = static_cast<int16_t>(getPosition().addRadian(degree_to_radian(90)).th);
		else
			turn_target_angle_ = static_cast<int16_t>(getPosition().addRadian(degree_to_radian(-90)).th);
		p_turn_movement_.reset(new MovementTurn(turn_target_angle_, 40));
		turn_left_ = !turn_left_;
	} else
		p_direct_go_movement_->run();
}
