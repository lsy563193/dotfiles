//
// Created by lsy563193 on 1/2/18.
//

#include <mode.hpp>
#include <robot.hpp>
#include <water_tank.hpp>
#include <brush.h>
#include "wifi/wifi.h"

#include "key_led.h"

void StateInit::init() {
	if(Mode::next_mode_i_ == Mode::cm_exploration || Mode::next_mode_i_ == Mode::md_go_to_charger)
		key_led.setMode(LED_STEADY, LED_ORANGE);
	else if (Mode::next_mode_i_ == Mode::cm_navigation && sp_cm_->isRemoteGoHomePoint())
		key_led.setMode(LED_STEADY, LED_ORANGE);
	else
		key_led.setMode(LED_STEADY, LED_GREEN);

	s_wifi.setWorkMode(Mode::next_mode_i_);
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
	ROS_INFO("%s %d: Enter state init.", __FUNCTION__, __LINE__);
}

void StateInit::initOpenLidar()
{
	key_led.setMode(LED_STEADY, LED_GREEN);
	brush.normalOperate();
	water_tank.setCurrentSwingMotorMode(water_tank.getUserSetSwingMotorMode());
	water_tank.checkEquipment() ? water_tank.open(WaterTank::operate_option::swing_motor)
								: vacuum.setSpeedByUserSetMode();
	sp_cm_->isUsingDustBox(!water_tank.getStatus(WaterTank::operate_option::swing_motor));
	ROS_INFO("%s %d: Enter state initOpenLidar.", __FUNCTION__, __LINE__);
}

void StateInit::initForNavigation()
{
	if (sp_cm_->isRemoteGoHomePoint())
	{
		key_led.setMode(LED_STEADY, LED_ORANGE);
		brush.slowOperate();
		water_tank.setCurrentSwingMotorMode(WaterTank::SWING_MOTOR_LOW);
		water_tank.checkEquipment() ? water_tank.open(WaterTank::operate_option::swing_motor)
									: vacuum.setSpeedByUserSetMode();
		sp_cm_->isUsingDustBox(!water_tank.getStatus(WaterTank::operate_option::swing_motor));
		ROS_INFO("%s %d: Enter state initForNavigation.", __FUNCTION__, __LINE__);
	} else
	{
		ROS_INFO("%s %d: Just init for lidar.", __FUNCTION__, __LINE__);
		initOpenLidar();
	}
}

void StateInit::initBackFromCharger() {
	key_led.setMode(LED_STEADY, LED_GREEN, 600);
	brush.slowOperate();
	water_tank.setCurrentSwingMotorMode(WaterTank::SWING_MOTOR_LOW);
	water_tank.checkEquipment() ? water_tank.open(WaterTank::operate_option::swing_motor)
								: vacuum.setSpeedByUserSetMode();
	sp_cm_->isUsingDustBox(!water_tank.getStatus(WaterTank::operate_option::swing_motor));
	ROS_INFO("%s %d: Enter state initBackFromCharger.", __FUNCTION__, __LINE__);
}

void StateInit::initForExploration() {
	key_led.setMode(LED_STEADY, LED_ORANGE, 600);
	brush.slowOperate();
	water_tank.setCurrentSwingMotorMode(WaterTank::SWING_MOTOR_LOW);
	water_tank.checkEquipment() ? water_tank.open(WaterTank::operate_option::swing_motor)
								: vacuum.setForCurrentMaxMode(false);
	sp_cm_->isUsingDustBox(!water_tank.getStatus(WaterTank::operate_option::swing_motor));
	ROS_INFO("%s %d: Enter state initForExploration.", __FUNCTION__, __LINE__);
}

void StateInit::initForSpot()
{
	key_led.setMode(LED_STEADY, LED_GREEN, 600);
	brush.fullOperate();
	water_tank.setCurrentSwingMotorMode(WaterTank::SWING_MOTOR_HIGH);
	water_tank.checkEquipment() ? water_tank.open(WaterTank::operate_option::swing_motor)
								: vacuum.setForCurrentMaxMode(true);
	sp_cm_->isUsingDustBox(!water_tank.getStatus(WaterTank::operate_option::swing_motor));
	ROS_INFO("%s %d: Enter state initForSpot.", __FUNCTION__, __LINE__);
}
