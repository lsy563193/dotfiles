//
// Created by lsy563193 on 1/2/18.
//

#include <mode.hpp>
#include <robot.hpp>
#include <water_tank.hpp>
#include <brush.h>
#include <gyro.h>
#include <charger.h>
#include "wifi/wifi.h"

#include "key_led.h"

void StateInit::init() {
	if(Mode::next_mode_i_ == Mode::md_go_to_charger)
		initForGoToCharger();
	else if(Mode::next_mode_i_ == Mode::cm_exploration)
		initForExploration();
	else if (Mode::next_mode_i_ == Mode::cm_navigation)
		initForNavigation();
	else if(Mode::next_mode_i_ == Mode::cm_spot)
		initForSpot();
	else //For wallFollow and remote mode
		initForCommonMode();

	s_wifi.setWorkMode(Mode::next_mode_i_);
	s_wifi.taskPushBack(S_Wifi::ACT::ACT_UPLOAD_STATUS);
	ROS_INFO("%s %d: Enter state init.", __FUNCTION__, __LINE__);
}

void StateInit::initForNavigation() {
	if(sp_cm_->isRemoteGoHomePoint() || sp_cm_->isWifiGoHomePoint())
		key_led.setMode(LED_STEADY, LED_ORANGE);
	else
		key_led.setMode(LED_STEADY, LED_GREEN);

	if(sp_cm_->action_i_ == sp_cm_->ac_back_from_charger)
		brush.slowOperate();
	else if(!charger.isOnStub())
	{
		brush.normalOperate();
		gyro.setTiltCheckingEnable(true);
	}
	water_tank.setCurrentSwingMotorMode(WaterTank::SWING_MOTOR_LOW);
	water_tank.checkEquipment() ? water_tank.open(WaterTank::operate_option::swing_motor)
															: vacuum.setSpeedByUserSetMode();
	sp_cm_->isUsingDustBox(!water_tank.getStatus(WaterTank::operate_option::swing_motor));
	ROS_INFO("%s %d: Enter state initForNavigation.", __FUNCTION__, __LINE__);
}

void StateInit::initForExploration() {
	key_led.setMode(LED_STEADY, LED_ORANGE, 600);
	brush.slowOperate();
	water_tank.setCurrentSwingMotorMode(WaterTank::SWING_MOTOR_LOW);
	water_tank.checkEquipment() ? water_tank.open(WaterTank::operate_option::swing_motor)
								: vacuum.setForCurrentMode(Vacuum::VacMode::vac_low_mode);
	sp_cm_->isUsingDustBox(!water_tank.getStatus(WaterTank::operate_option::swing_motor));
	gyro.setTiltCheckingEnable(true);
	ROS_INFO("%s %d: Enter state initForExploration.", __FUNCTION__, __LINE__);
}

void StateInit::initForCommonMode() {
	key_led.setMode(LED_STEADY, LED_GREEN);
	brush.normalOperate();
	water_tank.setCurrentSwingMotorMode(water_tank.getUserSetSwingMotorMode());
	water_tank.checkEquipment() ? water_tank.open(WaterTank::operate_option::swing_motor)
															: vacuum.setSpeedByUserSetMode();
	sp_cm_->isUsingDustBox(!water_tank.getStatus(WaterTank::operate_option::swing_motor));
	gyro.setTiltCheckingEnable(true);
	ROS_INFO("%s %d: Enter state initForCommonMode.", __FUNCTION__, __LINE__);
}

void StateInit::initForSpot() {
	key_led.setMode(LED_STEADY, LED_GREEN, 600);
	brush.fullOperate();
	water_tank.setCurrentSwingMotorMode(WaterTank::SWING_MOTOR_HIGH);
	water_tank.checkEquipment() ? water_tank.open(WaterTank::operate_option::swing_motor)
															: vacuum.setForCurrentMode(Vacuum::VacMode::vac_max_mode);
	sp_cm_->isUsingDustBox(!water_tank.getStatus(WaterTank::operate_option::swing_motor));
	gyro.setTiltCheckingEnable(true);
	ROS_INFO("%s %d: Enter state initForSpot.", __FUNCTION__, __LINE__);
}

void StateInit::initForGoToCharger() {
	key_led.setMode(LED_STEADY, LED_ORANGE);
	brush.slowOperate();
	water_tank.setCurrentSwingMotorMode(WaterTank::SWING_MOTOR_LOW);
	water_tank.checkEquipment() ? water_tank.open(WaterTank::operate_option::swing_motor)
															: vacuum.setForCurrentMode(Vacuum::VacMode::vac_low_mode);
	gyro.setTiltCheckingEnable(false); //disable tilt detect
	ROS_INFO("%s %d: Enter state initForGoToCharger.", __FUNCTION__, __LINE__);
}
