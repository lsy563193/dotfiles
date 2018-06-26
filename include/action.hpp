//
// Created by austin on 17-12-3.
//

#ifndef PP_ACTION_H
#define PP_ACTION_H

#include <cstdint>
#include <time.h>
#include "odom.h"
#include "ros/ros.h"
#include "mathematics.h"

class IAction{
public:
	IAction();
	virtual ~IAction() = default;

	virtual bool isFinish()=0;

	virtual bool isExit();

	virtual bool isTimeUp();

	virtual void run()=0;

	void updateStartTime();

	// For test mode.
	virtual void dataExtract(){};

	double start_timer_;
protected:
	double timeout_interval_{};
};

class ActionOpenGyro :public IAction
{
public:
	ActionOpenGyro();
	~ActionOpenGyro();
	bool isFinish() override;
	void run();

private:
	bool gyro_closed_{false};
	bool had_set_brush_slow_{false};
	bool had_set_brush_stop_{false};
	const uint8_t set_brush_slow_count_{2};
	const uint8_t set_brush_stop_count_{4};
};

class ActionOpenGyroAndLidar : public ActionOpenGyro
{
public:
	ActionOpenGyroAndLidar();
	~ActionOpenGyroAndLidar();
};

class ActionBackFromCharger :public IAction
{
public:
	ActionBackFromCharger();
	~ActionBackFromCharger();
	bool isFinish() override ;
	void run() override ;
private:
	enum {
		BACK = 0,
		TURN,
		FORWARD,
		BUMPER_BACK,
	};
	Point_t start_point_{odom.getOriginX(), odom.getOriginY(), odom.getRadian()};
	const float BACK_DIST = 0.15f;
	const float FORWARD_DIST = 0.35f;
	uint8_t flag_{BACK};
};

class ActionOpenLidar :public IAction
{
public:
	ActionOpenLidar();
	~ActionOpenLidar();
	bool isFinish() override;
	bool isTimeUp() override;
	void run() override;
};

class ActionAlign :public IAction
{
public:
	explicit ActionAlign();
	~ActionAlign();
	bool isFinish() override;
	bool isTimeUp() override;
	void run() override;

private:
	double align_radian = 0.0;
	double distance;
	bool isLeft = true;
	double wait_laser_timer_{};
};

class ActionOpenSlam :public IAction
{
public:
	ActionOpenSlam();
	~ActionOpenSlam();
	bool isFinish();
	void run();
private:
	double wait_slam_timer_{1.0};
	bool is_slam_start_{false};
};

class ActionSleep :public IAction
{
public:
	ActionSleep(bool fake_sleep);
	~ActionSleep() override ;
	bool isFinish() override ;
	void run();

	void lowPowerSleep();
private:
};


class ActionIdle :public IAction
{
public:
	ActionIdle();
	~ActionIdle() override;
	bool isFinish() override;
	bool isTimeUp() override;
	void run() override;
private:
	int error_alarm_cnt_{0};
	double error_alarm_time_{0};
	int trapped_alarm_cnt_{0};
	double trapped_alarm_time_{0};
};

class ActionPause :public IAction
{
public:
	ActionPause();
	~ActionPause() override;
	bool isFinish() override;
	bool isExit() override;
	void run() override;

private:
	double pause_start_time_;
	Pose pause_pose_;
};

class ActionCheckVacuum: public IAction
{
public:
	ActionCheckVacuum();
	~ActionCheckVacuum() override = default;

	bool isFinish() override;
	bool isExit() override;

	void run() override;
};

class ActionCheckBumper: public IAction
{
public:
	ActionCheckBumper();
	~ActionCheckBumper() override = default;

	bool isFinish() override;
	bool isExit() override;

	void run() override;
};

class ActionCheckWaterTank: public IAction
{
public:
	ActionCheckWaterTank();
	~ActionCheckWaterTank() override = default;

	bool isFinish() override
	{return false;};
	bool isExit() override
	{return false;};

	void waterTankTestRoutineThread();

	void run() override;

	bool dataExtract(const uint8_t *buf);

private:

	uint16_t error_code_{0};
	uint16_t error_content_{0};
	uint8_t error_step_{0};
	uint8_t test_stage_{1};
	double check_time_{0};
	uint32_t swing_motor_current_baseline_{0};
	uint32_t swing_motor_current_{0};
	int sum_cnt_{0};
};

class ActionLifeCheck: public IAction
{
public:
	ActionLifeCheck();
	~ActionLifeCheck() override = default;

	bool isFinish() override
	{return false;};

	bool isExit() override
	{return false;};

	void lifeTestRoutineThread();

	void run() override;

	bool dataExtract(const uint8_t *buf);

private:

	bool checkCurrent();

	double start_time_stamp_{0};
	double check_wheel_time_{0};
	uint16_t error_code_{0};
	uint16_t error_content_{0};
	uint8_t error_step_{0};
	uint8_t test_stage_{1};
	bool wheel_forward_{true};

	int sum_cnt_{0};
	uint16_t wheel_forward_current_cnt_{0};
	uint16_t wheel_backward_current_cnt_{0};

	uint32_t left_brush_current_baseline_{0};
	uint32_t right_brush_current_baseline_{0};
	uint32_t main_brush_current_baseline_{0};
	uint32_t left_wheel_current_baseline_{0};
	uint32_t right_wheel_current_baseline_{0};
	uint32_t vacuum_current_baseline_{0};
	uint32_t water_tank_current_baseline_{0};
	uint32_t robot_current_baseline_{0};

	uint32_t left_brush_current_{0};
	uint32_t right_brush_current_{0};
	uint32_t main_brush_current_{0};
	uint32_t left_wheel_forward_current_{0};
	uint32_t right_wheel_forward_current_{0};
	uint32_t left_wheel_backward_current_{0};
	uint32_t right_wheel_backward_current_{0};
	uint32_t vacuum_current_{0};
	uint32_t water_tank_current_{0};
	uint32_t robot_current_{0};

	/*uint32_t left_brush_current_max_{0};
	uint32_t right_brush_current_max_{0};
	uint32_t main_brush_current_max_{0};
	uint32_t left_wheel_forward_current_max_{0};
	uint32_t right_wheel_forward_current_max_{0};
	uint32_t left_wheel_backward_current_max_{0};
	uint32_t right_wheel_backward_current_max_{0};
	uint32_t vacuum_current_max_{0};
	uint32_t water_tank_current_max_{0};
	uint32_t robot_current_max_{0};*/
};

class ActionR16Test: public IAction
{
public:
	ActionR16Test();

	~ActionR16Test() override = default;

	bool isFinish() override
	{ return false; };

	bool isExit() override
	{ return false; };

	void run() override;

private:

	int test_stage_{0};

/*
 * Test RAM.
 */
	bool RAM_test();

/*
 * Test flash.
 */
	bool Flash_test();

/*
 * Test lidar.
 */
	bool lidar_test();

/*
 * Test lidar bumper.
 */
	uint8_t bumper_cnt_{0};
	bool lidar_bumper_test();

/*
 * Test wifi module.
 */
	bool wifi_test();

	void error_loop(uint8_t step, uint16_t content, uint16_t error_code);
};
#endif //PP_ACTION_H
