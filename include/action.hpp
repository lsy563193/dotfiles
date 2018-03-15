//
// Created by austin on 17-12-3.
//

#ifndef PP_ACTION_H
#define PP_ACTION_H

#include <cstdint>
#include <time.h>
#include "odom.h"
#include "water_tank.hpp"

class IAction{
public:
	IAction();
	virtual ~IAction() = default;

	virtual bool isFinish()=0;

	virtual bool isExit();

	virtual bool isTimeUp();

	virtual void run()=0;

	// For test mode.
	virtual void dataExtract(){};
protected:
	double start_timer_{};
	double timeout_interval_{};
};

class ActionOpenGyro :public IAction
{
public:
	ActionOpenGyro();
	bool isFinish() override;
	void run();

private:
	bool gyro_closed_{false};
};

class ActionBackFromCharger :public IAction
{
public:
	ActionBackFromCharger();
	~ActionBackFromCharger();
	bool isFinish();
	void run();
};

class ActionOpenLidar :public IAction
{
public:
	ActionOpenLidar();
	bool isFinish() override;
	bool isTimeUp() override;
	void run() override;
};

class ActionAlign :public IAction
{
public:
	ActionAlign();
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
	bool isFinish();
	void run();
private:
	double wait_slam_timer_{1.0};
	bool is_slam_start_{false};
};

class ActionSleep :public IAction
{
public:
	ActionSleep();
	~ActionSleep() override ;
	bool isFinish() override ;
	void run();
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
};

class ActionPause :public IAction
{
public:
	ActionPause();
	~ActionPause();
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
	~ActionCheckVacuum() = default;

	bool isFinish() override;
	bool isExit() override;

	void run() override;
};

class ActionCheckBumper: public IAction
{
public:
	ActionCheckBumper();
	~ActionCheckBumper() = default;

	bool isFinish() override;
	bool isExit() override;

	void run() override;
};

class ActionCheckWaterTank: public IAction
{
public:
	ActionCheckWaterTank();
	~ActionCheckWaterTank() = default;

	bool isFinish() override
	{return false;};
	bool isExit() override
	{return false;};

	void run() override
	{};
};

#endif //PP_ACTION_H
