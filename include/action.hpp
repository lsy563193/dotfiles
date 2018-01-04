//
// Created by austin on 17-12-3.
//

#ifndef PP_ACTION_H
#define PP_ACTION_H

#include <cstdint>
#include <time.h>
#include "odom.h"

class IAction{
public:
	IAction();
	virtual ~IAction() = default;

	virtual bool isFinish()=0;

	virtual bool isExit();

	virtual bool isTimeUp();

	virtual void run()=0;

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
	double align_angle = 0.0;
	double distance;
	bool isLeft = true;
};

class ActionOpenSlam :public IAction
{
public:
	ActionOpenSlam();
	bool isFinish();
	void run();
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
	~ActionIdle();
	bool isFinish();
	void run();
private:

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
#endif //PP_ACTION_H
