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

	bool isTimeUp();

	virtual void run()=0;

protected:
	double start_timer_{};
	double interval_{};
};

class ActionOpenGyro :public IAction
{
public:
	ActionOpenGyro();
	bool isFinish();
	void run();
//	virtual void curr()=0;
//friend Mode;


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
	//todo: timeout handling??
	ActionOpenLidar();
	bool isFinish();
	void run();
};

class ActionAlign :public IAction
{
public:
	ActionAlign();
	bool isFinish();
	void run();
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
