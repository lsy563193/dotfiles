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

	virtual ~IAction(){};

	virtual bool isFinish()=0;

	virtual bool isExit(){return false;};

	virtual bool isTimeUp(){};

	virtual void run()=0;

protected:
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


class ActionIdle :public IAction {
public:
	ActionIdle();
	~ActionIdle();
	bool isFinish();
	void run();
private:

};

class ActionCharge :public IAction
{
public:
	explicit ActionCharge(bool play_start_wav);
	~ActionCharge() override;
	bool isFinish() override;
	void run() override;

private:
	uint8_t disconnect_charger_count_;
	time_t show_battery_info_time_stamp_;
};

class ActionPause :public IAction
{
public:
	ActionPause();
	~ActionPause();
	bool isFinish() override;
	bool isExit() override;
	bool isTimeUp() override ;
	void run() override;

private:
	double pause_start_time_;
	Pose pause_pose_;
};
#endif //PP_ACTION_H
