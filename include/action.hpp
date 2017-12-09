//
// Created by austin on 17-12-3.
//

#ifndef PP_ACTION_H
#define PP_ACTION_H

#include <cstdint>
#include <time.h>

class IAction{
public:

	virtual ~IAction(){};

	virtual bool isFinish()=0;

	virtual void run()=0;

//	bool isTimeUp()
//	{
//		if(g_wf_diff_timer == 0 )
//			return false;
//		return ((uint32_t)difftime(time(NULL), g_wf_start_timer)) > g_wf_diff_timer;
//	}
protected:
//	uint32_t g_wf_start_timer{};
//	uint32_t g_wf_diff_timer{};
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

#endif //PP_ACTION_H