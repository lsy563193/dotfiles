//
// Created by austin on 17-12-3.
//

#ifndef PP_ACTION_H
#define PP_ACTION_H

class IAction{
public:
	virtual bool isFinish()=0;

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
	bool isFinish();
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


#endif //PP_ACTION_H
