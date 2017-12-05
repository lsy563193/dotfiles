//
// Created by austin on 17-12-3.
//

#ifndef PP_ACTION_H
#define PP_ACTION_H

class Mode;
class IAction{
public:
//	IAction IAction(Mode* p_mode);
	virtual bool isFinish()=0;
	static IAction *getNextAction();
	static void setNext(int index){
		s_index_ = index;
	};
	virtual void run()=0;
//	static void setMode(Mode* p_mode);

protected:
	static Mode* sp_mode_;
	static int s_index_;
public:
	enum {ac_null,ac_open_gyro,ac_back_form_charger,ac_open_lidar,ac_align,ac_open_slam};
};

class ActionOpenGyro :public IAction
{
public:
	ActionOpenGyro(Mode* p_mode);
	bool isFinish();
	void run();
//	virtual void setNext()=0;
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



#endif //PP_ACTION_H
