//
// Created by root on 11/29/17.
//

#ifndef PP_EVENT_ACTION_H
#define PP_EVENT_ACTION_H

#include "event_manager.h"
class EventAction:public EventHandle{
public:
	virtual bool isExit(){};
	virtual bool isStop(){};
	virtual void doSomething(){};
	virtual bool setNext(){};
	virtual void run();
};
class EventOpenGyro :public EventAction
{
public:
	EventOpenGyro();
	bool isExit(){};
	bool isStop();
	bool setNext();
	void doSomething();
};

class EventBackFromCharger :public EventAction
{
public:
//	EventBackFromCharger();
	bool isExit(){};
	bool isStop();
	bool setNext();
	void doSomething();
};
#endif //PP_EVENT_ACTION_H
