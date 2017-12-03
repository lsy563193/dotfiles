//
// Created by austin on 17-12-3.
//

#ifndef PP_ACTION_H
#define PP_ACTION_H

class Action{
public:
	virtual void action()=0;

	virtual bool isFinishAndUpdatePlan()=0;
	virtual bool isFinishAndUpdateAction()=0;

	virtual Action* getNextAction()=0;
protected:

};

#endif //PP_ACTION_H
