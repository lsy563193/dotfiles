//
// Created by lsy563193 on 12/4/17.
//

#ifndef PP_MODE_H_H
#define PP_MODE_H_H

#include "path_algorithm/path_algorithm.h"
#include "event_manager.h"
#include "boost/shared_ptr.hpp"
class IAction;
class Mode:public EventHandle {
public:
	virtual ~Mode() { };
	Mode *run();

	virtual bool isExit();

	virtual bool isFinish();

	virtual IAction *getNextActionOpenGyro();

	virtual IAction *getNextActionBackFromCharger();

	virtual IAction *getNextActionOpenLidar();

	virtual IAction *getNextActionAlign();

	virtual IAction *getNextActionOpenSlam();

	virtual IAction *getNextActionMoveForward();

	virtual IAction *getNextActionMoveTurn();

	virtual IAction *getNextActionMoveBack();

	virtual IAction *getNextActionMoveFollowWall();

	virtual IAction *getNextActionGoCharger();

protected:
	static boost::shared_ptr<IAction> sp_action_;
	Mode *p_next_clean_mode_;
};

class ACleanMode:public Mode{
public:
	bool isFinish();

protected:
	static boost::shared_ptr<State> sp_state_;
};

class CleanModeNav:public ACleanMode{
public:

	CleanModeNav();
//	bool isFinish();
	IAction* getNextActionOpenGyro();
	IAction* getNextActionOpenSlam();
};

#endif //PP_MODE_H_H
