//
// Created by lsy563193 on 12/4/17.
//

#ifndef PP_MODE_H_H
#define PP_MODE_H_H

#include <state.hpp>
//#include "action.hpp"
#include "event_manager.h"
#include "path_algorithm/path_algorithm.h"

class IAction;
class Mode:public EventHandle{
public:
	Mode* run();
	virtual bool isExit();
	virtual bool updateAction();
	virtual IAction* getNextActionOpenGyro()=0;
	virtual IAction* getNextActionBackFromCharger();
	virtual IAction* getNextActionOpenLidar();
	virtual IAction* getNextActionAlign();
	virtual IAction* getNextActionOpenSlam();

protected:
	static boost::shared_ptr<IAction> sp_action_;
	Mode* p_next_clean_mode_;
};

class ACleanMode:public Mode{
public:
	virtual bool updateAction()=0;

protected:
	boost::shared_ptr<PathAlgorithm> sp_path_algorithm_;
	static boost::shared_ptr<StateCleanNavigation> sp_state_;
};

class CleanModeNav:public ACleanMode{
public:

	CleanModeNav();
	bool updateAction();
	IAction* getNextActionOpenGyro();
	IAction* getNextActionOpenSlam();
};

#endif //PP_MODE_H_H
