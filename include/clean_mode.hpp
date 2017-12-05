//
// Created by root on 12/5/17.
//

#ifndef PP_CLEAN_MODE_HPP
#define PP_CLEAN_MODE_HPP

#include <mode/mode.hpp>
#include <state.hpp>

class ACleanMode:public Mode{
public:
	virtual bool updateAction()=0;

protected:
	boost::shared_ptr<PathAlgorithm> sp_path_algorithm_;
	static boost::shared_ptr<State> sp_state_;
};

class CleanModeNav:public ACleanMode{
public:

	CleanModeNav();
	bool updateAction();
	IAction* getNextActionOpenGyro();
	IAction* getNextActionOpenSlam();
};

#endif //PP_CLEAN_MODE_HPP
