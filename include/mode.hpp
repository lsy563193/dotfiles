//
// Created by lsy563193 on 12/4/17.
//

#ifndef PP_MODE_H_H
#define PP_MODE_H_H

#include "path_algorithm/path_algorithm.h"
#include "event_manager.h"
#include "boost/shared_ptr.hpp"
class Mode:public EventHandle {
public:
	virtual ~Mode() { };
	void run();

	virtual bool isExit();

	virtual bool isFinish();

	virtual IAction* getNextAction()=0;

protected:
	static boost::shared_ptr<IAction> sp_action_;
	int action_i_{ac_null};
	enum {
		ac_null,
		ac_open_gyro,
		ac_back_form_charger,
		ac_open_lidar,
		ac_align,
		ac_open_slam,
		ac_movement_forward,
		ac_movement_back,
		ac_movement_turn,
		ac_movement_follow_wall,
		ac_movement_go_charger,
	};
private:

};

class ACleanMode:public Mode{
public:
	virtual State* getNextState()=0;
	bool isFinish();

protected:
	static boost::shared_ptr<State> sp_state_;
	int state_i_{st_null};
		enum {
		st_null,
		st_state_clean,
		st_go_home_point,
		st_go_charger,
		st_trapped,
		st_tmp_spot,
		st_self_check,
		st_exploration,
	};
private:
};

class CleanModeNav:public ACleanMode{
public:
	CleanModeNav();
	State* getNextState();
	IAction* getNextAction();


};

#endif //PP_MODE_H_H
