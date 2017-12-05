//
// Created by lsy563193 on 12/4/17.
//

#ifndef PP_STATE_HPP
#define PP_STATE_HPP

#include <boost/shared_ptr.hpp>
#include "move_type.hpp"

class State {
public:
	virtual bool isFinish() = 0;
	virtual State* setNextState();

protected:
	static boost::shared_ptr<IMoveType> p_mt_;
//	enum{
//		st_null,
//		st_clean,
//		st_go_home,
//		st_go_charger,
//		st_trapped,
//		st_tmp_spot,
//		st_self_check,
//	};

//	static int index_;
};

class StateClean: public State {
public:
	StateClean();
	bool isFinish();
	State* setNextState();
public:
};

class StateGoHomePoint: public State {
public:
	StateGoHomePoint();
	bool isFinish();
	State* setNextState();

protected:
	int gh_state_{};

private:
	enum {gh_ing=0,gh_succuss,gh_faile};
};

class StateGoCharger: public State {
public:
	StateGoCharger();
	bool isFinish();
	State* setNextState();

};

class StateTrapped: public State {
public:
	StateTrapped();
	bool isFinish();
	State* setNextState();
};

class StateTmpSpot: public State {
public:
	StateTmpSpot();
	bool isFinish();
	State* setNextState();
};

class StateSelfCheck: public State {
public:
	StateSelfCheck();
	bool isFinish();
	State* setNextState();
};

class StateExploration: public State {
public:
	StateExploration();
	bool isFinish();
	State* setNextState();

};

//Movement *StateClean::generateMovement(PathType path, const Cell_t &curr) {
//	return nullptr;
//}

#endif //PP_STATE_HPP
