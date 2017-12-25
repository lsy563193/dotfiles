//
// Created by lsy563193 on 12/4/17.
//

#ifndef PP_STATE_HPP
#define PP_STATE_HPP

//#include <boost/shared_ptr.hpp>
class ACleanMode;
class State {
public:
//	virtual IMoveType* setNextAction_()=0;

//	bool isFinish(ACleanMode* p_mode, IMoveType* p_move_type,IAction* p_action, int& action_i);

protected:

};

class StateClean: public State {
public:
	StateClean();
//	IMoveType* setNextAction_();

public:
};

class StateGoHomePoint: public State {
public:
	StateGoHomePoint();
//	IMoveType* setNextAction_();

protected:
	int gh_state_{};

private:
	enum {gh_ing=0,gh_succuss,gh_faile};
};

class StateGoCharger: public State {
public:
	StateGoCharger();
//	IMoveType* setNextAction_();

};

class StateTrapped: public State {
public:
	StateTrapped();
//	IMoveType* setNextAction_();
};

class StateTmpSpot: public State {
public:
	StateTmpSpot();
//	IMoveType* setNextAction_();
};

class StateSelfCheck: public State {
public:
	StateSelfCheck();
//	IMoveType* setNextAction_();
};

class StateExploration: public State {
public:
	StateExploration();
//	IMoveType* setNextAction_();
};

//Movement *StateClean::generateMovement(PathType path, const Cell_t &curr) {
//	return nullptr;
//}

#endif //PP_STATE_HPP
