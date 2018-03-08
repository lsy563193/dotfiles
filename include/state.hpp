//
// Created by lsy563193 on 12/4/17.
//

#ifndef PP_STATE_HPP
#define PP_STATE_HPP

#include "move_type.hpp"
//#include <boost/shared_ptr.hpp>
class ACleanMode;
class State {
public:
//	virtual IMoveType* setNextAction_()=0;

	bool isSwitchByEvent();
	bool updateAction();
	void switchState();
//	bool setNextState();
//	State* updateState();
	void setMode(ACleanMode* cm)
	{sp_cm_ = cm;}
	ACleanMode* getMode()
	{return sp_cm_;}

	virtual void init()=0;

protected:
	static ACleanMode* sp_cm_;
};

class StateInit: public State {
public:
	StateInit() = default;
	void init() override;
};

class StateClean: public State {
public:
	StateClean() = default;
	void init() override;
//	IMoveType* setNextAction_();

public:
};

class StateGoHomePoint: public State {
public:
	StateGoHomePoint() = default;
	void init() override;
//	IMoveType* setNextAction_();
};

class StateGoCharger: public State {
public:
	StateGoCharger() = default;
	void init() override;
//	IMoveType* setNextAction_();

};


class StateCharge: public State {
public:
	StateCharge() = default;
	void init() override;
//	IMoveType* setNextAction_();

};
class StateFolllowWall: public State {
public:
	StateFolllowWall()= default;
	void init() override;
//	IMoveType* setNextAction_();
};

class StateSpot: public State {
public:
	StateSpot()= default;
	void init() override;
//	IMoveType* setNextAction_();
};

class StateExceptionResume: public State {
public:
	StateExceptionResume()= default;
	void init() override;
//	IMoveType* setNextAction_();
};

class StateExploration: public State {
public:
	StateExploration()= default;
	void init() override;
//	IMoveType* setNextAction_();
};
class StateResumeLowBatteryCharge: public State {
public:
	StateResumeLowBatteryCharge()= default;
	void init() override;
//	IMoveType* setNextAction_();
};

class StatePause: public State {
public:
	StatePause()= default;
	void init() override;
//	IMoveType* setNextAction_();
};

class StateDeskTest: public State {
public:
	StateDeskTest()= default;
	void init() override;
};
#endif //PP_STATE_HPP
