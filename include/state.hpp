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

	bool isFinish();
//	bool setNextState();
//	State* getNextState();
	void setMode(ACleanMode* cm)
	{sp_cm_ = cm;}
	ACleanMode* getMode()
	{return sp_cm_;}

	virtual void update()=0;

protected:
	static ACleanMode* sp_cm_;
};

class StateInit: public State {
public:
	StateInit() = default;
	void update() override;
};

class StateClean: public State {
public:
	StateClean() = default;
	void update() override;
//	IMoveType* setNextAction_();

public:
};

class StateGoHomePoint: public State {
public:
	StateGoHomePoint() = default;
	void update() override;
//	IMoveType* setNextAction_();
};

class StateGoCharger: public State {
public:
	StateGoCharger() = default;
	void update() override;
//	IMoveType* setNextAction_();

};


class StateCharge: public State {
public:
	StateCharge() = default;
	void update() override;
//	IMoveType* setNextAction_();

};
class StateTrapped: public State {
public:
	StateTrapped()= default;
	void update() override;
//	IMoveType* setNextAction_();
};

class StateTmpSpot: public State {
public:
	StateTmpSpot()= default;
	void update() override;
//	IMoveType* setNextAction_();
};

class StateSelfCheck: public State {
public:
	StateSelfCheck()= default;
	void update() override;
//	IMoveType* setNextAction_();
};

class StateExploration: public State {
public:
	StateExploration()= default;
	void update() override;
//	IMoveType* setNextAction_();
};
class StateResumeLowBatteryCharge: public State {
public:
	StateResumeLowBatteryCharge()= default;
	void update() override;
//	IMoveType* setNextAction_();
};

class StatePause: public State {
public:
	StatePause()= default;
	void update() override;
//	IMoveType* setNextAction_();
};
//Movement *StateClean::generateMovement(PathType path, const Cell_t &curr) {
//	return nullptr;
//}

#endif //PP_STATE_HPP
