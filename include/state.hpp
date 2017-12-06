//
// Created by lsy563193 on 12/4/17.
//

#ifndef PP_STATE_HPP
#define PP_STATE_HPP

//#include <boost/shared_ptr.hpp>

class State {
public:
	virtual State* getNextState()=0;

	bool isFinish();

//	void registerMode(ACleanMode* sp_mode){
//		sp_mode_ = sp_mode;
//	};

//	static ACleanMode* sp_mode_;

protected:
	static boost::shared_ptr<IMoveType> sp_move_type_;
	static int s_index_;
public:
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
};

class StateClean: public State {
public:
	StateClean();
	State* setNextState();

public:
};

class StateGoHomePoint: public State {
public:
	StateGoHomePoint();
	State* getNextState();

protected:
	int gh_state_{};

private:
	enum {gh_ing=0,gh_succuss,gh_faile};
};

class StateGoCharger: public State {
public:
	StateGoCharger();
	State* getNextState();

};

class StateTrapped: public State {
public:
	StateTrapped();
	State* getNextState();
};

class StateTmpSpot: public State {
public:
	StateTmpSpot();
	State* getNextState();
};

class StateSelfCheck: public State {
public:
	StateSelfCheck();
	State* getNextState();
};

class StateExploration: public State {
public:
	StateExploration();
	State* getNextState();
};

//Movement *StateClean::generateMovement(PathType path, const Cell_t &curr) {
//	return nullptr;
//}

#endif //PP_STATE_HPP
