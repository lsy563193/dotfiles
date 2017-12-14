//
// Created by root on 12/5/17.
//

#ifndef PP_MOVE_TYPE_HPP
#define PP_MOVE_TYPE_HPP

//#include "arch.hpp"
#include "boost/shared_ptr.hpp"

class ACleanMode;
class IMoveType:public IAction
{
public:
	IMoveType();
//	~IMoveType() = default;

	virtual bool isFinish();
	void run();
//	bool isFinish(int& action_i);
//	virtual IAction* setNextAction()=0;
//	void registerMode(ACleanMode* sp_mode)
//	{
//		sp_mode_ = sp_mode;
//	}

	static boost::shared_ptr<IAction> sp_movement_;
	static boost::shared_ptr<ACleanMode> sp_cm_;
	static int movement_i_;
	void resetTriggeredValue();
protected:
//	Path_t passed_path_;
//	Path_t plan_path_;
	int16_t turn_target_angle_{};
	enum{//movement
		mm_null,
		mm_back,
		mm_turn,
		mm_forward,
	};
};

class ActionLinear:public IMoveType
{
public:
	ActionLinear();
	bool isFinish();
//	IAction* setNextAction();
protected:
};

class ActionFollowWall:public IMoveType
{
public:
	ActionFollowWall(bool is_left);

//	IAction* setNextAction();
protected:
	static bool is_left_;
};

class ActionGoCharger:public IMoveType
{
public:
	ActionGoCharger();
protected:
};

#endif //PP_MOVE_TYPE_HPP
