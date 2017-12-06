//
// Created by root on 12/5/17.
//

#ifndef PP_MOVE_TYPE_HPP
#define PP_MOVE_TYPE_HPP

//#include "arch.hpp"
#include "boost/shared_ptr.hpp"

class IMoveType
{
public:
	bool isFinish();
	virtual IAction* getNextAction()=0;
//	void registerMode(ACleanMode* sp_mode)
//	{
//		sp_mode_ = sp_mode;
//	}

//	static ACleanMode* sp_mode_;
protected:
	static boost::shared_ptr<IAction> sp_movement_;
/*private:
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
	};*/
};

class MoveTypeLinear:public IMoveType
{
public:
	IAction* getNextAction();
protected:
};

class MoveTypeFollowWall:public IMoveType
{
public:
	IAction* getNextAction();
protected:
};

class MoveTypeGoCharger:public IMoveType
{
public:
	IAction* getNextAction();
protected:
};

#endif //PP_MOVE_TYPE_HPP
