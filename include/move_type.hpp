//
// Created by root on 12/5/17.
//

#ifndef PP_MOVE_TYPE_HPP
#define PP_MOVE_TYPE_HPP

//#include "arch.hpp"
#include "boost/shared_ptr.hpp"

class ACleanMode;
class IMoveType
{
public:
	bool isFinish(ACleanMode* p_mode_,IAction* p_action,  int& action_i);
//	virtual IAction* getNextMovement()=0;
//	void registerMode(ACleanMode* sp_mode)
//	{
//		sp_mode_ = sp_mode;
//	}

//	static ACleanMode* sp_mode_;
protected:

};

class MoveTypeLinear:public IMoveType
{
public:
//	IAction* getNextMovement();
protected:
};

class MoveTypeFollowWall:public IMoveType
{
public:
	MoveTypeFollowWall(bool is_left)
	{
		is_left_ = is_left;
	}
//	IAction* getNextMovement();
protected:
	static bool is_left_;
};

class MoveTypeGoCharger:public IMoveType
{
public:
//	IAction* getNextMovement();
protected:
};

#endif //PP_MOVE_TYPE_HPP
