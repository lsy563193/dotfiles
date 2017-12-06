//
// Created by root on 12/5/17.
//

#ifndef PP_MOVE_TYPE_HPP
#define PP_MOVE_TYPE_HPP

//#include "arch.hpp"
#include "boost/shared_ptr.hpp"

class Mode;
class IMoveType
{
public:
	virtual IMoveType* getNextMoveType()=0;
	bool isFinish();

//	void registerMode(ACleanMode* sp_mode)
//	{
//		sp_mode_ = sp_mode;
//	}

//	static ACleanMode* sp_mode_;
protected:
	static boost::shared_ptr<IAction> sp_movement_;
	IMoveType* p_next_mt_;
public:
	enum {
		mt_null,
		mt_linear,
		mt_follow_wall,
		mt_go_charger,
	};
};

class MoveTypeLinear:public IMoveType
{
public:
	IMoveType* getNextMoveType();
protected:
};

class MoveTypeFollowWall:public IMoveType
{
public:
	IMoveType* getNextMoveType();
protected:
};

class MoveTypeGoCharger:public IMoveType
{
public:
	IMoveType* getNextMoveType();
protected:
};

#endif //PP_MOVE_TYPE_HPP
