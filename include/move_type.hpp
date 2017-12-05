//
// Created by root on 12/5/17.
//

#ifndef PP_MOVE_TYPE_HPP
#define PP_MOVE_TYPE_HPP

#include "movement.hpp"

class IMoveType
{
public:

protected:
	static boost::shared_ptr<IMovement> sp_movement_;
};

class MoveTypeLinear
{
public:
	bool isFinish();
//	IMovement* setNetMovement();
protected:
};

class MoveTypeFollowWall
{
public:
	bool isFinish();
//	IMovement* setNetMovement();
protected:
};

class MoveTypeGoCharge
{
public:
	bool isFinish();
//	IMovement* setNetMovement();
protected:
};

#endif //PP_MOVE_TYPE_HPP
