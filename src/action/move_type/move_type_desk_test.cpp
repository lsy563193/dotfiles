//
// Created by austin on 18-3-7.
//

#include "move_type.hpp"

MoveTypeDeskTest::MoveTypeDeskTest()
{
	ROS_INFO("%s,%d: Enter move type desk test.", __FUNCTION__, __LINE__);
}

MoveTypeDeskTest::~MoveTypeDeskTest()
{
	ROS_INFO("%s,%d: Exit move type desk test.", __FUNCTION__, __LINE__);
}

bool MoveTypeDeskTest::isFinish()
{
	return true;
}

void MoveTypeDeskTest::run()
{
}
