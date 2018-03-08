//
// Created by austin on 18-3-7.
//

#include <serial.h>
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
	return false;
}

void MoveTypeDeskTest::run()
{
	switch (test_stage_)
	{
		case 0:
		{
			serial.setSendData(CTL_MAIN_BOARD_MODE, DESK_TEST_MODE_1);
			test_stage_++;
			break;
		}
		case 1:
		{
			break;
		}


		default:
			break;
	}
}
