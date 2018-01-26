//
// Created by austin on 18-1-26.
//

#include "r16_board_test.hpp"

#if R16_BOARD_TEST
void r16_board_test()
{

	// Test item 1: Speaker.
	speaker.test();
	// If you can not hear the voice, then speaker port has error, but there is no way to test it by software.

}

void error_loop()
{
	while (ros::ok())
	{
		ROS_INFO("%s %d: Test finish.", __FUNCTION__, __LINE__);
		sleep(1);
	}
}
#endif