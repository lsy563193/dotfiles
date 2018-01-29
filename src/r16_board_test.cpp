//
// Created by austin on 18-1-26.
//

#include "r16_board_test.hpp"

#if R16_BOARD_TEST
void r16_board_test(std::string serial_port, int baud_rate)
{

	// Test item 1: Speaker.
	speaker.test();
	// If you can not hear the voice, then speaker port has error, but there is no way to test it by software.

	// Test item 2: Communication to main board.
	if (!serial.init(serial_port.c_str(), baud_rate))
	{
		ROS_ERROR("%s %d: Serial init failed!!", __FUNCTION__, __LINE__);
		error_loop();
	}

	auto serial_receive_routine = new boost::thread(boost::bind(&Serial::receive_routine_cb, &serial));
	auto serial_send_routine = new boost::thread(boost::bind(&Serial::send_routine_cb, &serial));

	if (!serial.test())
	{
		ROS_ERROR("%s %d: Serial test failed!!", __FUNCTION__, __LINE__);
		error_loop();
	}

}

void error_loop()
{
	while (ros::ok())
	{
		speaker.play(VOICE_TEST_FAIL);
		ROS_ERROR("%s %d: Test ERROR.", __FUNCTION__, __LINE__);
		sleep(5);
	}
}
#endif