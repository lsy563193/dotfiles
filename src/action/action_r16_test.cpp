//
// Created by austin on 18-3-16.
//

#include <infrared_display.hpp>
#include <key_led.h>
#include <speaker.h>
#include <bumper.h>
#include <error.h>
#include <lidar.hpp>
#include <beeper.h>
#include <wifi/wifi.h>
#include <robot.hpp>
#include <wifi_led.hpp>
#include "ros/ros.h"
#include "action.hpp"

ActionR16Test::ActionR16Test()
{
	ROS_WARN("%s %d: Starting action R16 test." , __FUNCTION__, __LINE__);
}

void ActionR16Test::run()
{
	// Test item: RAM.
	infrared_display.displayNormalMsg(1, 0);
	if (!RAM_test())
	{
		ROS_ERROR("%s %d: RAM test failed!!", __FUNCTION__, __LINE__);
		error_loop(1, 0, RAM_ERROR);
	}
	ROS_WARN("%s %d: Test for RAM successed.", __FUNCTION__, __LINE__);

	// Test item: Flash.
	infrared_display.displayNormalMsg(2, 0);
	if (!Flash_test())
	{
		ROS_ERROR("%s %d: Flash test failed!!", __FUNCTION__, __LINE__);
		error_loop(2, 0, R16_FLASH_ERROR);
	}
	ROS_WARN("%s %d: Test for Flash succeeded.", __FUNCTION__, __LINE__);


	// Test item: Lidar.
	infrared_display.displayNormalMsg(3, 0);
	if (!lidar_test())
	{
		ROS_ERROR("%s %d: Lidar test failed!!", __FUNCTION__, __LINE__);
		error_loop(3, 0, LIDAR_ERROR);
	}
	ROS_WARN("%s %d: Test for lidar succeeded.", __FUNCTION__, __LINE__);

	// Test item: serial wifi test.
	infrared_display.displayNormalMsg(4, 0);
	if (!wifi_test())
	{
		ROS_ERROR("%s %d: Serial WIFI test failed!!", __FUNCTION__, __LINE__);
		error_loop(4, 0, SERIAL_WIFI_ERROR);
	}

	// Test item: Lidar bumper.
	infrared_display.displayNormalMsg(5, 0);
	if (bumper.lidarBumperInit(robot::instance()->getLidarBumperDev().c_str()) != 1 || !lidar_bumper_test())
	{

		ROS_ERROR("%s %d: Lidar bumper test failed!!", __FUNCTION__, __LINE__);
		error_loop(5, bumper_cnt_, LIDAR_BUMPER_ERROR);
	}
	ROS_WARN("%s %d: Test for lidar bumper succeeded.", __FUNCTION__, __LINE__);

	// Test finish.
	double alarm_time = ros::Time::now().toSec();
	speaker.play(VOICE_TEST_SUCCESS);
	infrared_display.displayNormalMsg(0, 9999);
	key_led.setMode(LED_STEADY, LED_GREEN);
	ROS_WARN("%s %d: Test finish.", __FUNCTION__, __LINE__);
	while (ros::ok())
	{
		if (ros::Time::now().toSec() - alarm_time > 5)
		{
			alarm_time = ros::Time::now().toSec();
			speaker.play(VOICE_TEST_SUCCESS);
			ROS_INFO("%s %d: Test finish.", __FUNCTION__, __LINE__);
		}
	}
}

bool ActionR16Test::RAM_test()
{
	ROS_WARN("%s %d: Start RAM test.", __FUNCTION__, __LINE__);
	bool test_ret = false;
	int RAM_test_size = 2; // In Mb.
	int RAM_test_block_cnt = 3; // Test RAM_test_block_cnt blocks of RAM sizing RAM_test_size Mb.

	pid_t status;
	// (Austin)Use the modified memtester.
	std::string cmd = "memtester " + std::to_string(RAM_test_size) + " 1 " + std::to_string(RAM_test_block_cnt);
	ROS_INFO("%s %d: Run command: %s", __FUNCTION__, __LINE__, cmd.c_str());
	status = system(cmd.c_str());

	if (-1 == status)
		ROS_ERROR("%s %d: system error!", __FUNCTION__, __LINE__);
	else
	{
		ROS_INFO("exit status value = [0x%x]", status);
		if (WIFEXITED(status))
		{
			if (0 == WEXITSTATUS(status))
				test_ret = true;
			else
				ROS_ERROR("%s %d: Program test for RAM failed, failed code:%d!!", __FUNCTION__, __LINE__, WEXITSTATUS(status));
		}
		else
			ROS_ERROR("%s %d: Test for RAM end for exception, failed code:%d!!", __FUNCTION__, __LINE__, WEXITSTATUS(status));
	}

	return test_ret;

	/*int pid;
	int status = 0;
	while ((pid = fork()) < 0)
	{
		ROS_ERROR("%s %d: fork() failed:%d.", __FUNCTION__, __LINE__, errno);
		usleep(50000);
	}

	if (pid == 0)
	{
		// Child process.
		ROS_INFO("%s %d: Child process up, pid:%d.", __FUNCTION__, __LINE__, getpid());

		int fail_code = execlp("memtester", "memtester", std::to_string(RAM_test_size).c_str(), "1",
			   std::to_string(RAM_test_block_cnt).c_str() ,NULL);
		ROS_ERROR("%s %d: Child process end with error: %02d, %s", __FUNCTION__, __LINE__, errno, strerror(errno));
		_exit(fail_code);
	} else
	{
		// Parent process.
		ROS_INFO("%s %d: Parent process up, child pid:%d.", __FUNCTION__, __LINE__, pid);
		while (ros::ok() && waitpid(pid, &status, WNOHANG) == 0)
		{
//			ROS_INFO("%s %d: Parent waiting for pid:%d", __FUNCTION__, __LINE__, pid);
//			if (errno != EINTR)
//			{
//				status = -1;
//				break;
//			}
			usleep(200000);
		}
		switch (status)
		{
			case 0:
				ROS_INFO("%s %d: Test for RAM successed.", __FUNCTION__, __LINE__);
				test_ret = true;
				break;
			default:
				ROS_ERROR("%s %d: Test for RAM failed, failed code:%d!!", __FUNCTION__, __LINE__, status);
				break;
		}
	}


	return test_ret;*/
}

bool ActionR16Test::Flash_test()
{

	ROS_WARN("%s %d: Start Flash test.", __FUNCTION__, __LINE__);
	std::string origin_file = "/origin_random.file";
	std::string new_file = "/random.file";
	std::string cmd;
	char origin_md5[32];
	char new_md5[32];

	// Check if origin file exists, if not, generate one.
	if (access(origin_file.c_str(), F_OK) == -1)
	{
		ROS_WARN("%s %d: Access file errno:%s.", __FUNCTION__, __LINE__, strerror(errno));
		size_t block_cnt = 1024 * 10; // 1 block means 1k byte.
		ROS_INFO("%s %d: Generate random file for %fMb.", __FUNCTION__, __LINE__, static_cast<float>(block_cnt) / 1024);
		cmd = "dd if=/dev/urandom of=" + origin_file + " count=" + std::to_string(block_cnt) + " bs=1024";
		system(cmd.c_str());
	}

	// Get the md5sum of origin file.
	cmd = "md5sum " + origin_file;
	FILE *f = popen(cmd.c_str(), "r");
	if (f)
	{
		fgets(origin_md5, 32, f);
		pclose(f);
	} else
	{
		ROS_ERROR("%s %d: popen file error:%s.", __FUNCTION__, __LINE__, strerror(errno));
		return false;
	}

	// Copy a new file from origin file.
	if (access(new_file.c_str(), F_OK) != -1)
	{
		ROS_WARN("%s %d: Delete existing %s.", __FUNCTION__, __LINE__, new_file.c_str());
		cmd = "rm " + new_file;
		system(cmd.c_str());
	}
	ROS_WARN("%s %d: Copy %s to %s.", __FUNCTION__, __LINE__, origin_file.c_str(), new_file.c_str());
	cmd = "cp " + origin_file + " " + new_file;
	system(cmd.c_str());

	// Get the md5sum of new file.
	cmd = "md5sum " + new_file;
	f = popen(cmd.c_str(), "r");
	if (f)
	{
		fgets(new_md5, 32, f);
		ROS_INFO("%s %d: Origin random file md5 : %s.", __FUNCTION__, __LINE__, origin_md5);
		ROS_INFO("%s %d: New random file md5    : %s.", __FUNCTION__, __LINE__, new_md5);
		pclose(f);
	} else
	{
		ROS_ERROR("%s %d: popen file error:%s.", __FUNCTION__, __LINE__, strerror(errno));
		return false;
	}

	return strcmp(origin_md5, new_md5) == 0;
}

bool ActionR16Test::lidar_test()
{
	ROS_WARN("%s %d: Start lidar test.", __FUNCTION__, __LINE__);
	ros::NodeHandle nh_;
	ros::Subscriber scan_sub_;
	scan_sub_ = nh_.subscribe("scanOriginal", 1, &Lidar::scantestCb, &lidar);
	lidar.init();
	while (ros::ok() && !lidar.motorCtrl(ON))
		usleep(500000);

	// Test logic(not finished).
	double start_time = ros::Time::now().toSec();
	bool receive_scan{false};
	while (ros::ok() && ros::Time::now().toSec() - start_time < 15)
	{
		usleep(200000);
		if (lidar.isScanOriginalReady())
		{
			ROS_INFO("%s %d: Receive scan.", __FUNCTION__, __LINE__);
			receive_scan = true;
			break;
		}
	}

	if (!receive_scan)
		ROS_ERROR("%s %d: Time out.", __FUNCTION__, __LINE__);

	lidar.motorCtrl(OFF);
	scan_sub_.shutdown();
	return receive_scan;
}

bool ActionR16Test::lidar_bumper_test()
{
	ROS_WARN("%s %d: Start lidar bumper test.", __FUNCTION__, __LINE__);
	// Heads up for starting lider bumper test.
	speaker.play(VOICE_WAKE_UP_UNOFFICIAL);

	int test_bumper_cnt = 5;
	bool last_bumper_status = false;
	double start_time = ros::Time::now().toSec();
	while (ros::ok() && bumper_cnt_ < test_bumper_cnt && ros::Time::now().toSec() - start_time < 10)
	{
		bumper.setLidarBumperStatus();
		if (!last_bumper_status && bumper.getLidarBumperStatus())
		{
			beeper.beepForCommand(VALID);
			bumper_cnt_++;
			infrared_display.displayNormalMsg(5, static_cast<uint16_t>(bumper_cnt_));
			ROS_INFO("%s %d: Hit lidar bumper for %d time.", __FUNCTION__, __LINE__, bumper_cnt_);
		}
		last_bumper_status = bumper.getLidarBumperStatus();
		usleep(20000);
	}

	return bumper_cnt_ == test_bumper_cnt;
}

void ActionR16Test::error_loop(uint8_t test_stage, uint16_t content, uint16_t error_code)
{
	infrared_display.displayErrorMsg(test_stage, content, error_code);
	key_led.setMode(LED_STEADY, LED_RED);

	double alarm_time = ros::Time::now().toSec();
	speaker.play(VOICE_TEST_FAIL);
	ROS_ERROR("%s %d: Test ERROR. test_stage_: %d. error_code: %d, content: %d", __FUNCTION__, __LINE__, test_stage, error_code, content);
	while (ros::ok())
	{
		if (ros::Time::now().toSec() - alarm_time > 5)
		{
			alarm_time = ros::Time::now().toSec();
			speaker.play(VOICE_TEST_FAIL);
			ROS_ERROR("%s %d: Test ERROR. test_stage_: %d. error_code: %d, content: %d", __FUNCTION__, __LINE__, test_stage, error_code, content);
		}
	}
}

bool ActionR16Test::wifi_test()
{
	wifi_led.enable();
	bool wifi_test_result = s_wifi.factoryTest();
	if (wifi_test_result)
		s_wifi.rebind();
	wifi_led.disable();
	return wifi_test_result;
}

