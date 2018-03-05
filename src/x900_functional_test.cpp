//
// Created by austin on 18-1-26.
//

#include "x900_functional_test.hpp"

#if X900_FUNCTIONAL_TEST
#include <sys/mount.h>
#include <sys/sysinfo.h>
#include <sys/vfs.h>
#include <random>
#include <wait.h>

void x900_functional_test(std::string serial_port, int baud_rate, std::string lidar_bumper_dev)
{
	uint16_t main_board_test_result=0;
	ROS_INFO("%s %d: Serial_port: %s, baudrate: %d, lidar_bumper_dev: %s.",
			 __FUNCTION__, __LINE__, serial_port.c_str(), baud_rate, lidar_bumper_dev.c_str());
	// Test item: Speaker.
	speaker.test();
	// If you can not hear the voice, then speaker port has error, but there is no way to test it by software.

	// Test item: Serial port.
	if (!serial.init(serial_port, baud_rate) || !serial_port_test())
	{
		ROS_ERROR("%s %d: Serial port test failed!!", __FUNCTION__, __LINE__);
		error_loop(SERIAL_ERROR);
	}
	ROS_INFO("Test serial port succeeded!!");
	auto serial_send_routine = new boost::thread(boost::bind(&Serial::send_routine_cb, &serial));

	/*// Test item: RAM.
	if (!RAM_test())
	{
		ROS_ERROR("%s %d: RAM test failed!!", __FUNCTION__, __LINE__);
		error_loop(RAM_ERROR);
	}
	ROS_INFO("%s %d: Test for RAM successed.", __FUNCTION__, __LINE__);

	// Test item: Flash.
	if (!Flash_test())
	{
		ROS_ERROR("%s %d: Flash test failed!!", __FUNCTION__, __LINE__);
		error_loop(FLASH_ERROR);
	}
	ROS_INFO("%s %d: Test for Flash succeeded.", __FUNCTION__, __LINE__);


	// Test item: Lidar.
	if (!lidar_test())
	{
		ROS_ERROR("%s %d: Lidar test failed!!", __FUNCTION__, __LINE__);
		error_loop(LIDAR_ERROR);
	}
	ROS_INFO("%s %d: Test for lidar succeeded.", __FUNCTION__, __LINE__);

	// Test item: Lidar bumper.
	if (bumper.lidarBumperInit(lidar_bumper_dev.c_str()) != 1 || !lidar_bumper_test())
	{

		ROS_ERROR("%s %d: Lidar bumper test failed!!", __FUNCTION__, __LINE__);
		error_loop(LIDAR_BUMPER_ERROR);
	}
	ROS_INFO("%s %d: Test for lidar bumper succeeded.", __FUNCTION__, __LINE__);
*/
	auto serial_receive_routine = new boost::thread(boost::bind(&Serial::receive_routine_cb, &serial));

	// Wait for the end of voice playing
	speaker.play(VOICE_NULL, false);
	usleep(2000);
	// Test hardware from main board.
	main_board_test_result = main_board_test();
	if (main_board_test_result)
	{
		ROS_ERROR("%s %d: Main board test failed!!", __FUNCTION__, __LINE__);
		error_loop(main_board_test_result);
	}
	ROS_INFO("Test main board success!!");

	// Test finish.
	double alarm_time = ros::Time::now().toSec();
	speaker.play(VOICE_TEST_SUCCESS);
	ROS_INFO("%s %d: Test finish.", __FUNCTION__, __LINE__);
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

void error_loop(uint16_t error_code)
{
	serial.setSendData(CTL_MAIN_BOARD_MODE, ALARM_ERROR_MODE);
	serial.setSendData(CTL_ERROR_CODE_HIGH, static_cast<uint8_t>(error_code >> 8));
	serial.setSendData(CTL_ERROR_CODE_LOW, static_cast<uint8_t>(error_code));

	double alarm_time = ros::Time::now().toSec();
	speaker.play(VOICE_TEST_FAIL);
	ROS_ERROR("%s %d: Test ERROR.", __FUNCTION__, __LINE__);
	while (ros::ok())
	{
		if (ros::Time::now().toSec() - alarm_time > 5)
		{
			alarm_time = ros::Time::now().toSec();
			speaker.play(VOICE_TEST_FAIL);
			ROS_ERROR("%s %d: Test ERROR. error:code: %d", __FUNCTION__, __LINE__, error_code);
		}
	}
}

bool RAM_test()
{
	ROS_INFO("%s %d: Start RAM test.", __FUNCTION__, __LINE__);
	bool test_ret = false;
	int RAM_test_size = 2; // In Mb.
	int RAM_test_block_cnt = 3; // Test 3 blocks of RAM and size of each block is RAM_test_size Mb.

	pid_t status;
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

		// (Austin)Use the modified memtester.
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

bool Flash_test()
{

	ROS_INFO("%s %d: Start Flash test.", __FUNCTION__, __LINE__);
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

bool serial_port_test()
{
	ROS_INFO("%s %d: Start serial test.", __FUNCTION__, __LINE__);
	bool test_ret = true;
	std::random_device rd;
	std::mt19937 random_number_engine(rd());
	std::uniform_int_distribution<uint8_t> dist_char;
	std::string send_string_sum{};
	std::string receive_string_sum{};
	uint8_t receive_data[REC_LEN];
	int test_frame_cnt = 50;

	serial.resetSendStream();
	for (uint8_t test_cnt = 0; test_cnt < test_frame_cnt; test_cnt++)
	{
		// Write random numbers to send stream.
		for (uint8_t i = CTL_WHEEL_LEFT_HIGH; i < CTL_CRC; i++)
		{
			if (i == CTL_MAIN_BOARD_MODE)
				serial.setSendData(i, SERIAL_TEST_MODE);
			else if (i == CTL_BEEPER)
				serial.setSendData(i, static_cast<uint8_t>(test_cnt + 1));
			else if (i == CTL_KEY_VALIDATION)
				// Avoid 0x40/0x41/0x42/0x23.
				serial.setSendData(i, SERIAL_TEST_MODE);
			else
			{
				uint8_t random_byte = dist_char(random_number_engine);
				serial.setSendData(i, random_byte);
			}
			send_string_sum += std::to_string(serial.getSendData(i));
		}
		serial.sendData();
//		robot::instance()->debugSendStream(serial.send_stream);

		int read_ret = serial.read(receive_data, REC_LEN);

		if (read_ret != REC_LEN)
		{
			ROS_ERROR("%s %d: Error during read:%d.", __FUNCTION__, __LINE__, read_ret);
			test_ret = false;
			break;
		}

		for (uint8_t i = CTL_WHEEL_LEFT_HIGH; i < CTL_CRC; i++)
			receive_string_sum += std::to_string(receive_data[i]);
//		robot::instance()->debugReceivedStream(receive_data);
	}

	if (!test_ret)
		return test_ret;

	return send_string_sum.compare(receive_string_sum) == 0;

	// Test serial port /dev/ttyS2 and /dev/ttyS3 with direct connection.
/*	Serial serial_port_S2;
	Serial serial_port_S3;
	std::string ttyS2 = "/dev/ttyS2";
	std::string ttyS3 = "/dev/ttyS3";

	const int write_data_length = 50;
	if (!serial_port_S2.init(ttyS2, 115200))
	{
		ROS_ERROR("%s %d: %s init failed!!", __FUNCTION__, __LINE__, ttyS2);
		return false;
	}
	if (!serial_port_S3.init(ttyS3, 115200))
	{
		ROS_ERROR("%s %d: %s init failed!!", __FUNCTION__, __LINE__, ttyS3);
		return false;
	}

	std::random_device rd;
	std::mt19937 random_number_engine(rd());
	std::uniform_int_distribution<uint8_t> dist_char;


	uint8_t buf1[write_data_length];
	uint8_t buf2[write_data_length];
	//Write random numbers to buffer.
	for (int i = 0; i < write_data_length; i++)
		buf1[i] = dist_char(random_number_engine);

	ROS_INFO("Start testing for serial port.", __FUNCTION__, __LINE__);
	//Send message
	for (int i = 0; i < write_data_length; i++)
	{
		int counter = 0;
		uint8_t byte;
		usleep(200);
		serial_port_S3.write(buf1 + i, 1);
		ROS_INFO("%s %d: i:%d, S3 writing finish, buff: %d.", __FUNCTION__, __LINE__, i, buf1[i]);
		//tcflush(fd,TCIOFLUSH);
		while (ros::ok() && serial_port_S2.read(&byte, 1) < 0 && counter < 100)
		{
			ROS_INFO("%s %d: counter:%d.", __FUNCTION__, __LINE__, counter);
			usleep(200);
			counter++;
		};
		ROS_INFO("%s %d: i:%d, S2 reading finish, buff: %d.", __FUNCTION__, __LINE__, i, byte);
		counter = 0;
		serial_port_S2.write(&byte, 1);
		ROS_INFO("%s %d: i:%d, S2 writing finish.", __FUNCTION__, __LINE__, i);
		//tcflush(fd,TCIOFLUSH);
		while (ros::ok() && serial_port_S3.read(buf2 + i, 1) < 0 && counter < 100)
		{
			ROS_INFO("%s %d: counter:%d.", __FUNCTION__, __LINE__, counter);
			usleep(200);
			counter++;
		};
		ROS_INFO("%s %d: i:%d, S3 reading finish, buff: %d.", __FUNCTION__, __LINE__, i, buf2[i]);

	}
	//ROS_INFO(" transfered string:  %s \n\n\n.", buf1);
	//ROS_INFO(" received string:  %s \n\n\n.", buf2);
	auto char_buf1 = reinterpret_cast<char *>(buf1);
	auto char_buf2 = reinterpret_cast<char *>(buf2);

	return strncmp(char_buf1, char_buf2, write_data_length) == 0;*/
}

bool lidar_test()
{
	ROS_INFO("%s %d: Start lidar test.", __FUNCTION__, __LINE__);
	ros::NodeHandle nh_;
	ros::Subscriber scan_sub_;
	scan_sub_ = nh_.subscribe("scanOriginal", 1, &Lidar::scantestCb, &lidar);
	lidar.init();
	while (ros::ok() && !lidar.motorCtrl(ON))
		usleep(500000);
	while (ros::ok() && !lidar.isScanOriginalReady())
		usleep(200000);

	// Test logic(not finished).
	sleep(1);

	lidar.motorCtrl(OFF);
	scan_sub_.shutdown();
	return true;
}

bool lidar_bumper_test()
{
	ROS_INFO("%s %d: Start lidar bumper test.", __FUNCTION__, __LINE__);

	int test_bumper_cnt = 5;
	int bumper_cnt = 0;
	bool last_bumper_status = false;
	while (ros::ok() && bumper_cnt < test_bumper_cnt)
	{
		bumper.setLidarBumperStatus();
		if (!last_bumper_status && bumper.getLidarBumperStatus())
		{
			bumper_cnt++;
			ROS_INFO("%s %d: Hit lidar bumper for %d time.", __FUNCTION__, __LINE__, bumper_cnt);
		}
		last_bumper_status = bumper.getLidarBumperStatus();
		usleep(20000);
	}

	return true;
}

bool power_supply_test()
{
	return true;
	double voltage = 0;
	const int8_t check_cnt = 50;

	while (!robot::instance()->isSensorReady())
		usleep(1000);

	for (auto i = 0; i < check_cnt; i++)
	{
		voltage += static_cast<double>(battery.getVoltage());
		usleep(20000);
	}
	voltage = voltage / 100.0;
	voltage = voltage / check_cnt;
	ROS_INFO("%s %d: Average voltage: %f.", __FUNCTION__, __LINE__, voltage);
	const double voltage_limit = 5.0;

	return voltage > voltage_limit;
}

uint16_t main_board_test()
{
	bool is_fixture = false;
	uint16_t test_result=0;
	uint8_t test_stage = ELECTRICAL_AND_LED_TEST_MODE;
	uint16_t baseline[8];
	while(ros::ok()) {
		switch (test_stage) {
			case ELECTRICAL_AND_LED_TEST_MODE:/*---Main board electrical specification and LED test---*/
				test_result = electrical_specification_and_led_test(baseline, is_fixture, test_stage);
				break;
			case OBS_TEST_MODE:/*---OBS---*/
				test_result = obs_test(test_stage, is_fixture);
				break;
			case BUMPER_TEST_MODE:/*---bumper---*/
				test_result = bumper_test(test_stage);
				break;
			case CLIFF_TEST_MODE:/*---cliff---*/
				test_result = cliff_test(test_stage);
				break;
			case RCON_TEST_MODE:/*---rcon---*/
				test_result = rcon_test(test_stage);
				break;
			case WHEELS_TEST_MODE:/*---wheels---*/
				test_result = wheels_test(test_stage, baseline);
				break;
			case BRUSHES_TEST_MODE:/*---brushes---*/
				test_result = brushes_test(test_stage, baseline);
				break;
			case VACUUM_TEST_MODE:/*---vacuum---*/
				test_result = vacuum_test(test_stage, baseline);
				break;
//			case CHARGE_CURRENT_TEST_MODE:/*---charge current---*/
//				test_result = charge_current_test(test_stage);
				break;
		}
		if(test_result)
			return test_result;
	}
}

/*bool memory_test()
{
	struct sysinfo info{};
	sysinfo(&info);
	auto ram_size = static_cast<uint32_t>(((size_t) info.totalram * (size_t) info.mem_unit) / 1024);
	ROS_INFO("%s %d: RAM size: %dkbytes.", __FUNCTION__, __LINE__, ram_size);
	struct statfs fsb{};

	uint32_t flash_size = 0, flash_size_temp = 0;

	if (statfs("/dev/mmcblk0", &fsb) == 0)
	{
		flash_size_temp = static_cast<uint32_t>(((size_t) fsb.f_blocks * (size_t) fsb.f_bsize) / 1024);
		flash_size += flash_size_temp;
		ROS_INFO("%s %d: Got /dev/mmcblk0 size %ld.", __FUNCTION__, __LINE__, flash_size_temp);
	}
	if (statfs("/dev/mmcblk0p1", &fsb) == 0)
	{
		flash_size_temp = static_cast<uint32_t>(((size_t) fsb.f_blocks * (size_t) fsb.f_bsize) / 1024);
		flash_size += flash_size_temp;
		ROS_INFO("%s %d: Got /dev/mmcblk0p1 size %ld.", __FUNCTION__, __LINE__, flash_size_temp);
	}
	if (statfs("/dev/mmcblk0p2", &fsb) == 0)
	{
		flash_size_temp = static_cast<uint32_t>(((size_t) fsb.f_blocks * (size_t) fsb.f_bsize) / 1024);
		flash_size += flash_size_temp;
		ROS_INFO("%s %d: Got /dev/mmcblk0p2 size %ld.", __FUNCTION__, __LINE__, flash_size_temp);
	}
	if (statfs("/dev/mmcblk0p3", &fsb) == 0)
	{
		flash_size_temp = static_cast<uint32_t>(((size_t) fsb.f_blocks * (size_t) fsb.f_bsize) / 1024);
		flash_size += flash_size_temp;
		ROS_INFO("%s %d: Got /dev/mmcblk0p3 size %ld.", __FUNCTION__, __LINE__, flash_size_temp);
	}
	if (statfs("/dev/mmcblk0p4", &fsb) == 0)
	{
		flash_size_temp = static_cast<uint32_t>(((size_t) fsb.f_blocks * (size_t) fsb.f_bsize) / 1024);
		flash_size += flash_size_temp;
		ROS_INFO("%s %d: Got /dev/mmcblk0p4 size %ld.", __FUNCTION__, __LINE__, flash_size_temp);
	}
	if (statfs("/dev/mmcblk0p5", &fsb) == 0)
	{
		flash_size_temp = static_cast<uint32_t>(((size_t) fsb.f_blocks * (size_t) fsb.f_bsize) / 1024);
		flash_size += flash_size_temp;
		ROS_INFO("%s %d: Got /dev/mmcblk0p5 size %ld.", __FUNCTION__, __LINE__, flash_size_temp);
	}
	if (statfs("/dev/mmcblk0p6", &fsb) == 0)
	{
		flash_size_temp = static_cast<uint32_t>(((size_t) fsb.f_blocks * (size_t) fsb.f_bsize) / 1024);
		flash_size += flash_size_temp;
		ROS_INFO("%s %d: Got /dev/mmcblk0p6 size %ld.", __FUNCTION__, __LINE__, flash_size_temp);
	}
	if (statfs("/dev/mmcblk0p7", &fsb) == 0)
	{
		flash_size_temp = static_cast<uint32_t>(((size_t) fsb.f_blocks * (size_t) fsb.f_bsize) / 1024);
		flash_size += flash_size_temp;
		ROS_INFO("%s %d: Got /dev/mmcblk0p7 size %ld.", __FUNCTION__, __LINE__, flash_size_temp);
	}
	if (statfs("/dev/mmcblk0p8", &fsb) == 0)
	{
		flash_size_temp = static_cast<uint32_t>(((size_t) fsb.f_blocks * (size_t) fsb.f_bsize) / 1024);
		flash_size += flash_size_temp;
		ROS_INFO("%s %d: Got /dev/mmcblk0p8 size %ld.", __FUNCTION__, __LINE__, flash_size_temp);
	}
	if (statfs("/dev/mmcblk0p9", &fsb) == 0)
	{
		flash_size_temp = static_cast<uint32_t>(((size_t) fsb.f_blocks * (size_t) fsb.f_bsize) / 1024);
		flash_size += flash_size_temp;
		ROS_INFO("%s %d: Got /dev/mmcblk0p9 size %ld.", __FUNCTION__, __LINE__, flash_size_temp);
	}
	if (statfs("/dev/mmcblk0p10", &fsb) == 0)
	{
		flash_size_temp = static_cast<uint32_t>(((size_t) fsb.f_blocks * (size_t) fsb.f_bsize) / 1024);
		flash_size += flash_size_temp;
		ROS_INFO("%s %d: Got /dev/mmcblk0p10 size %ld.", __FUNCTION__, __LINE__, flash_size_temp);
	}
	if (statfs("/dev/mmcblk0boot0", &fsb) == 0)
	{
		flash_size_temp = static_cast<uint32_t>(((size_t) fsb.f_blocks * (size_t) fsb.f_bsize) / 1024);
		flash_size += flash_size_temp;
		ROS_INFO("%s %d: Got /dev/mmcblk0boot0 size %ld.", __FUNCTION__, __LINE__, flash_size_temp);
	}
	if (statfs("/dev/mmcblk0boot1", &fsb) == 0)
	{
		flash_size_temp = static_cast<uint32_t>(((size_t) fsb.f_blocks * (size_t) fsb.f_bsize) / 1024);
		flash_size += flash_size_temp;
		ROS_INFO("%s %d: Got /dev/mmcblk0boot1 size %ld.", __FUNCTION__, __LINE__, flash_size_temp);
	}
	if (statfs("/mnt/UDISK", &fsb) == 0)
	{
		flash_size_temp = static_cast<uint32_t>(((size_t) fsb.f_blocks * (size_t) fsb.f_bsize) / 1024);
		flash_size += flash_size_temp;
		ROS_INFO("%s %d: Got /mnt/UDISK size %ld.", __FUNCTION__, __LINE__, flash_size_temp);
	}

	ROS_INFO("%s %d: Total flash size: %ldkBytes.", __FUNCTION__, __LINE__, flash_size);

	//Transform sizes into GB and send to main board.

	wheel.pidSetLeftSpeed(static_cast<float>(flash_size));
	wheel.pidSetRightSpeed(static_cast<float>(ram_size));

	ROS_INFO("Send memory data....");

	return true;
}*/

/*bool usb_test(std::string dev_path, std::string fs_type, int write_length)
{
	return true;
	std::random_device rd;
	std::mt19937 random_number_engine(rd());
	std::uniform_int_distribution<char> dist_char;
	char buf1[write_length];
	//Write random numbers to buffer.
	for (int i = 0; i < write_length; i++)
		buf1[i] = dist_char(random_number_engine);

	system("/bin/mkdir -p /mnt/tmp");
	usleep(100000);
	if (mount(dev_path.c_str(), "/mnt/tmp", fs_type.c_str(), MS_NOATIME, NULL))
	{
		ROS_ERROR("Mount block device failed!!");
		return false;
	}
	FILE *f = fopen("/mnt/tmp/random.file", "w");
	if (f == nullptr)
	{
		ROS_ERROR("Creating temporary file failed!!");
		umount2("/mnt/tmp", MNT_FORCE);
		return false;
	}
	fwrite(buf1, sizeof(char), static_cast<size_t>(write_length), f);
	fclose(f);
	system("/bin/sync");
	usleep(100000);
	f = fopen("/mnt/tmp/random.file", "r");
	if (f == nullptr)
	{
		ROS_ERROR("Reading from temporary file failed!!");
		umount2("/mnt/tmp", MNT_FORCE);
		return false;
	}
	char buf2[write_length];
	fread(buf2, sizeof(char), static_cast<size_t>(write_length), f);
	fclose(f);
	system("/bin/rm /mnt/tmp/random.file");
	usleep(100000);
	umount2("/mnt/tmp", MNT_FORCE);

	return strncmp(buf1, buf2, static_cast<size_t>(write_length)) == 0;
}*/
uint16_t electrical_specification_and_led_test(uint16_t *baseline, bool &is_fixture, uint8_t &test_stage)
{
	uint32_t temp_sum=0;
	uint8_t step=0;
	uint8_t count_20ms=0;
	uint8_t count_key_pressed=0;
	uint16_t baseline_voltage=0;
	uint8_t buf[REC_LEN];
	bool should_save_baseline = true;

	ROS_INFO("%s, %d", __FUNCTION__, __LINE__);
	serial.setSendData(CTL_MAIN_BOARD_MODE, ELECTRICAL_AND_LED_TEST_MODE);
	while(ros::ok())
	{
		serial.setSendData(CTL_MAIN_BOARD_MODE, ELECTRICAL_AND_LED_TEST_MODE);
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock)!=0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond,&recev_lock)!=0, "robotbase pthread receive cond wait fail");
		memcpy(buf,serial.receive_stream,sizeof(uint8_t)*REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock)!=0, "robotbase pthread receive unlock fail");
		if(buf[38] != ELECTRICAL_AND_LED_TEST_MODE)
		{
			serial.sendData();
			continue;
		}
		if(should_save_baseline)
		{
			should_save_baseline = false;
			baseline[LEFT_WHEEL] = ((uint16_t) buf[7] << 8) | buf[8];
			baseline[RIGHT_WHEEL] = ((uint16_t) buf[9] << 8) | buf[10];
			baseline[LEFT_BRUSH] = ((uint16_t) buf[11] << 8) | buf[12];
			baseline[MAIN_BRUSH] = ((uint16_t) buf[13] << 8) | buf[14];
			baseline[RIGHT_BRUSH] = ((uint16_t) buf[15] << 8) | buf[16];
			baseline[VACUUM] = ((uint16_t) buf[17] << 8) | buf[18];
			baseline[REF_VOLTAGE_ADC] = ((uint16_t) buf[19] << 8) | buf[20];
		}
		switch(step) {
			case 0:
				baseline_voltage = baseline[REF_VOLTAGE_ADC] * 330 / 4096;
				if (baseline_voltage < 125 || baseline_voltage > 135)
					return BASELINE_VOLTAGE_ERROR;
				step++;
				temp_sum = 0;
				count_20ms = 0;
			case 1:
				if (count_20ms < 20) {
					count_20ms++;
					temp_sum += static_cast<uint16_t>((buf[2] << 8) | buf[3]);
				}
				else {
					temp_sum /= 20;
					temp_sum = temp_sum *330 /4096 *835 /120;
					if (temp_sum < 1350 || temp_sum > 1700) {
						if (temp_sum < 1350)
							return BATTERY_LOW;
						else
							return BATTERY_ERROR;
					}
					temp_sum = 0;
					count_20ms = 0;
					step++;
				}
				break;
			case 2:/*---check baseline current---*/
				if (count_20ms < 10) {
					count_20ms++;
					temp_sum += static_cast<uint16_t>(buf[5] << 8) | buf[6];
				}
				else {
					temp_sum = (temp_sum / 10 - baseline[REF_VOLTAGE_ADC]) * 330 * 20 / 4096;
						if(temp_sum < 100 || temp_sum > 180)
						{
							ROS_INFO("baseline current: %d",temp_sum);
							if(temp_sum < 40)
								return BASELINE_CURRENT_LOW;
							else
								return BASELINE_CURRENT_ERROR;
						}
					baseline[SYSTEM_CURRENT] = static_cast<uint16_t>(temp_sum);
					temp_sum = 0;
					count_20ms = 0;
					step++;
				}
				break;
			case 3:/*---LED---*/
				count_20ms++;
				if (count_20ms < 20)
					key_led.set(100, 0);
				else if (count_20ms < 40)
					key_led.set(0, 100);
				else if (count_20ms < 60)
					key_led.set(100, 100);
				else if (count_20ms < 80)
				{
					key_led.set(0, 0);
					serial.setSendData(CTL_MIX, 1);
				}
				else
				{
					count_20ms = 0;
					serial.setSendData(CTL_MIX, 0);
				}
				if(buf[36])
				{
					count_key_pressed++;
				}
				else if(count_key_pressed)
				{
					if(count_key_pressed > 200)/*---long pressed, set is_fixture---*/
						is_fixture = true;
					else
						is_fixture = false;
					test_stage++;
				}
				break;
		}
		if(test_stage != ELECTRICAL_AND_LED_TEST_MODE)
			return 0;
		serial.sendData();
	}
}
uint16_t cliff_test(uint8_t &test_stage)
{
	uint16_t test_result = 0;
	uint8_t buf[REC_LEN];
	serial.setSendData(CTL_MAIN_BOARD_MODE, CLIFF_TEST_MODE);
	ROS_INFO("%s, %d", __FUNCTION__, __LINE__);
	while(ros::ok()) {
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock) != 0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond, &recev_lock) != 0, "robotbase pthread receive cond wait fail");
		memcpy(buf, serial.receive_stream, sizeof(uint8_t) * REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock) != 0, "robotbase pthread receive unlock fail");
		if(buf[38] != CLIFF_TEST_MODE)
		{
			serial.sendData();
			continue;
		}
		/*---Left Cliff---*/
		if (static_cast<uint16_t>(buf[2] << 8 | buf[3]) < 80) {
			test_result |= 0x0001;
		}
		if (static_cast<uint16_t>(buf[2] << 8 | buf[3]) > 1000) {
			test_result |= 0x0002;
		}
		/*---Front  Cliff---*/
		if (static_cast<uint16_t>(buf[4] << 8 | buf[5]) < 80) {
			test_result |= 0x0010;
		}
		if (static_cast<uint16_t>(buf[4] << 8 | buf[5]) > 1000) {
			test_result |= 0x0020;
		}
		/*---Right Cliff---*/
		if (static_cast<uint16_t>(buf[6] << 8 | buf[7]) < 80) {
			test_result |= 0x0100;
		}
		if (static_cast<uint16_t>(buf[6] << 8 | buf[7]) > 1000) {
			test_result |= 0x0200;
		}
		/*---Left Wheel Switch---*/
		if (buf[8] & 0x02) {
			test_result |= 0x1000;
		}
		else {
			test_result |= 0x2000;
		}
		/*---Right Wheel Switch---*/
		if (buf[8] & 0x01) {
			test_result |= 0x4000;
		}
		else {
			test_result |= 0x8000;
		}

		if ((test_result & 0xF333) == 0xF333) {
			test_stage++;
			return 0;
		}

		if(buf[36])
		{
			if((test_result & 0x0003) != 0x0003)
			{
				return LEFT_CLIFF_ERROR;
			}
			if((test_result & 0x0030) != 0x0030)
			{
				return FRONT_CLIFF_ERROR;
			}
			if((test_result & 0x0300) != 0x0300)
			{
				return RIGHT_CLIFF_ERROR;
			}
			if((test_result & 0x3000) != 0x3000)
			{
				return LEFT_WHEEL_SW_ERROR;
			}
			if((test_result & 0xC000) != 0xC000)
			{
				return RIGHT_WHEEL_SW_ERROR;
			}
		}
		serial.sendData();
	}
}
uint16_t bumper_test(uint8_t &test_stage)
{
	uint8_t test_result = 0;
	uint8_t buf[REC_LEN];
	serial.setSendData(CTL_MAIN_BOARD_MODE, BUMPER_TEST_MODE);
	ROS_INFO("%s, %d", __FUNCTION__, __LINE__);
	while(ros::ok())
	{
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock) != 0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond, &recev_lock) != 0, "robotbase pthread receive cond wait fail");
		memcpy(buf, serial.receive_stream, sizeof(uint8_t) * REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock) != 0, "robotbase pthread receive unlock fail");
		if(buf[38] != BUMPER_TEST_MODE)
		{
			serial.sendData();
			continue;
		}
		if (buf[28] & 0x20) {
			test_result |= 0x01;
		}
		else {
			test_result |= 0x02;
		}
		if (buf[28] & 0x01) {
			test_result |= 0x04;
		}
		else {
			test_result |= 0x08;
		}

		if ((test_result & 0x0f) == 0x0f) {
			test_stage++;
			return 0;
		}
		if (buf[36]) {
			if ((test_result & 0x03) != 0x03) {
				return LEFT_BUMPER_ERROR;
			}
			if ((test_result & 0x0c) != 0x0c) {
				return RIGHT_BUMPER_ERROR;
			}
		}
		serial.sendData();
	}
}
uint16_t obs_test(uint8_t &test_stage, bool is_fixture)
{
	uint16_t test_result = 0;
	uint8_t buf[REC_LEN];
	serial.setSendData(CTL_MAIN_BOARD_MODE, OBS_TEST_MODE);
	ROS_INFO("%s, %d", __FUNCTION__, __LINE__);
	while(ros::ok()) {
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock) != 0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond, &recev_lock) != 0, "robotbase pthread receive cond wait fail");
		memcpy(buf, serial.receive_stream, sizeof(uint8_t) * REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock) != 0, "robotbase pthread receive unlock fail");
		if(buf[38] != OBS_TEST_MODE)
		{
			serial.sendData();
			continue;
		}
		if (is_fixture) {
			if (static_cast<uint16_t >(buf[22] << 8 | buf[23]) > OBS_MANUAL_LIMIT_H)  //Left Obs
			{
				test_result |= 0x0001;
			}
			else if (static_cast<uint16_t>(buf[22] << buf[23]) < OBS_MANUAL_LIMIT_L) {
				test_result |= 0x0002;
			}
			if (static_cast<uint16_t>(buf[24] << 8 | buf[25]) > OBS_MANUAL_LIMIT_H)  //Front Obs
			{
				test_result |= 0x0004;
			}
			else if (static_cast<uint16_t>(buf[24] << 8 | buf[25]) < OBS_MANUAL_LIMIT_L) {
				test_result |= 0x0008;
			}
			if (static_cast<uint16_t>(buf[26] << 8 | buf[27]) > OBS_MANUAL_LIMIT_H) //Right Obs
			{
				test_result |= 0x0010;
			}
			else if (static_cast<uint16_t>(buf[26] << 8 | buf[27]) < OBS_MANUAL_LIMIT_L) {
				test_result |= 0x0020;
			}
			if (static_cast<uint16_t>(buf[18] << 8 | buf[19]) > WALL_MANUAL_LIMIT_H) //Left Wall
			{
				test_result |= 0x0040;
			}
			else if (static_cast<uint16_t>(buf[18] << 8 | buf[19]) < WALL_MANUAL_LIMIT_L) {
				test_result |= 0x0080;
			}
			if (static_cast<uint16_t>(buf[20] << 8 | buf[21]) > WALL_MANUAL_LIMIT_H) //Right Wall
			{
				test_result |= 0x0100;
			}
			else if (static_cast<uint16_t>(buf[20] << 8 | buf[21]) < WALL_MANUAL_LIMIT_L) {
				test_result |= 0x0200;
			}
		}
		else {
			if (static_cast<uint16_t>(buf[22] << 8 | buf[23]) > OBS_FIXTURE_LIMIT_H)  //Left Obs
			{
				test_result |= 0x0001;
			}
			else if (static_cast<uint16_t>(buf[22] << 8 | buf[23]) < OBS_FIXTURE_LIMIT_L) {
				test_result |= 0x0002;
			}
			if (static_cast<uint16_t>(buf[24] << 8 | buf[25]) > OBS_FIXTURE_LIMIT_H)  //Front Obs
			{
				test_result |= 0x0004;
			}
			else if (static_cast<uint16_t>(buf[24] << 8 | buf[25]) < OBS_FIXTURE_LIMIT_L) {
				test_result |= 0x0008;
			}
			if (static_cast<uint16_t>(buf[26] << 8 | buf[27]) > OBS_FIXTURE_LIMIT_H) //Right Obs
			{
				test_result |= 0x0010;
			}
			else if (static_cast<uint16_t>(buf[26] << 8 | buf[27]) < OBS_FIXTURE_LIMIT_L) {
				test_result |= 0x0020;
			}
			if (static_cast<uint16_t>(buf[18] << 8 | buf[19]) > WALL_FIXTURE_LIMIT_H) //Left Wall
			{
				test_result |= 0x0040;
			}
			else if (static_cast<uint16_t>(buf[18] << 8 | buf[19]) < WALL_FIXTURE_LIMIT_L) {
				test_result |= 0x0080;
			}
			if (static_cast<uint16_t>(buf[20] << 8 | buf[21]) > WALL_FIXTURE_LIMIT_H) //Right Wall
			{
				test_result |= 0x0100;
			}
			else if (static_cast<uint16_t>(buf[20] << 8 | buf[21]) < WALL_FIXTURE_LIMIT_L) {
				test_result |= 0x0200;
			}
		}

		if (is_fixture)  //fixture test
		{
			if (test_result == 0x2155)//test pass
			{
				test_stage++;
				return 0;
			}
			if (buf[36]) {
				if ((test_result & 0x0001) != 0x0001) {
					return LEFT_OBS_ERROR;
				}
				if ((test_result & 0x0004) != 0x0004) {
					return FRONT_OBS_ERROR;
				}
				if ((test_result & 0x0010) != 0x0010) {
					return RIGHT_OBS_ERROR;
				}
				if ((test_result & 0x0040) != 0x0040) {
					return LEFT_WALL_ERROR;
				}
				if ((test_result & 0x0100) != 0x0100) {
					return RIGHT_WALL_ERROR;
				}
			}
		}
		else   //manual mode
		{
			if (test_result == 0x23ff) {
				test_stage++;
				return 0;
			}

			if (buf[36]) {
				if ((test_result & 0x0003) != 0x0003) {
					return LEFT_OBS_ERROR;
				}
				if ((test_result & 0x000c) != 0x000c) {
					return FRONT_OBS_ERROR;
				}
				if ((test_result & 0x0030) != 0x0030) {
					return RIGHT_OBS_ERROR;
				}
				if ((test_result & 0x00c0) != 0x00c0) {
					return LEFT_WALL_ERROR;
				}
				if ((test_result & 0x0300) != 0x0300) {
					return RIGHT_WALL_ERROR;
				}
			}
		}
		serial.sendData();
	}
}
uint16_t rcon_test(uint8_t &test_stage)
{
	uint16_t test_result = 0;
	uint32_t Temp_Rcon_Status = 0;
	uint8_t buf[REC_LEN];
	serial.setSendData(CTL_MAIN_BOARD_MODE, RCON_TEST_MODE);
	ROS_INFO("%s, %d", __FUNCTION__, __LINE__);
	while(ros::ok()) {
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock) != 0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond, &recev_lock) != 0, "robotbase pthread receive cond wait fail");
		memcpy(buf, serial.receive_stream, sizeof(uint8_t) * REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock) != 0, "robotbase pthread receive unlock fail");
		if(buf[38] != RCON_TEST_MODE)
		{
			serial.sendData();
			continue;
		}
		Temp_Rcon_Status = static_cast<uint32_t>((buf[30] << 24) | (buf[31] << 16) | (buf[32] << 8) | buf[33]);
		if (Temp_Rcon_Status & (RconR_HomeL | RconR_HomeR))//right
		{
			test_result |= 0x0001;
		}
		else {
			test_result |= 0x0002;
		}

		if (Temp_Rcon_Status & (RconFR2_HomeL | RconFR2_HomeR))//front right 45
		{
			test_result |= 0x1000;
		}
		else {
			test_result |= 0x2000;
		}

		if (Temp_Rcon_Status & (RconFR_HomeL | RconFR_HomeR))// front right
		{
			test_result |= 0x0004;
		}
		else {
			test_result |= 0x0008;
		}

		if (Temp_Rcon_Status & (RconFL_HomeL | RconFL_HomeR))// front Left
		{
			test_result |= 0x0010;
		}
		else {
			test_result |= 0x0020;
		}

		if (Temp_Rcon_Status & (RconL_HomeL | RconL_HomeR))// Left
		{
			test_result |= 0x0040;
		}
		else {
			test_result |= 0x0080;
		}

		if (Temp_Rcon_Status & (RconFL2_HomeL | RconFL2_HomeR))//front left 45
		{
			test_result |= 0x4000;
		}
		else {
			test_result |= 0x8000;
		}

		if (Temp_Rcon_Status & (RconBL_HomeL | RconBL_HomeR))// Back Left
		{
			test_result |= 0x0100;
		}
		else {
			test_result |= 0x0200;
		}

		if (Temp_Rcon_Status & (RconBR_HomeL | RconBR_HomeR))// Back Right
		{
			test_result |= 0x0400;
		}
		else {
			test_result |= 0x0800;
		}

		if (test_result == 0xffff) {
			test_stage++;
			return 0;
		}
		if (buf[36]) {
			if ((test_result & 0x0300) != 0x0300) {
				return BLRCON_ERROR;
			}
			if ((test_result & 0x00c0) != 0x00c0) {
				return LRCON_ERROR;
			}
			if ((test_result & 0xc000) != 0xc000) {
				return FL2RCON_ERROR;
			}
			if ((test_result & 0x0030) != 0x0030) {
				return FLRCON_ERROR;
			}
			if ((test_result & 0x000c) != 0x000c) {
				return FRRCON_ERROR;
			}
			if ((test_result & 0x0003) != 0x0003) {
				return FR2RCON_ERROR;
			}
			if ((test_result & 0x3000) != 0x3000) {
				return RRCON_ERROR;
			}
			if ((test_result & 0x0c00) != 0x0c00) {
				return BRRCON_ERROR;
			}
		}
		serial.sendData();
	}
}
uint16_t wheels_test(uint8_t &test_stage, uint16_t *baseline)
{
	uint16_t test_result=0;
	uint8_t step=1;
	uint8_t count=0;
	uint8_t buf[REC_LEN];
	uint32_t current_current;
	uint32_t motor_current;
	serial.setSendData(CTL_MAIN_BOARD_MODE, WHEELS_TEST_MODE);
	serial.setSendData(CTL_LEFT_WHEEL_TEST_MODE, 0);
	serial.setSendData(CTL_RIGHT_WHEEL_TEST_MODE, 0);
	ROS_INFO("%s, %d", __FUNCTION__, __LINE__);
	while(ros::ok())
	{
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock) != 0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond, &recev_lock) != 0, "robotbase pthread receive cond wait fail");
		memcpy(buf, serial.receive_stream, sizeof(uint8_t) * REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock) != 0, "robotbase pthread receive unlock fail");
		if(buf[39] != WHEELS_TEST_MODE)
		{
			serial.sendData();
			continue;
		}
		switch(step) {
			case 1:
				wheel.setDirectionForward();
				wheel.setPidTargetSpeed(30, 0);
				count++;
				if (count > 50) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 2:
				count++;
				if (count < 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
					motor_current += static_cast<uint16_t>(buf[2] << 8 | buf[3]);
				}
				else {
					count = 0;
					current_current = (current_current / 10 * 330 / 4096 * 20) - baseline[SYSTEM_CURRENT];
					motor_current = (motor_current / 10 - baseline[LEFT_WHEEL]) * 330 / 4096 * 10;
					step++;
				}
				break;
			case 3:
				step++;
				if (current_current < 30 || current_current > 100 || motor_current < 20 || motor_current > 100) {
					return LEFT_WHEEL_FORWARD_CURRENT_ERROR;
				}
				else {
					test_result |= 0x0001;
				}
				break;
			case 4:
				step++;
				if (buf[4] > 95 || buf[4] < 50) {
					return LEFT_WHEEL_FORWARD_PWM_ERROR;
				}
				else {
					test_result |= 0x0002;
				}
				break;
			case 5:
				step++;
				if (buf[10] == 1) {
					return LEFT_WHEEL_FORWARD_ENCODER_ERROR;
				}
				else if (buf[10] == 2) {
					return LEFT_WHEEL_FORWARD_ENCODER_FAIL;
				}
				else {
					test_result |= 0x0004;
				}
				break;
			case 6:
				wheel.setDirectionBackward();
				wheel.setPidTargetSpeed(30, 0);
				count++;
				if (count > 50) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 7:
				count++;
				if (count < 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
					motor_current += static_cast<uint16_t>(buf[2] << 8 | buf[3]);
				}
				else {
					count = 0;
					current_current = (current_current / 10 * 330 / 4096 * 20) - baseline[SYSTEM_CURRENT];
					motor_current = (motor_current / 10 - baseline[RIGHT_WHEEL]) * 330 / 4096 * 10;
					step++;
				}
				break;
			case 8:
				step++;
				if (current_current < 30 || current_current > 100 || motor_current < 20 || motor_current > 100) {
					return LEFT_WHEEL_BACKWARD_CURRENT_ERROR;
				}
				else {
					test_result |= 0x0010;
				}
				break;
			case 9:
				step++;
				if (buf[4] > 95 || buf[4] < 50) {
					return LEFT_WHEEL_BACKWARD_PWM_ERROR;
				}
				else {
					test_result |= 0x0020;
				}
				break;
			case 10:
				step++;
				if (buf[10] == 1) {
					return LEFT_WHEEL_BACKWARD_ENCODER_ERROR;
				}
				else if (buf[10] == 2) {
					return LEFT_WHEEL_BACKWARD_ENCODER_FAIL;
				}
				else {
					test_result |= 0x0040;
				}
				break;
			case 11:
				wheel.setDirectionForward();
				wheel.setPidTargetSpeed(0, 30);
				count++;
				if (count > 50) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 12:
				count++;
				if (count < 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
					motor_current += static_cast<uint16_t>(buf[5] << 8 | buf[6]);
				}
				else {
					count = 0;
					current_current = (current_current / 10 * 330 / 4096 * 20) - baseline[SYSTEM_CURRENT];
					motor_current = (motor_current / 10 - baseline[RIGHT_WHEEL]) * 330 / 4096 * 10;
					step++;
				}
				break;
			case 13:
				step++;
				if (current_current < 30 || current_current > 100 || motor_current < 20 || motor_current > 100) {
					return RIGHT_WHEEL_FORWARD_CURRENT_ERROR;
				}
				else {
					test_result |= 0x0100;
				}
				break;
			case 14:
				step++;
				if (buf[7] > 95 || buf[7] < 50) {
					return RIGHT_WHEEL_FORWARD_PWM_ERROR;
				}
				else {
					test_result |= 0x0200;
				}
				break;
			case 15:
				step++;
				if (buf[11] == 1) {
					return RIGHT_WHEEL_FORWARD_ENCODER_ERROR;
				}
				else if (buf[11] == 2) {
					return RIGHT_WHEEL_FORWARD_ENCODER_FAIL;
				}
				else {
					test_result |= 0x0400;
				}
				break;
			case 16:
				wheel.setDirectionBackward();
				wheel.setPidTargetSpeed(0, 30);
				count++;
				if (count > 50) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 17:
				count++;
				if (count < 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
					motor_current += static_cast<uint16_t>(buf[5] << 8 | buf[6]);
				}
				else {
					count = 0;
					current_current = (current_current / 10 * 330 / 4096 * 20) - baseline[SYSTEM_CURRENT];
					motor_current = (motor_current / 10 - baseline[RIGHT_WHEEL]) * 330 / 4096 * 10;
					step++;
				}
				break;
			case 18:
				step++;
				if (current_current < 30 || current_current > 100 || motor_current < 20 || motor_current > 100) {
					return RIGHT_WHEEL_BACKWARD_CURRENT_ERROR;
				}
				else {
					test_result |= 0x1000;
				}
				break;
			case 19:
				step++;
				if (buf[7] > 95 || buf[7] < 50) {
					return RIGHT_WHEEL_BACKWARD_PWM_ERROR;
				}
				else {
					test_result |= 0x2000;
				}
				break;
			case 20:
				step++;
				if (buf[11] == 1) {
					return RIGHT_WHEEL_BACKWARD_ENCODER_ERROR;
				}
				else if (buf[11] == 2) {
					return RIGHT_WHEEL_BACKWARD_ENCODER_FAIL;
				}
				else {
					test_result |= 0x4000;
				}
				break;
			case 21:
				wheel.setDirectionForward();
				wheel.setPidTargetSpeed(RUN_TOP_SPEED, 0);
				serial.setSendData(CTL_LEFT_WHEEL_TEST_MODE, 1);
				count++;
				if (count > 50) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 22:
				count++;
				if(count < 250)
				{
					if(static_cast<uint16_t>(buf[2] << 8 | buf[3]) > 580)
					{
						count = 0;
						step++;
					}
				}
			case 23:
				count++;
				if (count < 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
				}
				else {
					step++;
					count = 0;
					current_current = (current_current / 10 * 330 / 4096 * 20) - baseline[SYSTEM_CURRENT];
					if(current_current < 800)
						return LEFT_WHEEL_STALL_ERROR;
					else
					{
						test_result |= 0x0008;
					}
				}
				break;
			case 24:
				wheel.setDirectionForward();
				wheel.setPidTargetSpeed(0, RUN_TOP_SPEED);
				serial.setSendData(CTL_RIGHT_WHEEL_TEST_MODE, 1);
				count++;
				if (count > 50) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 25:
				count++;
				if(count < 250)
				{
					if(static_cast<uint16_t>(buf[5] << 8 | buf[6]) > 580)
					{
						count = 0;
						step++;
					}
				}
			case 26:
				count++;
				if (count < 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
				}
				else {
					step++;
					count = 0;
					current_current = (current_current / 10 * 330 / 4096 * 20) - baseline[SYSTEM_CURRENT];
					if(current_current < 800)
						return RIGHT_WHEEL_STALL_ERROR;
					else
					{
						test_result |= 0x0800;
					}
				}
				break;
		}

		if((test_result & 0x7f7f) == 0x7f7f)
		{
			test_stage++;
			wheel.setPidTargetSpeed(0, 0);
			return 0;
		}
		serial.sendData();
	}
}
uint16_t brushes_test(uint8_t &test_stage, uint16_t *baseline)
{
	uint8_t test_result=0;
	uint32_t current_current=0;
	uint32_t motor_current=0;
	uint8_t buf[REC_LEN];
	uint8_t count=0;
	uint8_t step=1;

	serial.setSendData(CTL_MAIN_BOARD_MODE, BRUSHES_TEST_MODE);
	serial.setSendData(CTL_LEFT_BRUSH_TEST_MODE, 0);
	serial.setSendData(CTL_MAIN_BRUSH_TEST_MODE, 0);
	serial.setSendData(CTL_RIGHT_BRUSH_TEST_MODE, 0);
	ROS_INFO("%s, %d", __FUNCTION__, __LINE__);
	while(ros::ok()) {
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock) != 0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond, &recev_lock) != 0, "robotbase pthread receive cond wait fail");
		memcpy(buf, serial.receive_stream, sizeof(uint8_t) * REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock) != 0, "robotbase pthread receive unlock fail");
		if (buf[38] != BRUSHES_TEST_MODE)
		{
			serial.sendData();
			continue;
		}
		switch (step) {
			case 1:
				brush.setPWM(60, 0, 0);
				count++;
				if (count > 20) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 2:
				count++;
				if (count < 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
					motor_current += static_cast<uint16_t>(buf[2] << 8 | buf[3]);
				}
				else {
					count = 0;
					current_current = (current_current / 10 * 330 / 4096 * 20) - baseline[SYSTEM_CURRENT];
					motor_current = (motor_current / 10 - baseline[LEFT_BRUSH]) * 330 / 4096 * 10;
					step++;
				}
				break;
			case 3:
				step++;
				if (current_current < 30 || current_current > 120 || motor_current < 20 || motor_current > 110) {
					return LEFT_BRUSH_CURRENT_ERROR;
				}
				else {
					test_result |= 0x10;
				}
				serial.setSendData(CTL_LEFT_BRUSH_TEST_MODE, 1);
				break;
			case 4:
				count++;
				if (count < 250) {
					if (static_cast<uint16_t>(buf[2] << 8 | buf[2]) > baseline[LEFT_BRUSH] + 360) {
						count = 0;
						step++;
					}
				}
			case 5:
				count++;
				if (count < 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
				}
				else {
					step++;
					count = 0;
					current_current = (current_current / 10 * 330 / 4096 * 20) - baseline[SYSTEM_CURRENT];
					if (current_current < 250)
						return LEFT_BRUSH_STALL_ERROR;
					else
					{
						test_result |= 0x20;
					}
				}
				break;
			case 6:
				brush.setPWM(0, 60, 0);
				count++;
				if (count > 20) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 7:
				count++;
				if (count < 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
					motor_current += static_cast<uint16_t>(buf[4] << 8 | buf[5]);
				}
				else {
					count = 0;
					current_current = (current_current / 10 * 330 / 4096 * 20) - baseline[SYSTEM_CURRENT];
					motor_current = (motor_current / 10 - baseline[MAIN_BRUSH]) * 330 / 4096 * 10;
					step++;
				}
				break;
			case 8:
				step++;
				if (current_current < 100 || current_current > 300 || motor_current < 100 || motor_current > 300) {
					return MAIN_BRUSH_CURRENT_ERROR;
				}
				else {
					test_result |= 0x04;
				}
				serial.setSendData(CTL_MAIN_BRUSH_TEST_MODE, 1);
				break;
			case 9:
				count++;
				if (count < 250) {
					if (static_cast<uint16_t>(buf[2] << 8 | buf[2]) > baseline[MAIN_BRUSH] + 1150) {
						count = 0;
						step++;
					}
				}
			case 10:
				count++;
				if (count < 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
				}
				else {
					step++;
					count = 0;
					current_current = (current_current / 10 * 330 / 4096 * 20) - baseline[SYSTEM_CURRENT];
					if (current_current < 550)
						return MAIN_BRUSH_STALL_ERROR;
					else
					{
						test_result |= 0x08;
					}
				}
				break;
			case 11:
				brush.setPWM(0, 0, 60);
				count++;
				if (count > 5) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 12:
				count++;
				if (count < 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
					motor_current += static_cast<uint16_t>(buf[6] << 8 | buf[7]);
				}
				else {
					count = 0;
					current_current = (current_current / 10 * 330 / 4096 * 20) - baseline[SYSTEM_CURRENT];
					motor_current = (motor_current / 10 - baseline[RIGHT_BRUSH]) * 330 / 4096 * 10;
					step++;
				}
				break;
			case 13:
				step++;
				if (current_current < 30 || current_current > 120 || motor_current < 20 || motor_current > 110) {
					return RIGHT_BRUSH_CURRENT_ERROR;
				}
				else {
					test_result |= 0x01;
				}
				serial.setSendData(CTL_RIGHT_BRUSH_TEST_MODE, 1);
				break;
			case 14:
				count++;
				if (count < 250) {
					if (static_cast<uint16_t>(buf[6] << 8 | buf[7]) > baseline[RIGHT_BRUSH] + 360) {
						count = 0;
						step++;
					}
				}
			case 15:
				count++;
				if (count < 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
				}
				else {
					step++;
					count = 0;
					current_current = (current_current / 10 * 330 / 4096 * 20) - baseline[SYSTEM_CURRENT];
					if (current_current < 250)
						return RIGHT_BRUSH_STALL_ERROR;
					else
					{
						test_result |= 0x02;
					}
				}
				break;
		}
		if(test_result & 0x3f == 0x3f)
		{
			test_stage++;
			return 0;
		}
		serial.sendData();
	}
}
uint16_t vacuum_test(uint8_t &test_stage, uint16_t *baseline)
{
	uint16_t test_result=0;
	uint8_t step=1;
	uint8_t count=0;
	uint8_t buf[REC_LEN];
	uint32_t current_current;
	uint32_t motor_current;
	serial.setSendData(CTL_MAIN_BOARD_MODE, VACUUM_TEST_MODE);
	serial.setSendData(CTL_VACUUM_TEST_MODE, 0);
	ROS_INFO("%s, %d", __FUNCTION__, __LINE__);
	while(ros::ok()) {
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock) != 0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond, &recev_lock) != 0, "robotbase pthread receive cond wait fail");
		memcpy(buf, serial.receive_stream, sizeof(uint8_t) * REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock) != 0, "robotbase pthread receive unlock fail");
		if (buf[38] != VACUUM_TEST_MODE)
		{
			serial.sendData();
			continue;
		}
		switch (step) {
			case 1:
				vacuum.setMode(Vac_Normal);
				count++;
				if (count > 50) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 2:
				count++;
				if (count < 10) {
					current_current += (static_cast<uint16_t>(buf[4] << 8 | buf[5]) - baseline[REF_VOLTAGE_ADC]);
					motor_current += static_cast<uint16_t>(buf[2] << 8 | buf[3]);
				}
				else {
					count = 0;
					current_current = (current_current / 10 * 330 / 4096 * 20) - baseline[SYSTEM_CURRENT];
					motor_current = (motor_current / 10 - baseline[VACUUM]) * 330 / 4096 * 10;
					step++;
				}
				break;
			case 3:
				step++;
				if (current_current < 30 || current_current > 100 || motor_current < 20 || motor_current > 100) {
					return VACUUM_CURRENT_ERROR;
				}
				else {
					test_result |= 0x0001;
				}
				break;
			case 4:
				step++;
				if (buf[7] > 95 || buf[7] < 50) {
					return VACUUM_PWM_ERROR;
				}
				else {
					test_result |= 0x0002;
				}
				break;
			case 5:
				step++;
				if (buf[10] == 1) {
					return VACUUM_ENCODER_ERROR;
				}
				else if (buf[10] == 2) {
					return VACUUM_ENCODER_FAIL;
				}
				else {
					test_result |= 0x0004;
				}
				break;
			case 6:
				count++;
				if (count > 50) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 7:
				count++;
				if(count < 250)
				{
					if(static_cast<uint16_t>(buf[2] << 8 | buf[3]) > 580)
					{
						count = 0;
						step++;
					}
				}
			case 9:
				count++;
				if (count < 10) {
					current_current += (static_cast<uint16_t>(buf[4] << 8 | buf[5]) - baseline[REF_VOLTAGE_ADC]);
				}
				else {
					step++;
					count = 0;
					current_current = (current_current / 10 * 330 / 4096 * 20) - baseline[SYSTEM_CURRENT];
					if(current_current < 800)
						return VACUUM_STALL_ERROR;
					else
					{
						test_result |= 0x0008;
					}
				}
				break;
		}
		if((test_result&0x0008) == 0x0008)
		{
			test_stage++;
			return 0;
		}
		serial.sendData();
	}
}
//uint16_t charge_current_test(uint8_t &test_stage) {
//	uint16_t Temp_PWM = 0;
//	uint8_t Charge_Time = 0;
//	uint8_t ChargePwmLevel = 0;
//	extern uint32_t Battery_Voltage_IT;
//	while(1)
//	{
//		Current_Current=0;
//		ChargePwmLevel = Get_ChargePwmLevel(Battery_Voltage_IT);
//		while(1)	// Wait for charger plugin
//		{
//			delay(500);
//			Current_Current=Get_Charger_Voltage();
//			if((Current_Current>1700))break;
//		}
//		//	Set_Clean_Mode(Clean_Mode_Charging);  //To display charge mode
//		Timer15_Charge_Configuration();
//		Charge_PWM = 1;
//
//		Temp_PWM = 300;
//		Current_Current = 0;
//		Charge_Time = 0;
//		Beep(3);
//
//		while(1)
//		{
//			Charge_Time++;
//			if(Charge_Time>30)break;
//
//			if(ADC_Value.System_Current < BaseLIneADCV)
//				Current_Current = Get_Charge_Current(BaseLIneADCV);
//			else
//				Current_Current = 0;
//
//			if(Current_Current>300)//charge current over
//			{
//				if(Current_Current>400)
//				{
//					if(Temp_PWM<10)Temp_PWM=10;
//					Temp_PWM-=20;
//				}
//				else
//				{
//					if(Temp_PWM<2)Temp_PWM=2;
//					Temp_PWM-=2;
//				}
//			}
//			else
//			{
//				if(Current_Current<280)
//				{
//					Temp_PWM+=30;
//				}
//				else if(Current_Current<290)
//				{
//					Temp_PWM+=10;
//				}
//				else
//				{
//					Temp_PWM++;
//				}
//				if(Temp_PWM>1000)Temp_PWM=1000;
//			}
//
//			Charge_PWM = Temp_PWM;
//
//			delay(2500);
//		}
//
////		USPRINTF("finish,charge current %d,pwm :%d\n",Current_Current,Temp_PWM);
//		Charge_PWM = 0;
//
//		Work_Configuration();
//
//		if(Is_Fixture)
//		{
//			if((Temp_PWM < 350) || (Temp_PWM > 650))
//			{
//				return CHARGE_PWM_ERROR);
//				Wait_For_Return();
//			}
//		}
//		else
//		{
//			if(Temp_PWM < ((ChargePwmLevel-16)*10)||Temp_PWM > ((ChargePwmLevel+16)*10))
//			{
//				return CHARGE_PWM_ERROR);
//				Wait_For_Return();
//			}
//		}
//
//		if((Current_Current<280)||(Current_Current>320))
//		{
//			return CHARGE_CURRENT_ERROR);
//			Wait_For_Return();
//		}
//
//		while(1)  //Wait for remove charge power
//		{
//			delay(500);
//			Current_Current=Get_Charger_Voltage();
//			if(Current_Current<1600)break;
//		}
//
//		while(1)
//		{
//			delay(100);
//			if(Step_Switch())break;
//		}
//	}
//}//
#endif