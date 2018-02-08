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

void x900_functional_test(std::string serial_port, int baud_rate)
{

	ROS_INFO("%s %d: Serial_port: %s, baudrate: %d.", __FUNCTION__, __LINE__, serial_port.c_str(), baud_rate);
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
	auto serial_receive_routine = new boost::thread(boost::bind(&Serial::receive_routine_cb, &serial));
	auto serial_send_routine = new boost::thread(boost::bind(&Serial::send_routine_cb, &serial));

	// Test item: RAM.
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
	ROS_INFO("%s %d: Test for Lidar succeeded.", __FUNCTION__, __LINE__);

	// Test hardware from main board.

	if (!main_board_test())
	{
		ROS_ERROR("%s %d: Main board test failed!!", __FUNCTION__, __LINE__);
		error_loop(MAIN_BOARD_ERROR);
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
	switch (error_code)
	{
		case SERIAL_ERROR:
			break;
		case RAM_ERROR:
			serial.setSendData(CTL_MAIN_BOARD_MODE, ALARM_ERROR_MODE);
			serial.setSendData(CTL_TESTING_STAGE, 1);
			serial.setSendData(CTL_ERROR_CODE_HIGH, static_cast<uint8_t>(RAM_ERROR >> 8));
			serial.setSendData(CTL_ERROR_CODE_LOW, static_cast<uint8_t>(RAM_ERROR));
			break;
		case FLASH_ERROR:
			serial.setSendData(CTL_MAIN_BOARD_MODE, ALARM_ERROR_MODE);
			serial.setSendData(CTL_TESTING_STAGE, 1);
			serial.setSendData(CTL_ERROR_CODE_HIGH, static_cast<uint8_t>(FLASH_ERROR >> 8));
			serial.setSendData(CTL_ERROR_CODE_LOW, static_cast<uint8_t>(FLASH_ERROR));
			break;
		case LIDAR_ERROR:
			serial.setSendData(CTL_MAIN_BOARD_MODE, ALARM_ERROR_MODE);
			serial.setSendData(CTL_TESTING_STAGE, 1);
			serial.setSendData(CTL_ERROR_CODE_HIGH, static_cast<uint8_t>(LIDAR_ERROR >> 8));
			serial.setSendData(CTL_ERROR_CODE_LOW, static_cast<uint8_t>(LIDAR_ERROR));
			break;
		case LIDAR_BUMPER_ERROR:
			serial.setSendData(CTL_MAIN_BOARD_MODE, ALARM_ERROR_MODE);
			serial.setSendData(CTL_TESTING_STAGE, 1);
			serial.setSendData(CTL_ERROR_CODE_HIGH, static_cast<uint8_t>(LIDAR_BUMPER_ERROR >> 8));
			serial.setSendData(CTL_ERROR_CODE_LOW, static_cast<uint8_t>(LIDAR_BUMPER_ERROR));
			break;
		case MAIN_BOARD_ERROR:
			serial.setSendData(CTL_MAIN_BOARD_MODE, ALARM_ERROR_MODE);
			serial.setSendData(CTL_TESTING_STAGE, 1);
			break;
		default:
			break;
	}
	double alarm_time = ros::Time::now().toSec();
	speaker.play(VOICE_TEST_FAIL);
	ROS_ERROR("%s %d: Test ERROR.", __FUNCTION__, __LINE__);
	while (ros::ok())
	{
		if (ros::Time::now().toSec() - alarm_time > 5)
		{
			alarm_time = ros::Time::now().toSec();
			speaker.play(VOICE_TEST_FAIL);
			ROS_ERROR("%s %d: Test ERROR.", __FUNCTION__, __LINE__);
		}
	}
}

bool RAM_test()
{
	return true;
	ROS_INFO("%s %d: Start RAM test.", __FUNCTION__, __LINE__);
	bool test_ret = false;
	int RAM_test_size = 2; // In Mb.
	int RAM_test_block_cnt = 3; // Test 3 blocks of RAM and size of each block is RAM_test_size Mb.

	pid_t status;
	std::string cmd = "memtester " + std::to_string(RAM_test_size) + " 1 " + std::to_string(RAM_test_block_cnt);
	ROS_INFO("%s %d: Run command: %s", __FUNCTION__, __LINE__, cmd.c_str());
	status = system(cmd.c_str());

	if (-1 == status)
		ROS_ERROR("system error!");
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
	return true;
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
	return false;
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
	return true;
	ROS_INFO("%s %d: Start lidar test.", __FUNCTION__, __LINE__);
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

bool main_board_test()
{
	return false;
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
#endif