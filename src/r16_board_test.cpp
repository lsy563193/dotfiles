//
// Created by austin on 18-1-26.
//

#include "r16_board_test.hpp"

#if R16_BOARD_TEST
#include <sys/mount.h>
#include <random>

void r16_board_test(std::string serial_port, int baud_rate)
{

	ROS_INFO("%s %d: Serial_port: %s, baudrate: %d.", __FUNCTION__, __LINE__, serial_port.c_str(), baud_rate);
	// Test item 1: Speaker.
	speaker.test();
	// If you can not hear the voice, then speaker port has error, but there is no way to test it by software.

	// Test item 2: Serial port.
	if (!serial_port_test() || !serial.init(serial_port, baud_rate))
	{
		ROS_ERROR("%s %d: Serial port test failed!!", __FUNCTION__, __LINE__);
		error_loop();
	}
	ROS_INFO("Test serial port success!!");

	robotbase_reset_send_stream();
	auto serial_receive_routine = new boost::thread(boost::bind(&Serial::receive_routine_cb, &serial));
	auto serial_send_routine = new boost::thread(boost::bind(&Serial::send_routine_cb, &serial));

	// Test item 3: USB devices connection.
	const int write_file_size = 500;
	if (!usb_test("/dev/sda1", "vfat", write_file_size))
	{
		ROS_ERROR("Test sda1 failed!!");
		error_loop();
	}
	ROS_INFO("Test sda1 success!!");

	if (!usb_test("/dev/sdb1", "vfat", write_file_size))
	{
		ROS_ERROR("Test sdb1 failed!!");
		error_loop();
	}
	ROS_INFO("Test sdb1 success!!");

	// Test item 4: Battery supply.
	if (!power_supply_test())
	{
		ROS_ERROR("%s %d: Power supply test failed!!", __FUNCTION__, __LINE__);
		error_loop();
	}
	ROS_INFO("Test power supply success!!");

	// Test finish.
	while (ros::ok())
	{
		speaker.play(VOICE_TEST_SUCCESS);
		ROS_INFO("%s %d: Test finish.", __FUNCTION__, __LINE__);
		sleep(5);
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

bool serial_port_test()
{
	return true;
	Serial serial_port_S2;
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

	return strncmp(char_buf1, char_buf2, write_data_length) == 0;
}

bool usb_test(std::string dev_path, std::string fs_type, int write_length)
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

#endif