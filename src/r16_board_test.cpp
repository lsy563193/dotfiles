//
// Created by austin on 18-1-26.
//

#include "r16_board_test.hpp"

#if R16_BOARD_TEST
#include <sys/mount.h>
#include <random>

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

	// Test item 3: USB devices connection.
	const int write_file_size = 500;
	bool ret;
	ret = do_mount_and_test("/dev/sda1", "vfat", write_file_size);
	if (!ret)
	{
		ROS_ERROR("Test sda1 failed!!");
		error_loop();
	}
	ROS_INFO("Test sda1 successful!!");

	ret = do_mount_and_test("/dev/sdb1", "vfat", write_file_size);
	if (!ret)
	{
		ROS_ERROR("Test sdb1 failed!!");
		error_loop();
	}
	ROS_INFO("Test sdb1 successful!!");

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

bool do_mount_and_test(std::string dev_path, std::string fs_type, int write_length )
{
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

#endif