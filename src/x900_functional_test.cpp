//
// Created by austin on 18-1-26.
//

#include "x900_functional_test.hpp"

#include <sys/mount.h>
#include <sys/sysinfo.h>
#include <sys/vfs.h>
#include <random>
#include <wait.h>
#include "robot.hpp"
#include "infrared_display.hpp"

void x900_functional_test(std::string serial_port, int baud_rate, std::string lidar_bumper_dev)
{
	uint8_t test_stage=0;
	uint8_t test_result=0;
	uint16_t error_code=0;
	uint16_t current_data=0;
	/*--- disable serial send and robotbase thread ---*/
	send_thread_enable = false;
	robotbase_thread_enable = false;
	ROS_INFO("%s %d: Serial_port: %s, baudrate: %d, lidar_bumper_dev: %s.",
			 __FUNCTION__, __LINE__, serial_port.c_str(), baud_rate, lidar_bumper_dev.c_str());
	// Test item: Speaker.
	speaker.test();
	// If you can not hear the voice, then speaker port has error, but there is no way to test it by software.

	// Wait for the end of voice playing
	speaker.play(VOICE_NULL, false);
	usleep(200000);
	// Test item: Serial port.
	test_result = serial_port_test();
	if (test_result)
	{
		/*--- serial error ---*/
		if(test_result == 255) {
			ROS_ERROR("%s %d: Serial port test failed!!", __FUNCTION__, __LINE__);
			error_loop(FUNC_SERIAL_TEST_MODE, SERIAL_ERROR, 0);
		}
		else
		{
			ROS_ERROR("%s %d: Main board version failed!!", __FUNCTION__, __LINE__);
			error_loop(FUNC_SERIAL_TEST_MODE, MAIN_BOARD_VERSION_ERROR, test_result);
		}
	}
	ROS_INFO("Test serial port succeeded!!");

	recei_thread_enable = true;

	// Test hardware from main board.
	main_board_test(test_stage, error_code, current_data);
	if (error_code)
	{
		ROS_ERROR("%s %d: Main board test failed!!", __FUNCTION__, __LINE__);
		error_loop(test_stage, error_code, current_data);
	}
	ROS_INFO("Test main board success!!");

	// Test finish.
	double alarm_time = ros::Time::now().toSec();
	speaker.play(VOICE_TEST_SUCCESS);
	serial.setSendData(CTL_LED_GREEN, 100);
	serial.setSendData(CTL_LED_RED, 0);
	serial.setSendData(CTL_MIX, 0);
	infrared_display.displayNormalMsg(0, 9999);
	serial.sendData();
	ROS_INFO("%s %d: Test finish.", __FUNCTION__, __LINE__);
	while (ros::ok())
	{
		if (ros::Time::now().toSec() - alarm_time > 5)
		{
			alarm_time = ros::Time::now().toSec();
			speaker.play(VOICE_TEST_SUCCESS);
			serial.sendData();
			ROS_INFO("%s %d: Test finish.", __FUNCTION__, __LINE__);
		}
	}
}

void error_loop(uint8_t test_stage, uint16_t error_code, uint16_t current_data)
{
//	send_thread_enable = true;
	infrared_display.displayErrorMsg(test_stage-4, current_data, error_code);
	serial.setSendData(CTL_LED_RED, 100);
	serial.setSendData(CTL_LED_GREEN, 0);
	serial.setSendData(CTL_MIX, 0);
	serial.sendData();
	double alarm_time = ros::Time::now().toSec();
	speaker.play(VOICE_TEST_FAIL);
	ROS_ERROR("%s %d: Test ERROR.", __FUNCTION__, __LINE__);
	while (ros::ok())
	{
		if (ros::Time::now().toSec() - alarm_time > 5)
		{
			alarm_time = ros::Time::now().toSec();
			speaker.play(VOICE_TEST_FAIL);
			ROS_ERROR("%s %d: Test ERROR. test_stage_: %d. error_code: %d, current_data: %d", __FUNCTION__, __LINE__, test_stage, error_code, current_data);
			serial.sendData();
		}
	}
}

uint8_t serial_port_test()
{
	ROS_INFO("%s %d: Start serial test.", __FUNCTION__, __LINE__);
	uint8_t test_ret = 0;
	std::random_device rd;
	std::mt19937 random_number_engine(rd());
	std::uniform_int_distribution<uint8_t> dist_char;
	std::string send_string_sum{};
	std::string receive_string_sum{};
	uint8_t receive_data[REC_LEN];
	int test_frame_cnt = 50;

	infrared_display.displayNormalMsg(1, 0);
	serial.resetSendStream();
	serial.flush();
	for (uint8_t test_cnt = 0; test_cnt < test_frame_cnt; test_cnt++)
	{
		// Write random numbers to send stream.
		for (uint8_t i = CTL_WHEEL_LEFT_HIGH; i < CTL_IR_CTRL; i++)
		{
			if (i == CTL_WORK_MODE)
				serial.setSendData(i, FUNC_SERIAL_TEST_MODE);
			else if (i == CTL_BEEPER)
				serial.setSendData(i, static_cast<uint8_t>(test_cnt + 1));
			else if (i == CTL_KEY_VALIDATION)
				// Avoid 0x40/0x41/0x42/0x23.
				serial.setSendData(i, FUNC_SERIAL_TEST_MODE);
			/*--- for orange LED breath ---*/
			else if(i == CTL_LED_RED)
				serial.setSendData(i, test_cnt%10*10);
			else if(i == CTL_LED_GREEN)
				serial.setSendData(i, test_cnt%10*10);
			else
			{
				uint8_t random_byte = dist_char(random_number_engine);
				serial.setSendData(i, random_byte);
			}
			/*--- abandon the first package ---*/
			if(test_cnt > 0)
				send_string_sum += std::to_string(serial.getSendData(i));
		}
		serial.sendData();
//		serial.debugSendStream(serial.send_stream);

		int read_ret = serial.read(receive_data, REC_LEN);

		if (read_ret != REC_LEN)
		{
			ROS_ERROR("%s %d: Error during read:%d.", __FUNCTION__, __LINE__, read_ret);
			test_ret = 255;
			break;
		}

		if(test_cnt > 0) {
			for (uint8_t i = CTL_WHEEL_LEFT_HIGH; i < CTL_IR_CTRL; i++)
				receive_string_sum += std::to_string(receive_data[i]);
		}
		else
		{
			serial.flush();
			usleep(50000);
			serial.flush();
		}
//		serial.debugReceivedStream(receive_data);
		usleep(100000);
	}
	if(send_string_sum.compare(receive_string_sum) == 0)
	{
		if(receive_data[M0_VERSION_H] << 8 | receive_data[M0_VERSION_L] != 0)
			test_ret = receive_data[M0_VERSION_H] << 8 | receive_data[M0_VERSION_L];
	}
	else
		test_ret = 255;

	return test_ret;


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

void main_board_test(uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data)
{
	bool is_fixture = false;
	uint16_t test_result=0;
	uint16_t baseline[8];
	test_stage = FUNC_ELECTRICAL_AND_LED_TEST_MODE;
	while(ros::ok()) {
		key_led.set(100, 100);
		serial.setSendData(CTL_MIX, 1);
		infrared_display.displayNormalMsg(test_stage-4, 0);
		switch (test_stage) {
			case FUNC_ELECTRICAL_AND_LED_TEST_MODE:/*---Main board electrical specification and LED test---*/
				electrical_specification_and_led_test(baseline, is_fixture, test_stage, error_code, current_data);
				break;
			case FUNC_OBS_TEST_MODE:/*---OBS---*/
				obs_test(is_fixture, test_stage, error_code, current_data);
				break;
			case FUNC_BUMPER_TEST_MODE:/*---bumper---*/
				bumper_test(test_stage, error_code, current_data);
				break;
			case FUNC_CLIFF_TEST_MODE:/*---cliff---*/
				cliff_test(test_stage, error_code, current_data);
				break;
			case FUNC_RCON_TEST_MODE:/*---rcon---*/
				rcon_test(test_stage, error_code, current_data);
				break;
			case FUNC_WATER_TANK_TEST_MODE:/*---water tank---*/
				water_tank_test(test_stage, error_code, current_data);
				break;
			case FUNC_WHEELS_TEST_MODE:/*---wheels---*/
				wheels_test(baseline, test_stage, error_code, current_data);
				break;
			case FUNC_SIDEBRUSHES_TEST_MODE:/*---side brushes---*/
				side_brushes_test(baseline, test_stage, error_code, current_data);
				break;
			case FUNC_VACUUM_TEST_MODE:/*---vacuum---*/
				vacuum_test(baseline, test_stage, error_code, current_data);
				break;
			case FUNC_MAINBRUSH_TEST_MODE:/*--- main brush ---*/
				main_brush_test(baseline, test_stage, error_code, current_data);
				break;
			case FUNC_CHARGE_CURRENT_TEST_MODE:/*---charge current---*/
				charge_current_test(is_fixture, test_stage, error_code, current_data);
				break;
			case FUNC_FINISHED:
				test_stage = 0;
				error_code = 0;
				current_data = 0;
				return ;
		}
		if(error_code)
			return ;
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
void electrical_specification_and_led_test(uint16_t *baseline, bool &is_fixture, uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data)
{
	uint32_t temp_sum=0;
	uint8_t step=0;
	uint8_t count_20ms=0;
	uint8_t count_key_pressed=0;
	uint16_t baseline_voltage=0;
	uint8_t buf[REC_LEN];
	bool should_save_baseline = true;

	ROS_INFO("%s, %d", __FUNCTION__, __LINE__);
	serial.setSendData(CTL_WORK_MODE, FUNC_ELECTRICAL_AND_LED_TEST_MODE);
	serial.setSendData(CTL_BEEPER, 0);
	key_led.set(0, 0);
	serial.setSendData(CTL_MIX, 0);
	serial.sendData();
	while(ros::ok())
	{
		serial.setSendData(CTL_WORK_MODE, FUNC_ELECTRICAL_AND_LED_TEST_MODE);
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock)!=0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond,&recev_lock)!=0, "robotbase pthread receive cond wait fail");
		memcpy(buf,serial.receive_stream,sizeof(uint8_t)*REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock)!=0, "robotbase pthread receive unlock fail");
		if(buf[38] != FUNC_ELECTRICAL_AND_LED_TEST_MODE)
		{
			serial.sendData();
			continue;
		}
		if(should_save_baseline)
		{
			should_save_baseline = false;
			baseline[LEFT_WHEEL] = ((uint16_t) buf[4] << 8) | buf[5];
			baseline[RIGHT_WHEEL] = ((uint16_t) buf[6] << 8) | buf[7];
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
				{
					error_code = BASELINE_VOLTAGE_ERROR;
					current_data = baseline_voltage;
					return ;
				}
				if(baseline[LEFT_WHEEL] < 1550 || baseline[LEFT_WHEEL] > 1700)
				{
					error_code = BASELINE_LEFT_WHEEL_CURRENT_ERROR;
					current_data = baseline[LEFT_WHEEL];
					return ;
				}
				if(baseline[RIGHT_WHEEL] < 1550 || baseline[RIGHT_WHEEL] > 1700)
				{
					error_code = BASELINE_RIGHT_WHEEL_CURRENT_ERROR;
					current_data = baseline[RIGHT_WHEEL];
					return ;
				}
				if(baseline[LEFT_BRUSH] < 1550 || baseline[LEFT_BRUSH] > 1700)
				{
					error_code = BASELINE_LEFT_BRUSH_CURRENT_ERROR;
					current_data = baseline[LEFT_BRUSH];
					return ;
				}
				if(baseline[MAIN_BRUSH] < 1550 || baseline[MAIN_BRUSH] > 1700)
				{
					error_code = BASELINE_MAIN_BRUSH_CURRENT_ERROR;
					current_data = baseline[MAIN_BRUSH];
					return ;
				}
				if(baseline[RIGHT_BRUSH] < 1550 || baseline[RIGHT_BRUSH] > 1700)
				{
					error_code = BASELINE_RIGHT_BRUSH_CURRENT_ERROR;
					current_data = baseline[RIGHT_BRUSH];
					return ;
				}
				if(baseline[VACUUM] < 1550 || baseline[VACUUM] > 1700)
				{
					error_code = BASELINE_VACUUM_CURRENT_ERROR;
					current_data = baseline[VACUUM];
					return ;
				}
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
					temp_sum = temp_sum *330 /4096 *660 /100;
					ROS_INFO("%s, %d: battery voltage: %d", __FUNCTION__, __LINE__, temp_sum);
					if (temp_sum < 1350 || temp_sum > 1700) {
						if (temp_sum < 1350)
							error_code = BATTERY_LOW;
						else
							error_code = BATTERY_ERROR;
						current_data = static_cast<uint16_t>(temp_sum);
						return ;
					}
					temp_sum = 0;
					count_20ms = 0;
					step++;
				}
				break;
			case 2:/*---check baseline current---*/
				if (count_20ms < 10) {
					count_20ms++;
					temp_sum += static_cast<uint16_t>(buf[8] << 8) | buf[9];
				}
				else {
					temp_sum = (temp_sum / 10 - baseline[REF_VOLTAGE_ADC]) * 330 * 20 / 4096;
						if(temp_sum < 70 || temp_sum > 300)
						{
							ROS_INFO("baseline current: %d",temp_sum);
							if(temp_sum < 70)
								error_code = BASELINE_CURRENT_LOW;
							else
								error_code = BASELINE_CURRENT_ERROR;
							current_data = static_cast<uint16_t>(temp_sum);
							return ;
						}
					baseline[SYSTEM_CURRENT] = static_cast<uint16_t>(temp_sum);
					temp_sum = 0;
					count_20ms = 0;
					step++;
				}
				break;
			case 3:/*---LED---*/
					key_led.set(100, 100);
					serial.setSendData(CTL_MIX, 1);
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
		if(test_stage != FUNC_ELECTRICAL_AND_LED_TEST_MODE)
			return ;
		serial.sendData();
	}
}
void cliff_test(uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data)
{
	uint16_t test_result = 0;
	uint8_t buf[REC_LEN];
	serial.setSendData(CTL_WORK_MODE, FUNC_CLIFF_TEST_MODE);
	ROS_INFO("%s, %d", __FUNCTION__, __LINE__);
	while(ros::ok()) {
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock) != 0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond, &recev_lock) != 0, "robotbase pthread receive cond wait fail");
		memcpy(buf, serial.receive_stream, sizeof(uint8_t) * REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock) != 0, "robotbase pthread receive unlock fail");
		if(buf[38] != FUNC_CLIFF_TEST_MODE)
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
			return ;
		}

		if(buf[36])
		{
			if((test_result & 0x0003) != 0x0003)
				error_code = LEFT_CLIFF_ERROR;
			if((test_result & 0x0030) != 0x0030)
				error_code = FRONT_CLIFF_ERROR;
			if((test_result & 0x0300) != 0x0300)
				error_code = RIGHT_CLIFF_ERROR;
			if((test_result & 0x3000) != 0x3000)
				error_code = LEFT_WHEEL_SW_ERROR;
			if((test_result & 0xC000) != 0xC000)
				error_code = RIGHT_WHEEL_SW_ERROR;
			current_data = test_result;
			return ;
		}
		serial.sendData();
	}
}
void bumper_test(uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data)
{
	uint8_t test_result = 0;
	uint8_t buf[REC_LEN];
	serial.setSendData(CTL_WORK_MODE, FUNC_BUMPER_TEST_MODE);
	ROS_INFO("%s, %d", __FUNCTION__, __LINE__);
	while(ros::ok())
	{
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock) != 0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond, &recev_lock) != 0, "robotbase pthread receive cond wait fail");
		memcpy(buf, serial.receive_stream, sizeof(uint8_t) * REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock) != 0, "robotbase pthread receive unlock fail");
		if(buf[38] != FUNC_BUMPER_TEST_MODE)
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
		if (buf[28] & 0x10) {
			test_result |= 0x04;
		}
		else {
			test_result |= 0x08;
		}

		if ((test_result & 0x0f) == 0x0f) {
			test_stage++;
			return ;
		}
		if (buf[36]) {
			if ((test_result & 0x03) != 0x03)
				error_code = LEFT_BUMPER_ERROR;
			if ((test_result & 0x0c) != 0x0c)
				error_code = RIGHT_BUMPER_ERROR;
			current_data = test_result;
		}
		serial.sendData();
	}
}
void obs_test(bool is_fixture, uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data)
{
	uint16_t test_result = 0;
	uint8_t buf[REC_LEN];
	serial.setSendData(CTL_WORK_MODE, FUNC_OBS_TEST_MODE);
	ROS_INFO("%s, %d", __FUNCTION__, __LINE__);
	while(ros::ok()) {
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock) != 0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond, &recev_lock) != 0, "robotbase pthread receive cond wait fail");
		memcpy(buf, serial.receive_stream, sizeof(uint8_t) * REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock) != 0, "robotbase pthread receive unlock fail");
		if(buf[38] != FUNC_OBS_TEST_MODE)
		{
			serial.sendData();
			continue;
		}
		if (!is_fixture) {
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
		}

		if (is_fixture)  //fixture test
		{
			if ((test_result & 0x0015) == 0x0015)//test pass
			{
				test_stage++;
				return ;
			}
			if (buf[36]) {
				if ((test_result & 0x0001) != 0x0001)
					error_code = LEFT_OBS_ERROR;
				if ((test_result & 0x0004) != 0x0004)
					error_code = FRONT_OBS_ERROR;
				if ((test_result & 0x0010) != 0x0010)
					error_code = RIGHT_OBS_ERROR;
				current_data = test_result;
				return ;
			}
		}
		else   //manual mode
		{
			if ((test_result & 0x003F) == 0x003f) {
				test_stage++;
				return ;
			}

			if (buf[36]) {
				if ((test_result & 0x0003) != 0x0003)
					error_code = LEFT_OBS_ERROR;
				if ((test_result & 0x000c) != 0x000c)
					error_code = FRONT_OBS_ERROR;
				if ((test_result & 0x0030) != 0x0030)
					error_code = RIGHT_OBS_ERROR;
				current_data = test_result;
				return ;
			}
		}
		serial.sendData();
	}
}
void rcon_test(uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data)
{
	uint16_t test_result = 0;
	uint32_t Temp_Rcon_Status = 0;
	uint8_t buf[REC_LEN];
	serial.setSendData(CTL_WORK_MODE, FUNC_RCON_TEST_MODE);
	ROS_INFO("%s, %d", __FUNCTION__, __LINE__);
	while(ros::ok()) {
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock) != 0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond, &recev_lock) != 0, "robotbase pthread receive cond wait fail");
		memcpy(buf, serial.receive_stream, sizeof(uint8_t) * REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock) != 0, "robotbase pthread receive unlock fail");
		if(buf[38] != FUNC_RCON_TEST_MODE)
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
			return ;
		}
		if (buf[36]) {
			if ((test_result & 0x0300) != 0x0300)
				error_code = BLRCON_ERROR;
			if ((test_result & 0x00c0) != 0x00c0)
				error_code = LRCON_ERROR;
			if ((test_result & 0xc000) != 0xc000)
				error_code = FL2RCON_ERROR;
			if ((test_result & 0x0030) != 0x0030)
				error_code = FLRCON_ERROR;
			if ((test_result & 0x000c) != 0x000c)
				error_code = FRRCON_ERROR;
			if ((test_result & 0x0003) != 0x0003)
				error_code = FR2RCON_ERROR;
			if ((test_result & 0x3000) != 0x3000)
				error_code = RRCON_ERROR;
			if ((test_result & 0x0c00) != 0x0c00)
				error_code = BRRCON_ERROR;
			current_data = test_result;
			return ;
		}
		serial.sendData();
	}
}
void water_tank_test(uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data)
{
	uint8_t step = 0;
	uint8_t count = 0;
	uint8_t test_result = 0;
	uint8_t buf[REC_LEN];
	serial.setSendData(CTL_WORK_MODE, FUNC_WATER_TANK_TEST_MODE);
	ROS_INFO("%s, %d", __FUNCTION__, __LINE__);
	while(ros::ok()) {
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock) != 0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond, &recev_lock) != 0, "robotbase pthread receive cond wait fail");
		memcpy(buf, serial.receive_stream, sizeof(uint8_t) * REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock) != 0, "robotbase pthread receive unlock fail");
		if(buf[38] != FUNC_WATER_TANK_TEST_MODE)
		{
			serial.sendData();
			continue;
		}
		switch(step)
		{
			case 0:/*--- turn on pump and swing motor ---*/
				serial.setSendData(CTL_WATER_TANK, 0x80 | 40);
				count++;
				if(count > 10)
				{
					count = 0;
					step++;
				}
				break;
			case 1:
				if(static_cast<uint16_t>(buf[2] << 8 | buf[3]) > SWING_CURRENT_LIMIT)
				{
					if(count < 200)count++;
				}
				if(count > 20)
					test_result |= 0x01;
				if(buf[36])
				{
					if((test_result & 0x01) != 0x01)
					{
						error_code = SWING_MOTOR_ERROR;
						current_data = static_cast<uint16_t>((buf[2] << 8) | buf[3]);
					}
					else
						test_stage++;
					return ;
				}
				break;
		}
		serial.sendData();
	}
}
void wheels_test(uint16_t *baseline, uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data)
{
	uint16_t test_result=0;
	uint8_t step=1;
	uint8_t count=0;
	uint8_t buf[REC_LEN];
	uint32_t current_current=0;
	uint32_t motor_current=0;
	serial.setSendData(CTL_WORK_MODE, FUNC_WHEELS_TEST_MODE);
	serial.setSendData(CTL_WHEEL_LEFT_HIGH, 0);
	serial.setSendData(CTL_WHEEL_LEFT_LOW, 0);
	serial.setSendData(CTL_WHEEL_RIGHT_HIGH, 0);
	serial.setSendData(CTL_WHEEL_RIGHT_LOW, 0);
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
		if(buf[38] != FUNC_WHEELS_TEST_MODE)
		{
			serial.sendData();
			continue;
		}
		switch(step) {
			case 1:
				serial.setSendData(CTL_WHEEL_LEFT_HIGH, 200 >> 8);
				serial.setSendData(CTL_WHEEL_LEFT_LOW, 200 & 0xFF);
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
				if (count <= 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
					motor_current += static_cast<uint16_t>(buf[2] << 8 | buf[3]);
				}
				else {
					count = 0;
					current_current = (current_current / 10 * 330 * 20 / 4096) - baseline[SYSTEM_CURRENT];
					motor_current = (motor_current / 10 - baseline[LEFT_WHEEL]) * 330 * 10 / 4096;
					step++;
				}
				break;
			case 3:
				step++;
				if (current_current < 30 || current_current > 150 || motor_current < 20 || motor_current > 100) {
					error_code = LEFT_WHEEL_FORWARD_CURRENT_ERROR;
					current_data = static_cast<uint16_t>(motor_current);
					return ;
				}
				else {
					test_result |= 0x0001;
				}
				break;
			case 4:
				step++;
				if (buf[4] > 90 || buf[4] < 40) {
					error_code = LEFT_WHEEL_FORWARD_PWM_ERROR;
					current_data = buf[4];
					return ;
				}
				else {
					test_result |= 0x0002;
				}
				break;
			case 5:
				step++;
				if (buf[10] == 1) {
					error_code = LEFT_WHEEL_FORWARD_ENCODER_ERROR;
					current_data = 0;
					return ;
				}
				else if (buf[10] == 2) {
					error_code = LEFT_WHEEL_FORWARD_ENCODER_FAIL;
					current_data = 0;
					return ;
				}
				else {
					test_result |= 0x0004;
				}
				break;
			case 6:
				serial.setSendData(CTL_WHEEL_LEFT_HIGH, -200 >> 8);
				serial.setSendData(CTL_WHEEL_LEFT_LOW, -200 & 0xFF);
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
				if (count <= 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
					motor_current += static_cast<uint16_t>(buf[2] << 8 | buf[3]);
				}
				else {
					count = 0;
					current_current = (current_current / 10 * 330 * 20 / 4096) - baseline[SYSTEM_CURRENT];
					motor_current = (motor_current / 10 - baseline[RIGHT_WHEEL]) * 330 * 10 / 4096;
					step++;
				}
				break;
			case 8:
				step++;
				if (current_current < 30 || current_current > 150 || motor_current < 20 || motor_current > 100) {
					error_code = LEFT_WHEEL_BACKWARD_CURRENT_ERROR;
					current_data = static_cast<uint16_t>(motor_current);
					return ;
				}
				else {
					test_result |= 0x0010;
				}
				break;
			case 9:
				step++;
				if (buf[4] > 90 || buf[4] < 40) {
					error_code = LEFT_WHEEL_BACKWARD_PWM_ERROR;
					current_data = buf[4];
					return ;
				}
				else {
					test_result |= 0x0020;
				}
				break;
			case 10:
				step++;
				if (buf[10] == 1) {
					error_code = LEFT_WHEEL_BACKWARD_ENCODER_ERROR;
					current_data = 0;
					return ;
				}
				else if (buf[10] == 2) {
					error_code = LEFT_WHEEL_BACKWARD_ENCODER_FAIL;
					current_data = 0;
					return ;
				}
				else {
					test_result |= 0x0040;
				}
				break;
			case 11:
				serial.setSendData(CTL_WHEEL_LEFT_HIGH, 0);
				serial.setSendData(CTL_WHEEL_LEFT_LOW, 0);
				count++;
				if(count > 10)
				{
					count = 0;
					step++;
				}
				break;
			case 12:
				serial.setSendData(CTL_WHEEL_LEFT_HIGH, 200 >> 8);
				serial.setSendData(CTL_WHEEL_LEFT_LOW, 200 & 0xFF);
				serial.setSendData(CTL_LEFT_WHEEL_TEST_MODE, 1);
				count++;
				if (count > 50) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 13:
				if(static_cast<uint16_t>(buf[2] << 8 | buf[3]) > baseline[LEFT_WHEEL] + 580)
					count++;
				else
					count = 0;
				if(count > 2)
				{
					step++;
					count = 0;
				}
				if(buf[36]) {
					error_code = LEFT_WHEEL_STALL_ERROR;
					current_data = 0;
					return;
				}
				break;
			case 14:
				count++;
				if (count <= 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
				}
				else {
					step++;
					count = 0;
					current_current = (current_current / 10 * 330 * 20 / 4096) - baseline[SYSTEM_CURRENT];
					if(current_current < 800) {
						error_code = LEFT_WHEEL_STALL_ERROR;
						current_data = 0;
						return ;
					}
					else
					{
						test_result |= 0x0008;
					}
				}
				break;
			case 15:
				serial.setSendData(CTL_WHEEL_LEFT_HIGH, 0);
				serial.setSendData(CTL_WHEEL_LEFT_LOW, 0);
				serial.setSendData(CTL_WHEEL_RIGHT_HIGH, 200 >> 8);
				serial.setSendData(CTL_WHEEL_RIGHT_LOW, 200 & 0xFF);
				count++;
				if (count > 100) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 16:
				count++;
				if (count <= 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
					motor_current += static_cast<uint16_t>(buf[5] << 8 | buf[6]);
				}
				else {
					count = 0;
					current_current = (current_current / 10 * 330 * 20 / 4096) - baseline[SYSTEM_CURRENT];
					motor_current = (motor_current / 10 - baseline[RIGHT_WHEEL]) * 330 * 10 / 4096;
					step++;
				}
				break;
			case 17:
				step++;
				if (current_current < 30 || current_current > 150 || motor_current < 20 || motor_current > 100) {
					error_code = RIGHT_WHEEL_FORWARD_CURRENT_ERROR;
					current_data = motor_current;
					return ;
				}
				else {
					test_result |= 0x0100;
				}
				break;
			case 18:
				step++;
				if (buf[7] > 90 || buf[7] < 40) {
					error_code = RIGHT_WHEEL_FORWARD_PWM_ERROR;
					current_data = buf[7];
					return ;
				}
				else {
					test_result |= 0x0200;
				}
				break;
			case 19:
				step++;
				if (buf[11] == 1) {
					error_code = RIGHT_WHEEL_FORWARD_ENCODER_ERROR;
					current_data = 0;
					return ;
				}
				else if (buf[11] == 2) {
					error_code = RIGHT_WHEEL_FORWARD_ENCODER_FAIL;
					current_data = 0;
					return ;
				}
				else {
					test_result |= 0x0400;
				}
				break;
			case 20:
				serial.setSendData(CTL_WHEEL_RIGHT_HIGH, -200 >> 8);
				serial.setSendData(CTL_WHEEL_RIGHT_LOW, -200 & 0xFF);
				count++;
				if (count > 50) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 21:
				count++;
				if (count <= 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
					motor_current += static_cast<uint16_t>(buf[5] << 8 | buf[6]);
				}
				else {
					count = 0;
					current_current = (current_current / 10 * 330 * 20 / 4096) - baseline[SYSTEM_CURRENT];
					motor_current = (motor_current / 10 - baseline[RIGHT_WHEEL]) * 330 * 10 / 4096;
					step++;
				}
				break;
			case 22:
				step++;
				if (current_current < 30 || current_current > 150 || motor_current < 20 || motor_current > 100) {
					error_code = RIGHT_WHEEL_BACKWARD_CURRENT_ERROR;
					current_data = motor_current;
					return ;
				}
				else {
					test_result |= 0x1000;
				}
				break;
			case 23:
				step++;
				if (buf[7] > 90 || buf[7] < 40) {
					error_code = RIGHT_WHEEL_BACKWARD_PWM_ERROR;
					current_data = buf[7];
					return ;
				}
				else {
					test_result |= 0x2000;
				}
				break;
			case 24:
				step++;
				if (buf[11] == 1) {
					error_code = RIGHT_WHEEL_BACKWARD_ENCODER_ERROR;
					current_data = 0;
					return ;
				}
				else if (buf[11] == 2) {
					error_code = RIGHT_WHEEL_BACKWARD_ENCODER_FAIL;
					current_data = 0;
					return ;
				}
				else {
					test_result |= 0x4000;
				}
			case 25:
				serial.setSendData(CTL_WHEEL_RIGHT_HIGH, 0);
				serial.setSendData(CTL_WHEEL_RIGHT_LOW, 0);
				count++;
				if(count > 10)
				{
					count = 0;
					step++;
				}
				break;
			case 26:
				serial.setSendData(CTL_WHEEL_RIGHT_HIGH, 200 >> 8);
				serial.setSendData(CTL_WHEEL_RIGHT_LOW, 200 & 0xFF);
				serial.setSendData(CTL_RIGHT_WHEEL_TEST_MODE, 1);
				count++;
				if (count > 50) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 27:
				if(static_cast<uint16_t>(buf[5] << 8 | buf[6]) > baseline[RIGHT_WHEEL] + 580)
					count++;
				else
					count = 0;
				if(count > 2)
				{
					step++;
					count = 0;
				}
				if(buf[36])
				{
					error_code = RIGHT_WHEEL_STALL_ERROR;
					current_data = 0;
					return ;
				}
				break;
			case 28:
				count++;
				if (count <= 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
				}
				else {
					step++;
					count = 0;
					current_current = (current_current / 10 * 330 * 20 / 4096) - baseline[SYSTEM_CURRENT];
					if(current_current < 800)
					{
						error_code = RIGHT_WHEEL_STALL_ERROR;
						current_data = 0;
						return ;
					}
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
			return ;
		}
		serial.sendData();
	}
}
void side_brushes_test(uint16_t *baseline, uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data)
{
	uint8_t test_result=0;
	uint32_t current_current=0;
	uint32_t motor_current=0;
	uint8_t buf[REC_LEN];
	uint8_t count=0;
	uint8_t step=1;

	serial.setSendData(CTL_WORK_MODE, FUNC_SIDEBRUSHES_TEST_MODE);
	serial.setSendData(CTL_LEFT_BRUSH_TEST_MODE, 0);
	serial.setSendData(CTL_MAIN_BRUSH_TEST_MODE, 0);
	serial.setSendData(CTL_RIGHT_BRUSH_TEST_MODE, 0);
	brush.setPWM(0,0,0);
	ROS_INFO("%s, %d", __FUNCTION__, __LINE__);
	while(ros::ok()) {
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock) != 0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond, &recev_lock) != 0, "robotbase pthread receive cond wait fail");
		memcpy(buf, serial.receive_stream, sizeof(uint8_t) * REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock) != 0, "robotbase pthread receive unlock fail");
		if (buf[38] != FUNC_SIDEBRUSHES_TEST_MODE)
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
				if (count <= 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
					motor_current += static_cast<uint16_t>(buf[2] << 8 | buf[3]);
				}
				else {
					count = 0;
					current_current = (current_current / 10 * 330 * 20 / 4096) - baseline[SYSTEM_CURRENT];
					motor_current = (motor_current / 10 - baseline[LEFT_BRUSH]) * 330 * 10 / 4096;
					step++;
//					ROS_INFO("current: %d, motor: %d, baseline: %d", current_current, motor_current, baseline[LEFT_BRUSH]);
				}
				break;
			case 3:
				step++;
				if (current_current < 30 || current_current > 120 || motor_current < 20 || motor_current > 110) {
					error_code = LEFT_BRUSH_CURRENT_ERROR;
					current_data = static_cast<uint16_t>(motor_current);
					return ;
				}
				else {
					test_result |= 0x04;
				}
				serial.setSendData(CTL_LEFT_BRUSH_TEST_MODE, 1);
				break;
			case 4:
				brush.setPWM(0,0,0);
				count++;
				if(count > 25)
				{
					count = 0;
					step++;
				}
				break;
			case 5:
				brush.setPWM(60, 0, 0);
				count++;
				if (count > 20) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 6:
				if (static_cast<uint16_t>(buf[2] << 8 | buf[3]) > baseline[LEFT_BRUSH] + 360)
					count++;
				else
					count = 0;
				if(count > 2)
				{
					step++;
					count = 0;
				}
				if(buf[36])
				{
					error_code = LEFT_BRUSH_STALL_ERROR;
					current_data = 0;
					return ;
				}
				break;
			case 7:
				count++;
				if (count <= 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
				}
				else {
					step++;
					count = 0;
					current_current = (current_current / 10 * 330 * 20 / 4096) - baseline[SYSTEM_CURRENT];
					if (current_current < 250)
					{
						error_code = LEFT_BRUSH_STALL_ERROR;
						current_data = 0;
						return ;
					}
					else
					{
						test_result |= 0x08;
					}
				}
				break;
			case 8:
				brush.setPWM(0, 60, 0);
				count++;
				if (count > 5) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 9:
				count++;
				if (count <= 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
					motor_current += static_cast<uint16_t>(buf[6] << 8 | buf[7]);
				}
				else {
					count = 0;
					current_current = (current_current / 10 * 330 * 20 / 4096) - baseline[SYSTEM_CURRENT];
					motor_current = (motor_current / 10 - baseline[RIGHT_BRUSH]) * 330 * 10 / 4096;
					step++;
				}
				break;
			case 10:
				step++;
				if (current_current < 30 || current_current > 120 || motor_current < 20 || motor_current > 110) {
					error_code = RIGHT_BRUSH_CURRENT_ERROR;
					current_data = static_cast<uint16_t>(motor_current);
					return ;
				}
				else {
					test_result |= 0x01;
				}
				serial.setSendData(CTL_RIGHT_BRUSH_TEST_MODE, 1);
				break;
			case 11:
				brush.setPWM(0,0,0);
				count++;
				if(count > 25)
				{
					count = 0;
					step++;
				}
				break;
			case 12:
				brush.setPWM(0, 60, 0);
				count++;
				if (count > 20) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 13:
				if (static_cast<uint16_t>(buf[6] << 8 | buf[7]) > baseline[RIGHT_BRUSH] + 360)
					count++;
				else
					count = 0;
				if(count > 2)
				{
					step++;
					count = 0;
				}
				if(buf[36])
				{
					error_code = RIGHT_BRUSH_STALL_ERROR;
					current_data = 0;
					return ;
				}
				break;
			case 14:
				count++;
				if (count <= 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
				}
				else {
					step++;
					count = 0;
					current_current = (current_current / 10 * 330 * 20 / 4096) - baseline[SYSTEM_CURRENT];
					if (current_current < 250)
					{
						error_code = RIGHT_BRUSH_STALL_ERROR;
						current_data = 0;
						return ;
					}
					else
					{
						test_result |= 0x02;
					}
				}
				break;
		}
		if((test_result & 0x0f) == 0x0f)
		{
			test_stage++;
			return ;
		}
		serial.sendData();
	}
}
void vacuum_test(uint16_t *baseline, uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data)
{
	uint16_t test_result=0;
	uint8_t step=1;
	uint8_t count=0;
	uint8_t buf[REC_LEN];
	uint32_t current_current=0;
	uint32_t motor_current=0;
	serial.setSendData(CTL_WORK_MODE, FUNC_VACUUM_TEST_MODE);
	serial.setSendData(CTL_VACUUM_TEST_MODE, 0);
	ROS_INFO("%s, %d", __FUNCTION__, __LINE__);
	while(ros::ok()) {
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock) != 0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond, &recev_lock) != 0, "robotbase pthread receive cond wait fail");
		memcpy(buf, serial.receive_stream, sizeof(uint8_t) * REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock) != 0, "robotbase pthread receive unlock fail");
		if (buf[38] != FUNC_VACUUM_TEST_MODE)
		{
			serial.sendData();
			continue;
		}
		switch (step) {
			case 1:
				serial.setSendData(CTL_VACCUM_PWR, 80);
				count++;
				if (count > 100) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 2:
				count++;
				if (count <= 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
					motor_current += static_cast<uint16_t>(buf[2] << 8 | buf[3]);
				}
				else {
					count = 0;
					current_current = (current_current / 10 * 330 * 20 / 4096) - baseline[SYSTEM_CURRENT];
					motor_current = (motor_current / 10 - baseline[VACUUM]) * 330 * 10 / 4096;
					step++;
				}
				break;
			case 3:
				step++;
				if (current_current < 500 || current_current > 1100 || motor_current < 500 || motor_current > 1100) {
					error_code = VACUUM_CURRENT_ERROR;
					current_data = static_cast<uint16_t>(motor_current);
					ROS_INFO("motor: %d, current: %d", motor_current,current_current);
					return ;
				}
				else {
					test_result |= 0x0001;
				}
				break;
			case 4:
				step++;
				if (buf[7] > 95 || buf[7] < 50) {
					error_code = VACUUM_PWM_ERROR;
					current_data = buf[7];
					return ;
				}
				else {
					test_result |= 0x0002;
				}
				break;
			case 5:
				step++;
				if (buf[6] == 1) {
					error_code = VACUUM_ENCODER_ERROR;
					current_data = buf[6];
					return ;
				}
				else if (buf[6] == 2) {
					error_code = VACUUM_ENCODER_FAIL;
					current_data =  buf[6];
					return ;
				}
				else {
					test_result |= 0x0004;
				}
				break;
		}
		if((test_result&0x0007) == 0x0007)
		{
			test_stage++;
			return ;
		}
		serial.sendData();
	}
}
void main_brush_test(uint16_t *baseline, uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data)
{
	uint8_t test_result=0;
	uint32_t current_current=0;
	uint32_t motor_current=0;
	uint8_t buf[REC_LEN];
	uint8_t count=0;
	uint8_t step=1;

	serial.setSendData(CTL_WORK_MODE, FUNC_MAINBRUSH_TEST_MODE);
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
		if (buf[38] != FUNC_MAINBRUSH_TEST_MODE)
		{
			serial.sendData();
			continue;
		}
		switch (step) {
			case 1:
				brush.setPWM(0, 0, 80);
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
				if (count <= 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
					motor_current += static_cast<uint16_t>(buf[4] << 8 | buf[5]);
				}
				else {
					count = 0;
					current_current = (current_current / 10 * 330 * 20 / 4096) - baseline[SYSTEM_CURRENT];
					motor_current = (motor_current / 10 - baseline[MAIN_BRUSH]) * 330 * 10 / 4096;
					step++;
				}
				break;
			case 3:
				step++;
				if (current_current < 130 || current_current > 350 || motor_current < 100 || motor_current > 300) {
					error_code = MAIN_BRUSH_CURRENT_ERROR;
					current_data = static_cast<uint16_t>(motor_current);
					return ;
				}
				else {
					test_result |= 0x01;
				}
				serial.setSendData(CTL_MAIN_BRUSH_TEST_MODE, 1);
				break;
			case 4:
				brush.setPWM(0,0,0);
				count++;
				if(count > 25)
				{
					count = 0;
					step++;
				}
				break;
			case 5:
				brush.setPWM(0, 0, 80);
				count++;
				if (count > 20) {
					current_current = 0;
					motor_current = 0;
					step++;
					count = 0;
				}
				break;
			case 6:
				if (static_cast<uint16_t>(buf[4] << 8 | buf[5]) > baseline[MAIN_BRUSH] + 1150)
					count++;
				else
					count = 0;
				if(count > 2)
				{
					count = 0;
					step++;
				}
				if(buf[36])
				{
					error_code = MAIN_BRUSH_STALL_ERROR;
					current_data = 0;
					return ;
				}
				break;
			case 7:
				count++;
				if (count <= 10) {
					current_current += (static_cast<uint16_t>(buf[8] << 8 | buf[9]) - baseline[REF_VOLTAGE_ADC]);
				}
				else {
					step++;
					count = 0;
					current_current = (current_current / 10 * 330 * 20 / 4096) - baseline[SYSTEM_CURRENT];
					if (current_current < 550)
					{
						error_code = MAIN_BRUSH_STALL_ERROR;
						current_data = 0;
						return ;
					}
					else
					{
						test_result |= 0x02;
					}
				}
				break;
		}
		if((test_result & 0x03) == 0x03)
		{
			test_stage++;
			return ;
		}
		serial.sendData();
	}
}
void charge_current_test(bool is_fixture, uint8_t &test_stage, uint16_t &error_code, uint16_t &current_data)
{
	uint32_t charge_voltage = 0;
	uint8_t buf[REC_LEN];
	serial.setSendData(CTL_WORK_MODE, FUNC_CHARGE_CURRENT_TEST_MODE);
	serial.setSendData(CTL_CHARGER_CINNECTED_STATUS, 0);
	serial.setSendData(CTL_IS_FIXTURE, is_fixture);
	uint8_t count = 0;
	ROS_INFO("%s, %d", __FUNCTION__, __LINE__);
	while(ros::ok())
	{
		/*--------data extrict from serial com--------*/
		ROS_ERROR_COND(pthread_mutex_lock(&recev_lock) != 0, "robotbase pthread receive lock fail");
		ROS_ERROR_COND(pthread_cond_wait(&recev_cond, &recev_lock) != 0, "robotbase pthread receive cond wait fail");
		memcpy(buf, serial.receive_stream, sizeof(uint8_t) * REC_LEN);
		ROS_ERROR_COND(pthread_mutex_unlock(&recev_lock) != 0, "robotbase pthread receive unlock fail");
		if(buf[38] != FUNC_CHARGE_CURRENT_TEST_MODE)
		{
			serial.sendData();
			continue;
		}
		if(count < 10)
		{
			count++;
			charge_voltage += static_cast<uint16_t>(buf[2] << 8 | buf[3]);
		}
		else
		{
			count = 0;
			charge_voltage = charge_voltage /10 *330 /4096 *835 /120;
			if(charge_voltage > 1700)
				serial.setSendData(CTL_CHARGER_CINNECTED_STATUS, 1);
			else if(charge_voltage < 1600)
				serial.setSendData(CTL_CHARGER_CINNECTED_STATUS, 0);
			charge_voltage = 0;
		}
		if(buf[4] == 1) {
			error_code = CHARGE_PWM_ERROR;
			current_data = 0;
			return ;
		}
		else if(buf[4] == 2) {
			error_code = CHARGE_CURRENT_ERROR;
			current_data = 0;
			return ;
		}
		else if(buf[4] == 3)
		{
			test_stage++;
			return ;
		}
		serial.sendData();
	}
}
