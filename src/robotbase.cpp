#include "ros/ros.h"
#include "dev.h"
#include <std_msgs/String.h>
#include <pp/x900sensor.h>
#include <tf/transform_broadcaster.h>
#include "robotbase.h"


bool g_pp_shutdown = false;

bool robotbase_thread_stop = false;
bool send_thread_stop = false;
bool recei_thread_stop = false;
bool event_manager_thread_stop = false;
bool event_handle_thread_stop = false;
bool core_thread_stop = false;

pthread_t robotbaseThread_id;
pthread_t receiPortThread_id;
pthread_t sendPortThread_id;
pthread_mutex_t recev_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t  recev_cond = PTHREAD_COND_INITIALIZER;

pthread_mutex_t serial_data_ready_mtx = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t serial_data_ready_cond = PTHREAD_COND_INITIALIZER;

pp::x900sensor	sensor;

// todo: These variables and the function should be moved to Beeper class.
bool robotbase_beep_update_flag = false;
int robotbase_beeper_sound_loop_count = 0;
uint8_t robotbase_sound_code = 0;
int robotbase_beeper_sound_time_count = 0;
int temp_beeper_sound_time_count = -1;
int robotbase_beeper_silence_time_count = 0;
int temp_beeper_silence_time_count = 0;

// For led control.
uint8_t robotbase_led_type = LED_STEADY;
bool robotbase_led_update_flag = false;
uint8_t robotbase_led_color = LED_GREEN;
uint16_t robotbase_led_cnt_for_switch = 0;
uint16_t live_led_cnt_for_switch = 0;

extern boost::mutex odom_mutex;

void debug_received_stream()
{
	ROS_INFO("%s %d: Received stream:", __FUNCTION__, __LINE__);
	for (int i = 0; i < RECEI_LEN; i++)
		printf("%02d ", i);
	printf("\n");

	for (int i = 0; i < RECEI_LEN; i++)
		printf("%02x ", serial.receive_stream[i]);
	printf("\n");
}

void debug_send_stream(uint8_t *buf)
{
	ROS_INFO("%s %d: Send stream:", __FUNCTION__, __LINE__);
	for (int i = 0; i < SEND_LEN; i++)
		printf("%02x ", *(buf + i));
	printf("\n");
}

void robotbase_deinit(void)
{

	bumper.lidarBumperDeinit();
	recei_thread_stop = true;
	led.setMode(LED_STEADY, LED_OFF);
	serial.setSendData(CTL_BEEPER, 0x00);
	gyro.setOff();
	wheel.stop();
	brush.stop();
	vacuum.stop();
	serial.setMainBoardMode(NORMAL_SLEEP_MODE);
	usleep(40000);
	while(ros::ok() && !g_pp_shutdown){
		usleep(2000);
	}
	serial.close();
	pthread_mutex_destroy(&recev_lock);
	pthread_mutex_destroy(&serial_data_ready_mtx);
	
	pthread_cond_destroy(&recev_cond);
	pthread_cond_destroy(&serial_data_ready_cond);
	
}

void robotbase_reset_send_stream(void)
{
	for (int i = 0; i < SEND_LEN; i++) {
		if (i != CTL_LED_GREEN)
			serial.setSendData(i, 0x00);
		else
			serial.setSendData(i, 0x64);
	}
	serial.setSendData(0, 0xaa);
	serial.setSendData(1, 0x55);
	serial.setSendData(SEND_LEN - 2, 0xcc);
	serial.setSendData(SEND_LEN - 1, 0x33);

	serial.setMainBoardMode(IDLE_MODE);
	uint8_t buf[SEND_LEN];
	{
		boost::mutex::scoped_lock lock(g_send_stream_mutex);
		memcpy(buf, serial.send_stream, sizeof(uint8_t) * SEND_LEN);
	}
	uint8_t crc;
	crc = serial.calBufCrc8(buf, SEND_LEN - 3);
	serial.setSendData(SEND_LEN - 3, crc);
}


void process_beep()
{
	// This routine handles the speaker sounding logic
	// If temp_beeper_silence_time_count == 0, it is the end of loop of silence, so decrease the count and set sound in g_send_stream.
	if (temp_beeper_silence_time_count == 0){
		temp_beeper_silence_time_count--;
		temp_beeper_sound_time_count = robotbase_beeper_sound_time_count;
		serial.setSendData(CTL_BEEPER, robotbase_sound_code & 0xFF);
	}
	// If temp_beeper_sound_time_count == 0, it is the end of loop of sound, so decrease the count and set sound in g_send_stream.
	if (temp_beeper_sound_time_count == 0){
		temp_beeper_sound_time_count--;
		temp_beeper_silence_time_count = robotbase_beeper_silence_time_count;
		serial.setSendData(CTL_BEEPER, 0x00);
		// Decreace the speaker sound loop count because when it turns to silence this sound loop will be over when silence end, so we can decreace the sound loop count here.
		// If it is for constant beeper.play, the loop count will be less than 0, it will not decrease either.
		if (robotbase_beeper_sound_loop_count > 0){
			robotbase_beeper_sound_loop_count--;
		}
	}
	// If temp_beeper_silence_time_count == -1, it is in loop of sound, so decrease the count.
	if (temp_beeper_silence_time_count == -1){
		temp_beeper_sound_time_count--;
	}
	// If temp_beeper_sound_time_count == -1, it is in loop of silence, so decrease the count.
	if (temp_beeper_sound_time_count == -1){
		temp_beeper_silence_time_count--;
	}
}

void process_led()
{
	uint16_t led_brightness = 100;
	switch (robotbase_led_type)
	{
		case LED_STEADY:
		{
			robotbase_led_update_flag = false;
			break;
		}
		case LED_FLASH:
		{
			if (live_led_cnt_for_switch > robotbase_led_cnt_for_switch / 2)
				led_brightness = 0;
			break;
		}
		case LED_BREATH:
		{
			if (live_led_cnt_for_switch > robotbase_led_cnt_for_switch / 2)
				led_brightness = led_brightness * (2 * (float)live_led_cnt_for_switch / (float)robotbase_led_cnt_for_switch - 1.0);
			else
				led_brightness = led_brightness * (1.0 - 2 * (float)live_led_cnt_for_switch / (float)robotbase_led_cnt_for_switch);
			break;
		}
	}

	if (live_led_cnt_for_switch++ > robotbase_led_cnt_for_switch)
		live_led_cnt_for_switch = 0;

	switch (robotbase_led_color)
	{
		case LED_GREEN:
		{
			led.set(led_brightness, 0);
			break;
		}
		case LED_ORANGE:
		{
			led.set(led_brightness, led_brightness);
			break;
		}
		case LED_RED:
		{
			led.set(0, led_brightness);
			break;
		}
		case LED_OFF:
		{
			led.set(0, 0);
			break;
		}
	}
	//ROS_INFO("%s %d: live_led_cnt_for_switch: %d, led_brightness: %d.", __FUNCTION__, __LINE__, live_led_cnt_for_switch, led_brightness);
}

void robotbase_reset_odom_pose(void)
{
	// Reset the odom pose to (0, 0)
	boost::mutex::scoped_lock lock(odom_mutex);
	odom.setX(0);
	odom.setY(0);
}


