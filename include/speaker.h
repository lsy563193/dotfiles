#ifndef __SPEAKER_H__
#define __SPEAKER_H__

#include <stdint.h>
#include <alsa/asoundlib.h>

typedef enum {
	VOICE_NULL						= 0,
	VOICE_WELCOME_ILIFE				= 1,
	VOICE_PLEASE_START_CLEANING		= 2,
	VOICE_CLEANING_FINISHED			= 3,
	VOICE_CLEANING_START				= 4,
	VOICE_CLEANING_WALL_FOLLOW		= 5,
	VOICE_GO_HOME_MODE				= 6,
	VOICE_CLEANING_SPOT				= 7,
	VOICE_CLEANING_NAVIGATION			= 8,
	VOICE_APPOINTMENT_DONE			= 9,
	VOICE_VACCUM_MAX					= 10,
	VOICE_BATTERY_LOW					= 11,
	VOICE_BATTERY_CHARGE				= 12,
	VOICE_BATTERY_CHARGE_DONE			= 13,
	VOICE_ERROR_RUBBISH_BIN			= 14,
	VOICE_CLEAN_RUBBISH_BIN			= 15,
	VOICE_ERROR_LEFT_BRUSH			= 16,
	VOICE_ERROR_RIGHT_BRUSH			= 17,
	VOICE_ERROR_LEFT_WHEEL			= 18,
	VOICE_ERROR_RIGHT_WHEEL			= 19,
	VOICE_ERROR_MAIN_BRUSH			= 20,
	VOICE_ERROR_SUCTION_FAN			= 21,
	VOICE_ERROR_BUMPER				= 22,
	VOICE_ERROR_CLIFF					= 23,
	VOICE_ERROR_MOBILITY_WHEEL		= 24,
	VOICE_ERROR_LIFT_UP				= 25,
	VOICE_TEST_MODE					= 26,
	VOICE_CAMERA_CALIBRATION_MODE		= 27,
	VOICE_TEST_MODE_IEC				= 28,
	VOICE_CAMERA_CALIBRATION_START	= 29,
	VOICE_CAMERA_CALIBRATION_SUCCESS	= 30,
	VOICE_CAMERA_CALIBRATION_FAIL		= 31,
	VOICE_TEST_SUCCESS				= 32,
	VOICE_TEST_FAIL					= 33,
	VOICE_WIFI_CONNECTING				= 34,
	VOICE_WIFI_CONNECTED				= 35,
	VOICE_TEST_LIDAR					= 36,
	VOICE_CLEANING_CONTINUE			= 37,
	VOICE_SYSTEM_INITIALIZING			= 38,
	VOICE_BACK_TO_CHARGER_FAILED		= 39,
	VOICE_CLEANING_PAUSE				= 40,
	VOICE_CLEAR_ERROR_UNOFFICIAL					= 41,
	VOICE_CANCEL_APPOINTMENT_UNOFFICIAL			= 42,
	VOICE_APPOINTMENT_START_UNOFFICIAL			= 43,
	VOICE_CLEANING_STOP_UNOFFICIAL				= 44,
	VOICE_CHECK_SWITCH				= 45,
	VOICE_ROBOT_TRAPPED					= 46,
	VOICE_EXPLORATION_START_UNOFFICIAL			= 47,
	VOICE_ERROR_LIFT_UP_CANCEL_APPOINTMENT_UNOFFICIAL	=	48,
	VOICE_BATTERY_LOW_CANCEL_APPOINTMENT_UNOFFICIAL	=	49,
	VOICE_ERROR_LIFT_UP_CLEANING_STOP_UNOFFICIAL	=	50,
	VOICE_COPYING_LOG_UNOFFICIAL = 51,
	VOICE_COPYING_LOG_SUCCESS_UNOFFICIAL = 52,
	VOICE_COPYING_LOG_FAIL_UNOFFICIAL = 53,
	VOICE_PROCESS_ERROR_UNOFFICIAL				= 54,
	VOICE_USER_KILL_UNOFFICIAL					= 55,
	VOICE_WIFI_UNCONNECTED_UNOFFICIAL = 56,
	VOICE_WIFI_UNBIND_UNOFFICIAL = 57,
	VOICE_WIFI_SMART_LINK_UNOFFICIAL = 58,
	VOICE_CLOUD_CONNECTED_UNOFFICIAL = 59,
	VOICE_CLOUD_UNCONNECTED_UNOFFICIAL = 60,
	VOICE_SOFTWARE_VERSION_UNOFFICIAL	= 61,
	VOICE_IM_HERE_UNOFFICIAL = 62,
	VOICE_CONVERT_TO_LARGE_SUCTION_UNOFFICIAL= 63,
	VOICE_CONVERT_TO_NORMAL_SUCTION_UNOFFICIAL= 64,
	VOICE_FAILED_TO_FIND_CHARGEER_UNOFFICIAL = 65,
	VOICE_END_TO_FIND_CHARGEER_UNOFFICIAL = 66,
	VOICE_CLEANING_FINISH_BACK_TO_CHARGER = 67,
	VOICE_PLEASE_PULL_OUT_THE_PLUG = 68,
	VOICE_VALID_CMD_UNOFFICIAL = 69,
	VOICE_INVALID_CMD_UNOFFICIAL = 70,
	VOICE_WAKE_UP_UNOFFICIAL = 71,
	VOICE_SLEEP_UNOFFICIAL = 72,
}VoiceType;

typedef struct
{
	char			riff_type[4];
	unsigned int	riff_size;
	char			wave_type[4];
	char			format_type[4];
	unsigned int	format_size;
	unsigned short	compression_code;
	unsigned short	num_channels;
	unsigned int	sample_rate;
	unsigned int	bytes_per_second;
	unsigned short	block_align;
	unsigned short	bits_per_sample;
	char			data_type[4];
	unsigned int	data_size;
}VoiceHeaderType;

typedef struct
{
	VoiceType type;
	bool can_be_interrupted;
}Voice;

class Speaker {
public:
	Speaker(void);

	void play(VoiceType voice_type, bool can_be_interrputed = true);

	void playRoutine();

	void stop();

	bool test();

private:
	bool openVoiceFile(Voice voice);

	bool openPcmDriver(void);

	bool initPcmDriver(void);

	void closePcmDriver(void);

	int launchMixer(void);

	void adjustVolume(long volume);

	void _play(void);

	snd_pcm_t *handle_;

	snd_mixer_t *mixer_fd_;

	snd_mixer_elem_t *elem_;

	FILE *fp_;

	char *buffer_;

	size_t buffer_size_;

	VoiceHeaderType voice_header_;

	Voice curr_voice_{VOICE_NULL, true};

	bool finish_playing_{true};

	bool break_playing_{false};

	bool speak_thread_stop_;
};

extern Speaker speaker;
#endif
