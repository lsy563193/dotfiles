#ifndef __SPEAKER_H__
#define __SPEAKER_H__

#include <stdint.h>
#include <alsa/asoundlib.h>

typedef enum {
	SPEAKER_WELCOME_ILIFE				= 1,
	SPEAKER_PLEASE_START_CLEANING		= 2,
	SPEAKER_CLEANING_FINISHED			= 3,
	SPEAKER_CLEANING_START				= 4,
	SPEAKER_CLEANING_WALL_FOLLOW		= 5,
	SPEAKER_BACK_TO_CHARGER				= 6,
	SPEAKER_CLEANING_SPOT				= 7,
	SPEAKER_CLEANING_NAVIGATION			= 8,
	SPEAKER_APPOINTMENT_DONE			= 9,
	SPEAKER_VACCUM_MAX					= 10,
	SPEAKER_BATTERY_LOW					= 11,
	SPEAKER_BATTERY_CHARGE				= 12,
	SPEAKER_BATTERY_CHARGE_DONE			= 13,
	SPEAKER_ERROR_RUBBISH_BIN			= 14,
	SPEAKER_CLEAN_RUBBISH_BIN			= 15,
	SPEAKER_ERROR_LEFT_BRUSH			= 16,
	SPEAKER_ERROR_RIGHT_BRUSH			= 17,
	SPEAKER_ERROR_LEFT_WHEEL			= 18,
	SPEAKER_ERROR_RIGHT_WHEEL			= 19,
	SPEAKER_ERROR_MAIN_BRUSH			= 20,
	SPEAKER_ERROR_SUCTION_FAN			= 21,
	SPEAKER_ERROR_BUMPER				= 22,
	SPEAKER_ERROR_CLIFF					= 23,
	SPEAKER_ERROR_MOBILITY_WHEEL		= 24,
	SPEAKER_ERROR_LIFT_UP				= 25,
	SPEAKER_TEST_MODE					= 26,
	SPEAKER_CAMERA_CALIBRATION_MODE		= 27,
	SPEAKER_TEST_MODE_IEC				= 28,
	SPEAKER_CAMERA_CALIBRATION_START	= 29,
	SPEAKER_CAMERA_CALIBRATION_SUCCESS	= 30,
	SPEAKER_CAMERA_CALIBRATION_FAIL		= 31,
	SPEAKER_TEST_SUCCESS				= 32,
	SPEAKER_TEST_FAIL					= 33,
	SPEAKER_WIFI_CONNECTING				= 34,
	SPEAKER_WIFI_CONNECTED				= 35,
	SPEAKER_TEST_LIDAR					= 36,
	SPEAKER_CLEANING_CONTINUE			= 37,
	SPEAKER_SYSTEM_INITIALIZING			= 38,
	SPEAKER_BACK_TO_CHARGER_FAILED		= 39,
	SPEAKER_CLEANING_PAUSE				= 40,
	SPEAKER_CLEAR_ERROR					= 41,
	SPEAKER_CANCEL_APPOINTMENT			= 42,
	SPEAKER_PLAN_CLEANING_START			= 43,
	SPEAKER_CLEANING_STOP				= 44,
	SPEAKER_CHECK_SWITCH				= 45,
	SPEAKER_ROBOT_STUCK					= 46,
	SPEAKER_EXPLORATION_START			= 47,
}SpeakerType;

typedef struct
{
	char			riffType[4];
	unsigned int	riffSize;
	char			waveType[4];
	char			formatType[4];
	unsigned int	formatSize;
	unsigned short	compressionCode;
	unsigned short	numChannels;
	unsigned int	sampleRate;
	unsigned int	bytesPerSecond;
	unsigned short	blockAlign;
	unsigned short	bitsPerSample;
	char			dataType[4];
	unsigned int	dataSize;
} SpeakerHeaderType;

typedef struct
{
	SpeakerType speakerType;
	bool canBeInterrupted;
}SpeakerStatus;

class Speaker {
public:
	Speaker(void);

	void play(SpeakerType action, bool can_be_interrputed = true);

	void play_routine(void);

private:
	bool openPcmDriver(void);

	void closePcmDriver(void);

	void launchMixer(void);

	void adjustVolume(long volume);

	void finishPlaying(void);

	snd_pcm_t *handle_;

	snd_mixer_t *mixer_fd_;

	snd_mixer_elem_t *elem_;

	FILE *fp_;

	char *buffer_;

	SpeakerHeaderType speaker_header_; 

	std::list<SpeakerStatus> speaker_list_;

	bool can_pp_run_;
};

extern Speaker speaker;
#endif
