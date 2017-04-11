#ifndef __WAV_H__
#define __WAV_H__

#include <stdint.h>

typedef enum {
	WAV_START_CLEANING = 0,
	WAV_BACK_TO_CHARGER,
	WAV_CONTINUE_CLEANING,
	WAV_TAKEN_UP,
	WAV_ERROR,
}WavType;

void wav_play(WavType action);

#endif
