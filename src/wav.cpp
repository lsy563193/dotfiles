#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <alsa/asoundlib.h>

#include <ros/ros.h>

#include <string>

#include "wav.h"

using namespace std;

#define PP_PACKAGE_PATH	"/opt/ros/indigo/share/pp/audio/"

const string audio_files[] = {
				PP_PACKAGE_PATH"startcleaning.wav",
				PP_PACKAGE_PATH"backtocharger.wav",
				PP_PACKAGE_PATH"continuecleaning.wav",
				PP_PACKAGE_PATH"machinehang.wav",
				PP_PACKAGE_PATH"error.wav",
				};

struct WAV_HEADER
{
	char		riffType[4];
	unsigned int	riffSize;
	char		waveType[4];
	char		formatType[4];
	unsigned int	formatSize;
	unsigned short	compressionCode;
	unsigned short	numChannels;
	unsigned int	sampleRate;
	unsigned int	bytesPerSecond;
	unsigned short	blockAlign;
	unsigned short	bitsPerSample;
	char		dataType[4];
	unsigned int	dataSize;
} wav_header;
 
void wav_play(WavType action)
{
	int	rc, ret, size, dir, channels, frequency, bit, datablock, nread;
	char	*buffer;
	FILE	*fp;

	unsigned int	val;

	snd_pcm_t		*handle;

	snd_pcm_uframes_t	frames;
	snd_pcm_hw_params_t	*params;

	dir = 0;
	fp = fopen(audio_files[action].c_str(), "rb");
	if (fp == NULL) {
		ROS_ERROR("open file failed:\n");
		return;
	}

	nread = fread(&wav_header, 1, sizeof(wav_header), fp);

	ROS_DEBUG("nread: %d\n", nread);
	ROS_DEBUG("RIFF Type: %s\n", wav_header.riffType);
	ROS_DEBUG("File Size: %d\n", wav_header.riffSize);
	ROS_DEBUG("Wave Type: %s\n", wav_header.waveType);
	ROS_DEBUG("Format Type: %s\n", wav_header.formatType);
	ROS_DEBUG("Format Size: %d\n",wav_header.formatSize);
	ROS_DEBUG("Compression Code: %d\n", wav_header.compressionCode);
	ROS_DEBUG("Channels: %d\n", wav_header.numChannels);
	ROS_DEBUG("Sample Rate: %d\n", wav_header.sampleRate);
	ROS_DEBUG("Bytes Per Sample: %d\n", wav_header.bytesPerSecond);
	ROS_DEBUG("Block Align: %d\n", wav_header.blockAlign);
	ROS_DEBUG("Bits Per Sample: %d\n", wav_header.bitsPerSample);
	ROS_DEBUG("Data Type: %s\n", wav_header.dataType);
	ROS_DEBUG("Data Size: %d\n", wav_header.dataSize);

	channels = wav_header.numChannels;
	frequency = wav_header.sampleRate;
	bit = wav_header.bitsPerSample;
	datablock = wav_header.blockAlign;

	rc = snd_pcm_open(&handle, "default", SND_PCM_STREAM_PLAYBACK, 0);
	if (rc < 0)	{
		ROS_ERROR("open PCM device failed:");
		return;
	}

	snd_pcm_hw_params_alloca(&params);
	if (rc < 0) {
		ROS_ERROR("snd_pcm_hw_params_alloca");
		return;
	}

	rc = snd_pcm_hw_params_any(handle, params);
	if (rc < 0) {
		ROS_ERROR("snd_pcm_hw_params_any");
		return;
	}

	rc = snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
	if (rc < 0) {
		ROS_ERROR("sed_pcm_hw_set_access");
		return;
	}

	switch (bit / 8) {
		case 1:
			snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_U8);
			break ;
		case 2:
			snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);
			break ;
		case 3:
			snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S24_LE);
			break ;
	}

	rc = snd_pcm_hw_params_set_channels(handle, params, channels);
	if (rc < 0) {
		ROS_ERROR("snd_pcm_hw_params_set_channels:");
		return;
	}

	val = frequency;
	rc = snd_pcm_hw_params_set_rate_near(handle, params, &val, &dir);
	if (rc < 0) {
		ROS_ERROR("snd_pcm_hw_params_set_rate_near:");
		return;
	}
 
	rc = snd_pcm_hw_params(handle, params);
	if (rc < 0) {
		ROS_ERROR("snd_pcm_hw_params: ");
		return;
	}

	rc = snd_pcm_hw_params_get_period_size(params, &frames, &dir);
	if (rc < 0) {
		ROS_ERROR("snd_pcm_hw_params_get_period_size:");
		return;
	}

	size = frames * datablock;

	buffer = (char*) malloc(size);

	// Seek to audio data
	fseek(fp, 58, SEEK_SET);

	while (1) {
		memset(buffer, 0, sizeof(char)* size);
		ret = fread(buffer, 1, size, fp);
		if (ret == 0) {
			ROS_DEBUG("end of audio file");
			break;
		} else if (ret != size) {
		}

		while ((ret = snd_pcm_writei(handle, buffer, frames)) < 0) {
			usleep(2000);
			if (ret == -EPIPE) {
				ROS_DEBUG("audio underrun occurred");
				snd_pcm_prepare(handle);
			} else if (ret < 0) {
				ROS_DEBUG("error from writei: %s", snd_strerror(ret));
			}
		}
	}

	snd_pcm_drain(handle);
	snd_pcm_close(handle);
	free(buffer);
	return;
}
