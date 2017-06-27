#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <alsa/asoundlib.h>

#include <ros/ros.h>

#include <string>

#include "wav.h"

using namespace std;

#define PP_PACKAGE_PATH	"/opt/ros/indigo/share/pp/audio/%02d.wav"

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

snd_pcm_t *handle;
snd_mixer_t *mixer_fd;
snd_mixer_elem_t *elem;

void wav_play(WavType action)
{
	int	rc, ret, size, dir, channels, frequency, bit, datablock, nread;
	char	*buffer, audio_file[64];
	FILE	*fp;

	unsigned int	val;

	snd_pcm_uframes_t	frames;
	snd_pcm_hw_params_t	*params;

	dir = 0;
	snprintf(audio_file, 38, PP_PACKAGE_PATH, action);
	fp = fopen(audio_file, "rb");
	if (fp == NULL) {
		ROS_ERROR("open file failed: %s\n", audio_file);
		return;
	}

	ROS_WARN("%s %d: Play the wav %d.", __FUNCTION__, __LINE__, action);
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

	wav_open_pcm_driver();

	snd_pcm_hw_params_alloca(&params);

	rc = snd_pcm_hw_params_any(handle, params);
	if (rc < 0) {
		ROS_ERROR("snd_pcm_hw_params_any");
		wav_close_pcm_driver();
		return;
	}

	rc = snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
	if (rc < 0) {
		ROS_ERROR("sed_pcm_hw_set_access");
		wav_close_pcm_driver();
		return;
	}

	switch (bit / 8) {
		case 1:
			rc = snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_U8);
			if (rc < 0) {
				ROS_ERROR("%s %d: snd_pcm_hw_params_set_format", __FUNCTION__, __LINE__);
				wav_close_pcm_driver();
				return;
			}
			break ;
		case 2:
			rc = snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);
			if (rc < 0) {
				ROS_ERROR("%s %d: snd_pcm_hw_params_set_format", __FUNCTION__, __LINE__);
				wav_close_pcm_driver();
				return;
			}
			break ;
		case 3:
			rc = snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S24_LE);
			if (rc < 0) {
				ROS_ERROR("%s %d: snd_pcm_hw_params_set_format", __FUNCTION__, __LINE__);
				wav_close_pcm_driver();
				return;
			}
			break ;
	}

	rc = snd_pcm_hw_params_set_channels(handle, params, channels);
	if (rc < 0) {
		ROS_ERROR("snd_pcm_hw_params_set_channels:");
		wav_close_pcm_driver();
		return;
	}

	val = frequency;
	rc = snd_pcm_hw_params_set_rate_near(handle, params, &val, &dir);
	if (rc < 0) {
		ROS_ERROR("snd_pcm_hw_params_set_rate_near:");
		wav_close_pcm_driver();
		return;
	}
 
	rc = snd_pcm_hw_params(handle, params);
	if (rc < 0) {
		ROS_ERROR("snd_pcm_hw_params: ");
		wav_close_pcm_driver();
		return;
	}

	rc = snd_pcm_hw_params_get_period_size(params, &frames, &dir);
	if (rc < 0) {
		ROS_ERROR("snd_pcm_hw_params_get_period_size:");
		wav_close_pcm_driver();
		return;
	}

	size = frames * datablock;

	buffer = (char*) malloc(size);

	// Seek to audio data
	fseek(fp, 58, SEEK_SET);

	while (ros::ok()) {
		memset(buffer, 0, sizeof(char)* size);
		ret = fread(buffer, 1, size, fp);
		if (ret == 0) {
			ROS_DEBUG("end of audio file");
			break;
		} else if (ret != size) {
		}

		while ((ret = snd_pcm_writei(handle, buffer, ret/datablock)) < 0) {
			usleep(2000);
			if (ret == -EPIPE) {
				ROS_DEBUG("audio underrun occurred");
				snd_pcm_prepare(handle);
			} else if (ret < 0) {
				ROS_DEBUG("error from writei: %s", snd_strerror(ret));
			}
		}
	}

	fclose(fp);

	wav_close_pcm_driver();
	free(buffer);
	return;
}

bool wav_open_pcm_driver(void)
{
	int rc = 0;
	wav_launch_mixer();
	wav_adjust_volume(0);
	ROS_DEBUG("%s %d: Open wav driver.", __FUNCTION__, __LINE__);
	rc = snd_pcm_open(&handle, "plug:dmix", SND_PCM_STREAM_PLAYBACK, 0);
	if (rc < 0)	{
		ROS_ERROR("open PCM device failed:");
		return false;
	}
	wav_adjust_volume(55);
	return true;
}

void wav_close_pcm_driver(void)
{
	ROS_DEBUG("%s %d: Close wav driver.", __FUNCTION__, __LINE__);
	wav_adjust_volume(0);
	snd_pcm_drain(handle);
	snd_pcm_close(handle);
	snd_mixer_close(mixer_fd);
}

void wav_launch_mixer(void)
{

	int rc;

	ROS_DEBUG("%s %d: Launch mixer.", __FUNCTION__, __LINE__);
	// Open the mixer.
	rc = snd_mixer_open(&mixer_fd, 0);
	if (rc < 0)
	{
		ROS_ERROR("%s %d: snd_mixer_open error: %d.", __FUNCTION__, __LINE__, rc);
		mixer_fd = NULL;
		return;
	}

	// Attach an HCTL to an opened mixer.
	rc = snd_mixer_attach(mixer_fd, "default");
	if (rc < 0)
	{
		ROS_ERROR("%s %d: snd_mixer_attach error: %d.", __FUNCTION__, __LINE__, rc);
		snd_mixer_close(mixer_fd);
		mixer_fd = NULL;
		return;
	}

	// Register the mixer.
	rc = snd_mixer_selem_register(mixer_fd, NULL, NULL);
	if (rc < 0)
	{
		ROS_ERROR("%s %d: snd_mixer_selem_register error: %d.", __FUNCTION__, __LINE__, rc);
		snd_mixer_close(mixer_fd);
		mixer_fd = NULL;
		return;
	}

	// Load the mixer.
	rc = snd_mixer_load(mixer_fd);
	if (rc < 0)
	{
		ROS_ERROR("%s %d: snd_mixer_selem_register error: %d.", __FUNCTION__, __LINE__, rc);
		snd_mixer_close(mixer_fd);
		mixer_fd = NULL;
		return;
	}
}

void wav_adjust_volume(long volume)
{
	long min_volume, max_volume;
	// Adjust the volume.
	for(elem=snd_mixer_first_elem(mixer_fd); elem; elem=snd_mixer_elem_next(elem))
	{
		if (snd_mixer_elem_get_type(elem) == SND_MIXER_ELEM_SIMPLE && snd_mixer_selem_is_active(elem))// Find the available and active mixer.
		{
			if(!strncmp(snd_mixer_selem_get_name(elem), "Lineout volume control", 22))
			{
				snd_mixer_selem_get_playback_volume_range(elem, &min_volume, &max_volume);
				//ROS_WARN("%s %d: Mixer %s volume range (%ld, %ld).", __FUNCTION__, __LINE__, snd_mixer_selem_get_name(elem), min_volume, max_volume);
				ROS_DEBUG("%s %d: Set volumn as %ld.", __FUNCTION__, __LINE__, volume);
				snd_mixer_selem_set_playback_volume_all(elem, volume);
				break;
			}
		}
	}
}
