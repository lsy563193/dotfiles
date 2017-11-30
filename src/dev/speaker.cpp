#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <alsa/asoundlib.h>

#include <ros/ros.h>

#include <string>

#include "speaker.h"

using namespace std;

#define PP_PACKAGE_PATH	"/opt/ros/indigo/share/pp/audio/%02d.wav"

void Speaker::play(SpeakerType action)
{
	int	rc, ret, size, dir, channels, frequency, bit, datablock, nread;
	char	audio_file[64];

	unsigned int	val;

	snd_pcm_uframes_t	frames;
	snd_pcm_hw_params_t	*params;

	dir = 0;
	snprintf(audio_file, 38, PP_PACKAGE_PATH, action);
	fp_ = fopen(audio_file, "rb");
	if (fp_ == NULL) {
		ROS_ERROR("open file failed: %s\n", audio_file);
		return;
	}

	ROS_INFO("%s %d: Play the wav %d.", __FUNCTION__, __LINE__, action);
	nread = fread(&speaker_header_, 1, sizeof(speaker_header_), fp_);

	ROS_DEBUG("nread: %d\n", nread);
	ROS_DEBUG("RIFF Type: %s\n", speaker_header_.riffType);
	ROS_DEBUG("File Size: %d\n", speaker_header_.riffSize);
	ROS_DEBUG("Wave Type: %s\n", speaker_header_.waveType);
	ROS_DEBUG("Format Type: %s\n", speaker_header_.formatType);
	ROS_DEBUG("Format Size: %d\n",speaker_header_.formatSize);
	ROS_DEBUG("Compression Code: %d\n", speaker_header_.compressionCode);
	ROS_DEBUG("Channels: %d\n", speaker_header_.numChannels);
	ROS_DEBUG("Sample Rate: %d\n", speaker_header_.sampleRate);
	ROS_DEBUG("Bytes Per Sample: %d\n", speaker_header_.bytesPerSecond);
	ROS_DEBUG("Block Align: %d\n", speaker_header_.blockAlign);
	ROS_DEBUG("Bits Per Sample: %d\n", speaker_header_.bitsPerSample);
	ROS_DEBUG("Data Type: %s\n", speaker_header_.dataType);
	ROS_DEBUG("Data Size: %d\n", speaker_header_.dataSize);

	channels = speaker_header_.numChannels;
	frequency = speaker_header_.sampleRate;
	bit = speaker_header_.bitsPerSample;
	datablock = speaker_header_.blockAlign;

	openPcmDriver();

	snd_pcm_hw_params_alloca(&params);

	rc = snd_pcm_hw_params_any(handle_, params);
	if (rc < 0) {
		ROS_ERROR("snd_pcm_hw_params_any");
		closePcmDriver();
		return;
	}

	rc = snd_pcm_hw_params_set_access(handle_, params, SND_PCM_ACCESS_RW_INTERLEAVED);
	if (rc < 0) {
		ROS_ERROR("sed_pcm_hw_set_access");
		closePcmDriver();
		return;
	}

	switch (bit / 8) {
		case 1:
			rc = snd_pcm_hw_params_set_format(handle_, params, SND_PCM_FORMAT_U8);
			if (rc < 0) {
				ROS_ERROR("%s %d: snd_pcm_hw_params_set_format", __FUNCTION__, __LINE__);
				closePcmDriver();
				return;
			}
			break ;
		case 2:
			rc = snd_pcm_hw_params_set_format(handle_, params, SND_PCM_FORMAT_S16_LE);
			if (rc < 0) {
				ROS_ERROR("%s %d: snd_pcm_hw_params_set_format", __FUNCTION__, __LINE__);
				closePcmDriver();
				return;
			}
			break ;
		case 3:
			rc = snd_pcm_hw_params_set_format(handle_, params, SND_PCM_FORMAT_S24_LE);
			if (rc < 0) {
				ROS_ERROR("%s %d: snd_pcm_hw_params_set_format", __FUNCTION__, __LINE__);
				closePcmDriver();
				return;
			}
			break ;
	}

	rc = snd_pcm_hw_params_set_channels(handle_, params, channels);
	if (rc < 0) {
		ROS_ERROR("snd_pcm_hw_params_set_channels:");
		closePcmDriver();
		return;
	}

	val = frequency;
	rc = snd_pcm_hw_params_set_rate_near(handle_, params, &val, &dir);
	if (rc < 0) {
		ROS_ERROR("snd_pcm_hw_params_set_rate_near:");
		closePcmDriver();
		return;
	}

	rc = snd_pcm_hw_params(handle_, params);
	if (rc < 0) {
		ROS_ERROR("snd_pcm_hw_params: ");
		closePcmDriver();
		return;
	}

	rc = snd_pcm_hw_params_get_period_size(params, &frames, &dir);
	if (rc < 0) {
		ROS_ERROR("snd_pcm_hw_params_get_period_size:");
		closePcmDriver();
		return;
	}

	size = frames * datablock;

	buffer_ = (char*) malloc(size);

	// Seek to audio data
	fseek(fp_, 58, SEEK_SET);

	while (ros::ok()) {
		memset(buffer_, 0, sizeof(char)* size);
		ret = fread(buffer_, 1, size, fp_);
		if (ret == 0) {
			ROS_DEBUG("end of audio file");
			break;
		} else if (ret != size) {
		}

		while ((ret = snd_pcm_writei(handle_, buffer_, ret/datablock)) < 0) {
			usleep(2000);
			if (ret == -EPIPE) {
				ROS_DEBUG("audio underrun occurred");
				snd_pcm_prepare(handle_);
			} else if (ret < 0) {
				ROS_DEBUG("error from writei: %s", snd_strerror(ret));
			}
		}
	}

	closePcmDriver();
	return;
}

bool Speaker::openPcmDriver(void)
{
	int rc = 0;
	launchMixer();
	adjustVolume(0);
	ROS_DEBUG("%s %d: Open wav driver.", __FUNCTION__, __LINE__);
	rc = snd_pcm_open(&handle_, "plug:dmix", SND_PCM_STREAM_PLAYBACK, 0);
	if (rc < 0)	{
		ROS_ERROR("open PCM device failed:");
		return false;
	}
	adjustVolume(55);
	return true;
}

void Speaker::closePcmDriver(void)
{
	ROS_DEBUG("%s %d: Close wav driver.", __FUNCTION__, __LINE__);
	fclose(fp_);
	free(buffer_);
	adjustVolume(0);
	snd_pcm_drain(handle_);
	snd_pcm_close(handle_);
	snd_mixer_close(mixer_fd_);
}

void Speaker::launchMixer(void)
{

	int rc;

	ROS_DEBUG("%s %d: Launch mixer.", __FUNCTION__, __LINE__);
	// Open the mixer.
	rc = snd_mixer_open(&mixer_fd_, 0);
	if (rc < 0)
	{
		ROS_ERROR("%s %d: snd_mixer_open error: %d.", __FUNCTION__, __LINE__, rc);
		mixer_fd_ = NULL;
		return;
	}

	// Attach an HCTL to an opened mixer.
	rc = snd_mixer_attach(mixer_fd_, "default");
	if (rc < 0)
	{
		ROS_ERROR("%s %d: snd_mixer_attach error: %d.", __FUNCTION__, __LINE__, rc);
		snd_mixer_close(mixer_fd_);
		mixer_fd_ = NULL;
		return;
	}

	// Register the mixer.
	rc = snd_mixer_selem_register(mixer_fd_, NULL, NULL);
	if (rc < 0)
	{
		ROS_ERROR("%s %d: snd_mixer_selem_register error: %d.", __FUNCTION__, __LINE__, rc);
		snd_mixer_close(mixer_fd_);
		mixer_fd_ = NULL;
		return;
	}

	// Load the mixer.
	rc = snd_mixer_load(mixer_fd_);
	if (rc < 0)
	{
		ROS_ERROR("%s %d: snd_mixer_selem_register error: %d.", __FUNCTION__, __LINE__, rc);
		snd_mixer_close(mixer_fd_);
		mixer_fd_ = NULL;
		return;
	}
}

void Speaker::adjustVolume(long volume)
{
	long min_volume, max_volume;
	// Adjust the volume.
	for(elem_=snd_mixer_first_elem(mixer_fd_); elem_; elem_=snd_mixer_elem_next(elem_))
	{
		if (snd_mixer_elem_get_type(elem_) == SND_MIXER_ELEM_SIMPLE && snd_mixer_selem_is_active(elem_))// Find the available and active mixer.
		{
			if(!strncmp(snd_mixer_selem_get_name(elem_), "Lineout volume control", 22))
			{
				snd_mixer_selem_get_playback_volume_range(elem_, &min_volume, &max_volume);
				//ROS_WARN("%s %d: Mixer %s volume range (%ld, %ld).", __FUNCTION__, __LINE__, snd_mixer_selem_get_name(elem_), min_volume, max_volume);
				ROS_DEBUG("%s %d: Set volumn as %ld.", __FUNCTION__, __LINE__, volume);
				snd_mixer_selem_set_playback_volume_all(elem_, volume);
				break;
			}
		}
	}
}

Speaker speaker;

