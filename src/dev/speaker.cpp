#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <alsa/asoundlib.h>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <string>

#include "dev.h"
#include "speaker.h"

using namespace std;

#define PP_PACKAGE_PATH	"/opt/ros/indigo/share/pp/audio/%02d.wav"
#define WAV_SKIP_SIZE 25
#define WAV_HEADER_SIZE 58

Speaker speaker;

Speaker::Speaker(void)
{
	speak_thread_stop_ = false;
}

void Speaker::playRoutine()
{
	ROS_INFO("robotbase,\033[32m%s\033[0m,%d is up.",__FUNCTION__,__LINE__);
	while(!speak_thread_stop_)
	{
		if(!finish_playing_)
		{
			if (!openVoiceFile(curr_voice_))
			{
				finish_playing_ = true;
				continue;
			}

			if (!openPcmDriver())
			{
				finish_playing_ = true;
				continue;
			}

			if (!initPcmDriver())
			{
				finish_playing_ = true;
				closePcmDriver();
				continue;
			}

			_play();

			closePcmDriver();
			ROS_INFO("%s %d: Finish playing voice:%d", __FUNCTION__, __LINE__, curr_voice_.type);
			finish_playing_ = true;

			if (break_playing_)
				break_playing_ = false;
		}
		else
			usleep(1000);
	}
	closePcmDriver();
	printf("%s,%d exit\n",__FUNCTION__,__LINE__);
}

void Speaker::play(VoiceType voice_type, bool can_be_interrupted)
{
	if (!finish_playing_ && !curr_voice_.can_be_interrupted)
	{
		ROS_INFO("%s %d: Wait for previous voice finish.", __FUNCTION__, __LINE__);
		while (!finish_playing_)
			usleep(1500);
	}

	ROS_INFO("%s %d: Ask play_routine to play voice:%d.(%sallowed to be interrupted)",
			 __FUNCTION__, __LINE__, voice_type, can_be_interrupted ? "" : "not ");

	if (!finish_playing_)
	{
		break_playing_ = true;
		ROS_INFO("%s %d: Wait for breaking previous voice.", __FUNCTION__, __LINE__);
		while (!finish_playing_)
			usleep(1500);
	}

	if (voice_type == VOICE_NULL)
		// Just for hanging up the thread until previous voice finish.
		return;

	curr_voice_.type = voice_type;
	curr_voice_.can_be_interrupted = can_be_interrupted;

	finish_playing_ = false;
}

bool Speaker::openVoiceFile(Voice voice)
{
	char	audio_file[64];
	snprintf(audio_file, 38, PP_PACKAGE_PATH, voice.type);
	fp_ = fopen(audio_file, "rb");
	if (fp_ == NULL)
	{
		ROS_ERROR("open file failed: %s\n", audio_file);
		return false;
	}

	auto nread = fread(&voice_header_, 1, sizeof(voice_header_), fp_);

	ROS_DEBUG("nread: %d", nread);
	ROS_DEBUG("RIFF Type: %s", voice_header_.riff_type);
	ROS_DEBUG("File Size: %d", voice_header_.riff_size);
	ROS_DEBUG("Wave Type: %s", voice_header_.wave_type);
	ROS_DEBUG("Format Type: %s", voice_header_.format_type);
	ROS_DEBUG("Format Size: %d", voice_header_.format_size);
	ROS_DEBUG("Compression Code: %d", voice_header_.compression_code);
	ROS_DEBUG("Channels: %d", voice_header_.num_channels);
	ROS_DEBUG("Sample Rate: %d", voice_header_.sample_rate);
	ROS_DEBUG("Bytes Per Sample: %d", voice_header_.bytes_per_second);
	ROS_DEBUG("Block Align: %d", voice_header_.block_align);
	ROS_DEBUG("Bits Per Sample: %d", voice_header_.bits_per_sample);
	ROS_DEBUG("Data Type: %s", voice_header_.data_type);
	ROS_DEBUG("Data Size: %d", voice_header_.data_size);

	return true;
}

bool Speaker::openPcmDriver(void)
{
	auto ret = launchMixer();
	if (ret != 0)
	{
		ROS_ERROR("Launch mixer failed:");
		return false;
	}

	adjustVolume(0);
	ROS_DEBUG("%s %d: Open wav driver.", __FUNCTION__, __LINE__);
	if (snd_pcm_open(&handle_, "plug:dmix", SND_PCM_STREAM_PLAYBACK, 0) < 0)
	{
		ROS_ERROR("open PCM device failed:");
		return false;
	}
	adjustVolume(SPEAKER_VOLUME);
	return true;
}

bool Speaker::initPcmDriver()
{
	snd_pcm_hw_params_t	*params;
	snd_pcm_hw_params_alloca(&params);

	if (snd_pcm_hw_params_any(handle_, params) < 0)
	{
		ROS_ERROR("snd_pcm_hw_params_any");
		return false;
	}

	if (snd_pcm_hw_params_set_access(handle_, params, SND_PCM_ACCESS_RW_INTERLEAVED) < 0)
	{
		ROS_ERROR("sed_pcm_hw_set_access");
		return false;
	}

	auto bit = voice_header_.bits_per_sample;
	switch (bit / 8)
	{
		case 1:
			if (snd_pcm_hw_params_set_format(handle_, params, SND_PCM_FORMAT_U8) < 0)
			{
				ROS_ERROR("%s %d: snd_pcm_hw_params_set_format", __FUNCTION__, __LINE__);
				return false;
			}
			break;
		case 2:
			if (snd_pcm_hw_params_set_format(handle_, params, SND_PCM_FORMAT_S16_LE) < 0)
			{
				ROS_ERROR("%s %d: snd_pcm_hw_params_set_format", __FUNCTION__, __LINE__);
				return false;
			}
			break;
		case 3:
			if (snd_pcm_hw_params_set_format(handle_, params, SND_PCM_FORMAT_S24_LE) < 0)
			{
				ROS_ERROR("%s %d: snd_pcm_hw_params_set_format", __FUNCTION__, __LINE__);
				return false;
			}
			break;
	}

	auto channels = voice_header_.num_channels;
	if (snd_pcm_hw_params_set_channels(handle_, params, channels) < 0)
	{
		ROS_ERROR("snd_pcm_hw_params_set_channels:");
		return false;
	}

	auto frequency = voice_header_.sample_rate;
	int dir = 0;
	if (snd_pcm_hw_params_set_rate_near(handle_, params, &frequency, &dir) < 0)
	{
		ROS_ERROR("snd_pcm_hw_params_set_rate_near:");
		return false;
	}

	if (snd_pcm_hw_params(handle_, params) < 0)
	{
		ROS_ERROR("snd_pcm_hw_params: ");
		return false;
	}

	snd_pcm_uframes_t	frames;
	if (snd_pcm_hw_params_get_period_size(params, &frames, &dir) < 0)
	{
		ROS_ERROR("snd_pcm_hw_params_get_period_size:");
		return false;
	}

	auto datablock = voice_header_.block_align;
	buffer_size_ = frames * datablock;

	buffer_ = (char *) malloc(buffer_size_);

	// Seek to audio data
	fseek(fp_, WAV_SKIP_SIZE * datablock  + WAV_HEADER_SIZE, SEEK_SET);

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

int Speaker::launchMixer(void)
{
	int rc = 0;

	ROS_DEBUG("%s %d: Launch mixer.", __FUNCTION__, __LINE__);
	// Open the mixer.
	rc = snd_mixer_open(&mixer_fd_, 0);
	if (rc < 0)
	{
		ROS_ERROR("%s %d: snd_mixer_open error: %d.", __FUNCTION__, __LINE__, rc);
		mixer_fd_ = NULL;
		return rc;
	}

	// Attach an HCTL to an opened mixer.
	rc = snd_mixer_attach(mixer_fd_, "default");
	if (rc < 0)
	{
		ROS_ERROR("%s %d: snd_mixer_attach error: %d.", __FUNCTION__, __LINE__, rc);
		snd_mixer_close(mixer_fd_);
		mixer_fd_ = NULL;
		return rc;
	}

	// Register the mixer.
	rc = snd_mixer_selem_register(mixer_fd_, NULL, NULL);
	if (rc < 0)
	{
		ROS_ERROR("%s %d: snd_mixer_selem_register error: %d.", __FUNCTION__, __LINE__, rc);
		snd_mixer_close(mixer_fd_);
		mixer_fd_ = NULL;
		return rc;
	}

	// Load the mixer.
	rc = snd_mixer_load(mixer_fd_);
	if (rc < 0)
	{
		ROS_ERROR("%s %d: snd_mixer_selem_register error: %d.", __FUNCTION__, __LINE__, rc);
		snd_mixer_close(mixer_fd_);
		mixer_fd_ = NULL;
	}

	return rc;
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

void Speaker::_play(void)
{
	while (ros::ok())
	{
		if (break_playing_)
			break;
		memset(buffer_, 0, sizeof(char) * buffer_size_);
		auto ret = fread(buffer_, 1, buffer_size_, fp_);
		if (ret == 0)
		{
			ROS_DEBUG("end of audio file");
			break;
		}
//				else if (ret != buffer_size_) {
//
//				}

		snd_pcm_sframes_t ret_vel;
		while (ros::ok())
		{
			if (break_playing_)
				break;

			ret_vel = snd_pcm_writei(handle_, buffer_,
										 static_cast<snd_pcm_uframes_t>(ret / voice_header_.block_align));
			if (ret_vel >= 0)
				break;
			usleep(2000);
			if (ret_vel == -EPIPE)
			{
				ROS_ERROR("audio underrun occurred");
				snd_pcm_prepare(handle_);
			} else
				ROS_ERROR("error from writei: %s", snd_strerror(ret_vel));
		}
	}

}

void Speaker::stop()
{
	break_playing_ = true;
	speak_thread_stop_ = true;
}

bool Speaker::test()
{
	speaker.play(VOICE_TEST_MODE, false);
#if DEBUG_ENABLE
	speaker.play(VOICE_SOFTWARE_VERSION_UNOFFICIAL, false);
#endif
}
