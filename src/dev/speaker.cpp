#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <alsa/asoundlib.h>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>

#include <string>

#include "speaker.h"

using namespace std;

#define PP_PACKAGE_PATH	"/opt/ros/indigo/share/pp/audio/%02d.wav"

boost::mutex speaker_list_mutex;

Speaker::Speaker(void)
{
	can_pp_keep_running_ = true;
}

void Speaker::playRoutine(void)
{
	int	rc, ret, size, dir, channels, frequency, bit, datablock, nread;
	char	audio_file[64];

	unsigned int	val;
	VoiceStruct curr_voice;

	snd_pcm_uframes_t	frames;
	snd_pcm_hw_params_t	*params;

	while(ros::ok())
	{
		if(!can_pp_keep_running_)
		{
			speaker_list_mutex.lock();
			curr_voice.type = list_.front().type;
			curr_voice.can_be_interrupted = list_.front().can_be_interrupted;
			list_.pop_front();
			speaker_list_mutex.unlock();

			/*---pp can go on if the speaker can be interrupt---*/
			if(curr_voice.can_be_interrupted)
				can_pp_keep_running_ = true;
			dir = 0;
			snprintf(audio_file, 38, PP_PACKAGE_PATH, curr_voice.type);
			fp_ = fopen(audio_file, "rb");
			if (fp_ == NULL) {
				ROS_ERROR("open file failed: %s\n", audio_file);
				continue;
			}

			ROS_INFO("%s %d: Play the wav %d.", __FUNCTION__, __LINE__, curr_voice.type);
			nread = fread(&voice_header_, 1, sizeof(voice_header_), fp_);

			ROS_DEBUG("nread: %d\n", nread);
			ROS_DEBUG("RIFF Type: %s\n", voice_header_.riff_type);
			ROS_DEBUG("File Size: %d\n", voice_header_.riff_size);
			ROS_DEBUG("Wave Type: %s\n", voice_header_.wave_type);
			ROS_DEBUG("Format Type: %s\n", voice_header_.format_type);
			ROS_DEBUG("Format Size: %d\n",voice_header_.format_size);
			ROS_DEBUG("Compression Code: %d\n", voice_header_.compression_code);
			ROS_DEBUG("Channels: %d\n", voice_header_.num_channels);
			ROS_DEBUG("Sample Rate: %d\n", voice_header_.sample_rate);
			ROS_DEBUG("Bytes Per Sample: %d\n", voice_header_.bytes_per_second);
			ROS_DEBUG("Block Align: %d\n", voice_header_.block_align);
			ROS_DEBUG("Bits Per Sample: %d\n", voice_header_.bits_per_sample);
			ROS_DEBUG("Data Type: %s\n", voice_header_.data_type);
			ROS_DEBUG("Data Size: %d\n", voice_header_.data_size);

			channels = voice_header_.num_channels;
			frequency = voice_header_.sample_rate;
			bit = voice_header_.bits_per_sample;
			datablock = voice_header_.block_align;

			openPcmDriver();

			snd_pcm_hw_params_alloca(&params);

			rc = snd_pcm_hw_params_any(handle_, params);
			if (rc < 0) {
				ROS_ERROR("snd_pcm_hw_params_any");
				finishPlaying();
				continue;
			}

			rc = snd_pcm_hw_params_set_access(handle_, params, SND_PCM_ACCESS_RW_INTERLEAVED);
			if (rc < 0) {
				ROS_ERROR("sed_pcm_hw_set_access");
				finishPlaying();
				continue;
			}

			switch (bit / 8) {
				case 1:
					rc = snd_pcm_hw_params_set_format(handle_, params, SND_PCM_FORMAT_U8);
					if (rc < 0) {
						ROS_ERROR("%s %d: snd_pcm_hw_params_set_format", __FUNCTION__, __LINE__);
						finishPlaying();
						continue;
					}
					break ;
				case 2:
					rc = snd_pcm_hw_params_set_format(handle_, params, SND_PCM_FORMAT_S16_LE);
					if (rc < 0) {
						ROS_ERROR("%s %d: snd_pcm_hw_params_set_format", __FUNCTION__, __LINE__);
						finishPlaying();
						continue;
					}
					break ;
				case 3:
					rc = snd_pcm_hw_params_set_format(handle_, params, SND_PCM_FORMAT_S24_LE);
					if (rc < 0) {
						ROS_ERROR("%s %d: snd_pcm_hw_params_set_format", __FUNCTION__, __LINE__);
						finishPlaying();
						continue;
					}
					break ;
			}

			rc = snd_pcm_hw_params_set_channels(handle_, params, channels);
			if (rc < 0) {
				ROS_ERROR("snd_pcm_hw_params_set_channels:");
				finishPlaying();
				continue;
			}

			val = frequency;
			rc = snd_pcm_hw_params_set_rate_near(handle_, params, &val, &dir);
			if (rc < 0) {
				ROS_ERROR("snd_pcm_hw_params_set_rate_near:");
				finishPlaying();
				continue;
			}

			rc = snd_pcm_hw_params(handle_, params);
			if (rc < 0) {
				ROS_ERROR("snd_pcm_hw_params: ");
				finishPlaying();
				continue;
			}

			rc = snd_pcm_hw_params_get_period_size(params, &frames, &dir);
			if (rc < 0) {
				ROS_ERROR("snd_pcm_hw_params_get_period_size:");
				finishPlaying();
				continue;
			}

			size = frames * datablock;

			buffer_ = (char*) malloc(size);

			// Seek to audio data
			fseek(fp_, 58, SEEK_SET);

			while (ros::ok()) {
				/*---new speaker type---*/
				if(!can_pp_keep_running_ && curr_voice.can_be_interrupted)
					break;
				memset(buffer_, 0, sizeof(char)* size);
				ret = fread(buffer_, 1, size, fp_);
				if (ret == 0) {
					ROS_DEBUG("end of audio file");
					break;
				} else if (ret != size) {
				}

				while ((ret = snd_pcm_writei(handle_, buffer_, ret/datablock)) < 0) {
					/*---new speaker type---*/
					if(!can_pp_keep_running_ && curr_voice.can_be_interrupted)
						break;
					usleep(2000);
				  	if (ret == -EPIPE) {
						ROS_DEBUG("audio underrun occurred");
						snd_pcm_prepare(handle_);
					} else if (ret < 0) {
						ROS_DEBUG("error from writei: %s", snd_strerror(ret));
					}
				}
			}

			if(curr_voice.can_be_interrupted)
			{
				/*---can_pp_keep_running_ is already set if speaker can be interrupted, so close pcm driver only---*/
				closePcmDriver();
			}
			else
			{
				/*---speaker can not be interrupted means playing is already finished now, close pcm driver and let pp run---*/
				finishPlaying();
			}
		}
		else
			usleep(1000);
	}
	return;
}

void Speaker::play(VoiceType wav, bool can_be_interrupted)
{
	ROS_INFO("%s %d: Ask play_routine to play wav:%d.", __FUNCTION__, __LINE__, wav);
	VoiceStruct temp_voice;

	temp_voice.type = wav;
	temp_voice.can_be_interrupted = can_be_interrupted;
	speaker_list_mutex.lock();
	list_.push_back(temp_voice);
	speaker_list_mutex.unlock();
	can_pp_keep_running_ = false;
	while(!can_pp_keep_running_)
		usleep(1000);
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

void Speaker::finishPlaying(void)
{
	closePcmDriver();
	can_pp_keep_running_ = true;
}

Speaker speaker;

