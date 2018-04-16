//
// Created by root on 11/20/17.
//
#include <cstdint>
#include <speaker.h>
#include "beeper.h"
#include "serial.h"

Beeper beeper;
void Beeper::beep(uint8_t sound_code, int sound_time_ms, int silence_time_ms, int total_time_count)
{
#if DEBUG_ENABLE
	// Sound_Code means the interval of the speaker sounding, higher interval makes lower sound.
	sound_code_ = sound_code;
	// Total_Time_Count means how many loops of speaker sound loop will it sound.
	sound_loop_count_ = total_time_count;
	// A speaker sound loop contains one sound time and one silence time
	// Sound_Time_Count means how many loops of g_send_stream loop will it sound in one speaker sound loop
	sound_time_count_ = sound_time_ms / 20;
	// Silence_Time_Count means how many loops of g_send_stream loop will it be silence in one speaker sound loop, -1 means consistently beeper.play.
	silence_time_count_ = silence_time_ms / 20;
	// Trigger the init flag to start the new beeper.play action
	update_flag_ = true;
#endif
}

void Beeper::beepForCommand(bool valid)
{
	if (valid)
		speaker.play(VOICE_VALID_CMD_UNOFFICIAL);
//		beep(2, 40, 0, 1);
	else
		speaker.play(VOICE_INVALID_CMD_UNOFFICIAL);
//		beep(5, 40, 0, 1);
}

void Beeper::processBeep()
{
	// Force reset the beeper action when beeper() function is called, especially when last beeper action is not over. It can stop last beeper action and directly start the updated beeper.play action.
	if (update_flag_)
	{
		temp_sound_time_count_ = -1;
		temp_silence_time_count_ = 0;
		update_flag_ = false;
	}

	//ROS_INFO("%s %d: tmp_sound_count: %d, tmp_silence_count: %d, sound_loop_count: %d.", __FUNCTION__, __LINE__, temp_sound_time_count_, temp_silence_time_count_, sound_loop_count_);
	// If count > 0, it is processing for different alarm.
	if (sound_loop_count_ == 0)
		return;

	// This routine handles the speaker sounding logic
	// If temp_silence_time_count_ == 0, it is the end of loop of silence, so decrease the count and set sound in g_send_stream.
	if (temp_silence_time_count_ == 0){
		temp_silence_time_count_--;
		temp_sound_time_count_ = sound_time_count_;
		serial.setSendData(CTL_BEEPER, static_cast<uint8_t>(sound_code_ & 0xFF));
	}
	// If temp_sound_time_count_ == 0, it is the end of loop of sound, so decrease the count and set sound in g_send_stream.
	if (temp_sound_time_count_ == 0){
		temp_sound_time_count_--;
		temp_silence_time_count_ = silence_time_count_;
		serial.setSendData(CTL_BEEPER, 0x00);
		// Decreace the speaker sound loop count because when it turns to silence this sound loop will be over when silence end, so we can decreace the sound loop count here.
		// If it is for constant beeper.play, the loop count will be less than 0, it will not decrease either.
		if (sound_loop_count_ > 0){
			sound_loop_count_--;
		}
	}
	// If temp_silence_time_count_ == -1, it is in loop of sound, so decrease the count.
	if (temp_silence_time_count_ == -1){
		temp_sound_time_count_--;
	}
	// If temp_sound_time_count_ == -1, it is in loop of silence, so decrease the count.
	if (temp_sound_time_count_ == -1){
		temp_silence_time_count_--;
	}
}

