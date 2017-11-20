//
// Created by root on 11/20/17.
//
#include "pp.h"
#include "beep.h"

Beep beeper;
void Beep::play(uint8_t Sound_Code, int Sound_Time_Ms, int Silence_Time_Ms, int Total_Time_Count)
{
	// Sound_Code means the interval of the speaker sounding, higher interval makes lower sound.
	robotbase_sound_code = Sound_Code;
	// Total_Time_Count means how many loops of speaker sound loop will it sound.
	robotbase_speaker_sound_loop_count = Total_Time_Count;
	// A speaker sound loop contains one sound time and one silence time
	// Sound_Time_Count means how many loops of g_send_stream loop will it sound in one speaker sound loop
	robotbase_speaker_sound_time_count = Sound_Time_Ms / 20;
	// Silence_Time_Count means how many loops of g_send_stream loop will it be silence in one speaker sound loop, -1 means consistently beeper.play.
	robotbase_speaker_silence_time_count = Silence_Time_Ms / 20;
	// Trigger the update flag to start the new beeper.play action
	robotbase_beep_update_flag = true;
}

void Beep::play_for_command(bool valid)
{
	if (valid)
		play(2, 40, 0, 1);
	else
		play(5, 40, 0, 1);
}

