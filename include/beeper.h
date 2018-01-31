//
// Created by root on 11/20/17.
//

#ifndef PP_BEEP_H
#define PP_BEEP_H

#define VALID						true
#define INVALID						false

class Beeper
{
public:
	void beep(uint8_t Sound_Code, int Sound_Time_Count, int Silence_Time_Count, int Total_Time_Count);

	void beepForCommand(bool valid);

	void processBeep();
private:

	bool update_flag_;
	int sound_loop_count_;
	uint8_t sound_code_;
	int sound_time_count_;
	int temp_sound_time_count_;
	int silence_time_count_;
	int temp_silence_time_count_;
};

extern Beeper beeper;
#endif //PP_BEEP_H
