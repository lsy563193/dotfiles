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
	void beep(uint8_t sound_code, int sound_time_count, int silence_time_count, int total_time_count);

	void beepForCommand(bool valid);

	void processBeep();

	void debugBeep(bool valid);
private:

	bool update_flag_{false};
	int sound_loop_count_{0};
	uint8_t sound_code_{0};
	int sound_time_count_{0};
	int temp_sound_time_count_{-1};
	int silence_time_count_{0};
	int temp_silence_time_count_{0};
};

extern Beeper beeper;
#endif //PP_BEEP_H
