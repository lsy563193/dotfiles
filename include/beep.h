//
// Created by root on 11/20/17.
//

#ifndef PP_BEEP_H
#define PP_BEEP_H

#define VALID						true
#define INVALID						false


class Beep {
public:
void play(uint8_t Sound_Code, int Sound_Time_Count, int Silence_Time_Count, int Total_Time_Count);

void play_for_command(bool valid);

};

extern Beep beeper;
#endif //PP_BEEP_H
