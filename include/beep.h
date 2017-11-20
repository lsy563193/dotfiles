//
// Created by root on 11/20/17.
//

#ifndef PP_BEEP_H
#define PP_BEEP_H


class beep {

};

void beep(uint8_t Sound_Code, int Sound_Time_Count, int Silence_Time_Count, int Total_Time_Count);

void beep_for_command(bool valid);


#endif //PP_BEEP_H
