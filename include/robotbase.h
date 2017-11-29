#ifndef _ROBOTBASE_H_
#define _ROBOTBASE_H_

#include <sys/types.h>
#include <ros/ros.h>

#include "config.h"

extern bool g_is_tilt;

extern bool robotbase_beep_update_flag;
extern int robotbase_speaker_sound_loop_count;
extern uint8_t robotbase_sound_code;
extern int robotbase_speaker_sound_time_count;
extern int robotbase_speaker_silence_time_count;
// For led control.
extern uint8_t robotbase_led_type;
extern bool robotbase_led_update_flag;
extern uint8_t robotbase_led_color;
extern uint16_t robotbase_led_cnt_for_switch;
extern uint16_t live_led_cnt_for_switch;

extern bool key_or_clean_button_detected;
int robotbase_init();
void robotbase_deinit(void);
void robotbase_reset_send_stream(void);
bool is_robotbase_stop(void);
void *serial_receive_routine(void*);
void *robotbase_routine(void*);
void *serial_send_routine(void*);
void process_beep();
void process_led();
void robotbase_reset_odom_pose(void);
void robotbase_restore_slam_correction(void);
#endif
