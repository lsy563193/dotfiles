#ifndef _ROBOTBASE_H_
#define _ROBOTBASE_H_

#include <sys/types.h>
#include <ros/ros.h>
#include <pp/slam_angle_offset.h>
#include "config.h"

#if __ROBOT_X900
#define RECEI_LEN 55
extern uint8_t receiStream[RECEI_LEN];
#define SEND_LEN 20
#elif __ROBOT_X400
#define RECEI_LEN	50
extern uint8_t receiStream[RECEI_LEN];
#define SEND_LEN 19
#endif



extern uint8_t sendStream[SEND_LEN] ;
extern bool robotbase_beep_update_flag;
extern int robotbase_speaker_sound_loop_count;
extern uint8_t robotbase_sound_code;
extern int robotbase_speaker_sound_time_count;
extern int robotbase_speaker_silence_time_count;
extern bool key_or_clean_button_detected;
int robotbase_init();
void robotbase_deinit(void);
bool is_robotbase_stop(void);
void *serial_receive_routine(void*);
void *robotbase_routine(void*);
void *serial_send_routine(void*);
void slam_angle_offset_callback(const pp::slam_angle_offset::ConstPtr& msg);
void process_beep();

#endif
