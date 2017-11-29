//
// Created by root on 11/22/17.
//

#ifndef PP_DEV_H
#define PP_DEV_H

#if __ROBOT_X900
#define KEY_CLEAN 1
#else
#define KEY_CLEAN 0x02
#endif

#include "serial.h"
#include "controller.h"
#include "battery.h"
#include "beep.h"
#include "bumper.h"
#include "brush.h"
#include "charger.h"
#include "clean_timer.h"
#include "cliff.h"
#include "gyro.h"
#include "key.h"
#include "laser.hpp"
#include "led.h"
#include "obs.h"
#include "omni.h"
#include "rcon.h"
#include "remote.h"
#include "vacuum.h"
#include "wav.h"
#include "wall_follow.h"
#include "wheel.h"

#endif //PP_DEV_H
