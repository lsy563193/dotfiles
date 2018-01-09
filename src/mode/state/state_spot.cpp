//
// Created by lsy563193 on 12/4/17.
//


#include "pp.h"
#include "arch.hpp"

void StateSpot::init() {
    vacuum.setTmpMode(Vac_Max);
    brush.fullOperate();
    speaker.play(VOICE_CLEANING_SPOT,false);
}
