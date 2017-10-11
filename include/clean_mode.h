//
// Created by lsy563193 on 17-9-27.
//

#ifndef PP_CLEAN_MODE_H
#define PP_CLEAN_MODE_H

enum{
Clean_Mode_Userinterface = 1,
Clean_Mode_Spiral,
Clean_Mode_WallFollow,
Clean_Mode_RandomMode,
Clean_Mode_Charging,
Clean_Mode_GoHome,
Clean_Mode_Sleep,
Clean_Mode_SelfCheck,
Clean_Mode_Test,
Clean_Mode_Zigzag,
Clean_Mode_Remote,
Clean_Mode_Spot,
Clean_Mode_Mobility,
Clean_Mode_Navigation,
Clean_Mode_Exploration
};

bool cm_is_follow_wall();
bool cm_is_navigation();
bool cm_is_exploration();
bool cm_is_spot();
bool cm_is_go_home();
void cm_set(uint8_t mode);
uint8_t cm_get(void);

#endif //PP_CLEAN_MODE_H
