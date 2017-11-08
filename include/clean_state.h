//
// Created by lsy563193 on 17-9-27.
//

#ifndef PP_CLEAN_STATE_H
#define PP_CLEAN_STATE_H

enum{
		CS_CLEAN,
		CS_GO_HOME_POINT,
		CS_GO_CHANGER,
		CS_TMP_SPOT,
		CS_TRAPPED,
		CS_EXPLORATION,
		CS_STOP,
};

bool cs_init();
bool cs_is_going_home();
bool cs_is_go_home_point();
bool cs_is_go_charger();
bool cs_is_exploration();
bool cs_is_clean();
bool cs_is_tmp_spot();
bool cs_is_trapped();
int cs_get(void);
void cs_set(int);

#endif //PP_CLEAN_MODE_H
