//
// Created by lsy563193 on 17-9-27.
//

#ifndef PP_CLEAN_STATE_H
#define PP_CLEAN_STATE_H

enum{
	CS_OPEN_GYRO,
	CS_BACK_FROM_CHARGER,
	CS_OPEN_LASER,
	CS_ALIGN,
	CS_OPEN_SLAM,
	CS_CLEAN,
	CS_GO_HOME_POINT,
	CS_GO_CHANGER,
	CS_TMP_SPOT,
	CS_TRAPPED,
	CS_EXPLORATION,
	CS_SELF_CHECK,
	CS_STOP,
};

class CleanState {
public:
	CleanState() {
		cs_ = 0;
	}

public:
	bool init();

	bool is_open_gyro();

	bool is_back_from_charger();

	bool is_open_laser();

	bool is_align();

	bool is_open_slam();

	bool is_going_home();

	bool is_go_home_point();

	bool is_go_charger();

	bool is_exploration();

	bool is_clean();

	bool is_tmp_spot();

	bool is_trapped();

	bool is_self_check();

	int get(void);

	void set(int);

private:
	int cs_;//clean state
};
extern CleanState cs;

#endif //PP_CLEAN_MODE_H
