//
// Created by root on 11/17/17.
//

#ifndef PP_REMOTE_H
#define PP_REMOTE_H


class Remote {
public:
	Remote() {
		g_status = 0;
		g_move_flag = 0;
	}

/*----------------------------------------Remote--------------------------------*/
	uint8_t key(uint8_t key) {
		// Debug
		if (g_status > 0) {
			ROS_DEBUG("%s, %d g_status = %x", __FUNCTION__, __LINE__, g_status);
		}
		if (g_status & key) {
			return 1;
		}
		else {
			return 0;
		}
	}

	void set(uint8_t cmd) {
		g_status |= cmd;
	}

	void reset(void) {
		g_status = 0;
	}

	uint8_t get(void) {
		return g_status;
	}

	void reset_move_with(void) {
		g_move_flag = 0;
	}

private:
	uint8_t g_status;
	uint8_t g_move_flag;
};

extern Remote remote;

#endif //PP_REMOTE_H
