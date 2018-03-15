//
// Created by root on 11/17/17.
//

#ifndef PP_REMOTE_H
#define PP_REMOTE_H

#define REMOTE_FORWARD				(1)
#define REMOTE_LEFT					(2)
#define REMOTE_RIGHT				(3)
#define REMOTE_MAX					(4)
#define REMOTE_CLEAN				(5)
#define REMOTE_HOME					(6)
#define REMOTE_WALL_FOLLOW			(7)
#define REMOTE_SPOT					(8)
#define REMOTE_WIFI					(9)

class Remote {
public:
	Remote() {
		key_status_ = 0;
	}

	bool isKeyTrigger(uint8_t key);

	void set(uint8_t cmd) {
		key_status_ = cmd;
	}

	void reset(void) {
		key_status_ = 0;
	}

	uint8_t get(void) {
		return key_status_;
	}

private:
	uint8_t key_status_;
};

extern Remote remote;

#endif //PP_REMOTE_H
