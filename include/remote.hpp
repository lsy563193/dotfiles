//
// Created by root on 11/17/17.
//

#ifndef PP_REMOTE_H
#define PP_REMOTE_H

#define REMOTE_ALL					((uint8_t) 0xFF)
#define REMOTE_FORWARD				((uint8_t) 0x80)
#define REMOTE_LEFT					((uint8_t) 0x40)
#define REMOTE_RIGHT				((uint8_t) 0x20)
#define REMOTE_BACKWARD				((uint8_t) 0x10)
#define REMOTE_MAX					((uint8_t) 0x10)
#define REMOTE_CLEAN				((uint8_t) 0x08)
#define REMOTE_HOME					((uint8_t) 0x04)
#define REMOTE_WALL_FOLLOW			((uint8_t) 0x02)
#define REMOTE_SPOT					((uint8_t) 0x01)

class Remote {
public:
	Remote() {
		key_status_ = 0;
	}

	bool isKeyTrigger(uint8_t key);

	void set(uint8_t cmd) {
		key_status_ |= cmd;
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
