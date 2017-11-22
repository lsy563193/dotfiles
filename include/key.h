//
// Created by root on 11/17/17.
//

#ifndef PP_KEY_H
#define PP_KEY_H


class Key {
public:
	Key() {
		status_ = false;
		g_status = 0;
	}

	void reset(void) {
		status_ = false;
	}

	uint8_t get(void) {
		return status_;
	}

	void set() {
		status_ = true;
	}


	void set_press(uint8_t key) {
		g_status |= key;
	}

	void reset_press(uint8_t key) {
		g_status &= ~key;
	}

	uint8_t get_press(void) {
		return g_status;
	}

	void eliminate_jitter(uint8_t key_);

private:
	bool status_;
	volatile uint8_t g_status;

// For avoid key value palse.
	int8_t press_count;
	int8_t release_count;

};


extern Key key;
#endif //PP_KEY_H
