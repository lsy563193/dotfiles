//
// Created by root on 11/17/17.
//

#ifndef PP_KEY_H
#define PP_KEY_H


class Key {
public:
	Key() {
		triggered_start_time_stamp_ = time(NULL);
		trigger_status_ = false;
		press_status_ = 0;
	}

	void resetTriggerStatus(void) {
		trigger_status_ = false;
	}

	bool getTriggerStatus(void) {
		return trigger_status_;
	}

	void setTriggerStatus() {
		trigger_status_ = true;
	}


	void setPressStatus() {
		press_status_ = true;
	}

	void resetPressStatus() {
		press_status_ = false;
	}

	bool getPressStatus(void) {
		return press_status_;
	}

	void eliminate_jitter(uint8_t key_triggered);

	double getPressTime();

private:
	double triggered_start_time_stamp_;
	bool trigger_status_;
	bool press_status_;

// For avoid key value palse.
	uint8_t press_count_;
	uint8_t release_count_;

};

extern Key key;
#endif //PP_KEY_H
