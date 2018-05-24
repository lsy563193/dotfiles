//
// Created by root on 11/17/17.
//

#ifndef PP_OBS_H
#define PP_OBS_H

#include <cstdint>

class Obs {
public:
	Obs();

//--------------------------------------Obs Dynamic adjust----------------------
	void DynamicAdjust(uint16_t count);

	int16_t getFrontTrigValue(void) {
		return front_trig_value_;
	}

	int16_t getLeftTrigValue(void) {
		return left_trig_value_;
	}

	int16_t getRightTrigValue(void) {
		return right_trig_value_;
	}

	int16_t getFrontBaseline(void) {
		return front_baseline_;
	}

	int16_t getLeftBaseline(void) {
		return left_baseline_;
	}

	int16_t getRightBaseline(void) {
		return right_baseline_;
	}

	uint8_t getStatus(int16_t left_offset = 0, int16_t front_offset = 0, int16_t right_offset = 0);

	int16_t getFront(void) {
		return front_value_;
	}

	void setFront(int16_t value);

	int16_t getLeft(void) {
		return left_value_;
	}

	void setLeft(int16_t value);

	int16_t getRight(void) {
		return right_value_;
	}

	void setRight(int16_t value);

	bool frontTriggered(void);

	void control(bool _switch);
private:

	int16_t left_value_;
	int16_t front_value_;
	int16_t right_value_;
	int16_t left_trig_value_;
	int16_t front_trig_value_;
	int16_t right_trig_value_;
	int16_t left_baseline_;
	int16_t front_baseline_;
	int16_t right_baseline_;
	bool switch_{true};

	typedef int16_t(Obs::*Pfunc)(void);
	typedef int16_t(Obs::*Pdata);
};

extern Obs obs;

#endif //PP_OBS_H
