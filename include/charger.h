//
// Created by root on 11/20/17.
//

#ifndef PP_CHARGER_H
#define PP_CHARGER_H

#include <cstdint>

class Charger {
public:
	bool isOnStub();

	bool isDirected();

	bool isStop();

	void setStart();

	void setStop();

	void setChargeStatus(uint8_t val)
	{
		status_ = val;
	}
	uint8_t getChargeStatus() const
	{
		return status_;
	}

	bool enterNavFromChargeMode()
	{
		return enter_nav_from_charge_mode_;
	}

	void enterNavFromChargeMode(bool val)
	{
		enter_nav_from_charge_mode_ = val;
	}

private:
	/*1 byte */
	//0: Not on charger stub.
	//1: On charger stub and charging.
	//2: On charger stub but not charging.
	//3: Direct connect to charge line but not charging.
	//4: Direct connect to charge line and charging.
	uint8_t status_;

	bool enter_nav_from_charge_mode_{false};
};

extern Charger charger;

#endif //PP_CHARGER_H

