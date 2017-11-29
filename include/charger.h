//
// Created by root on 11/20/17.
//

#ifndef PP_CHARGER_H
#define PP_CHARGER_H
class Charger {
public:
	bool isOnStub(void);

	bool isDirected(void);

	void setStart(void);

	void setStop(void);

	void setChargeStatus(uint8_t val)
	{
		status_ = val;
	}
	uint8_t getChargeStatus() const
	{
		return status_;
	}

private:
	/*1 byte */
	uint8_t status_;

};

extern Charger charger;

#endif //PP_CHARGER_H

