//
// Created by root on 11/20/17.
//
#include "charger.h"
#include "serial.h"

Charger charger;
/*-------------------------------Check if at charger stub------------------------------------*/
bool Charger::isOnStub() {
	// 1: On charger stub and charging.
	// 2: On charger stub but not charging.
	if (status_ == 2 || status_ == 1)
		return true;
	else
		return false;
}

bool Charger::isDirected() {
	// 3: Direct connect to charge line but not charging.
	// 4: Direct connect to charge line and charging.
	return status_ == 3 || status_ == 4;
}

void Charger::setStart() {
	// This function will turn on the charging function.
	serial.setSendData(CTL_CHARGER, 0x01);
}

void Charger::setStop() {
	// Set the flag to false so that it can quit charger is_max_clean_state_.
	serial.setSendData(CTL_CHARGER, 0x00);
}

bool Charger::isStop()
{
	return serial.getSendData(CTL_CHARGER) == 0;
}


