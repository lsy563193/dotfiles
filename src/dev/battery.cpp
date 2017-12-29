//
// Created by root on 11/17/17.
//

#include "pp.h"
#include "battery.h"

Battery battery;

bool Battery::isFull(void)
{
	return (getVoltage() > BATTERY_FULL_VOLTAGE);
}

bool Battery::isReadyToClean(void)
{
	// Check if battary is lower than the low battery go home voltage value.
	return (getVoltage() >= BATTERY_READY_TO_CLEAN_VOLTAGE);
}

bool Battery::isLow()
{
	return (getVoltage() <= LOW_BATTERY_STOP_VOLTAGE);
}

bool Battery::isReadyToResumeCleaning(void)
{
	// Check if battary is lower than the low battery go home voltage value.
	return (getVoltage() >= RESUME_CLEANING_VOLTAGE);
}

bool Battery::shouldGoHome(void)
{
	// Check if battary is lower than the low battery go home voltage value.
	return (getVoltage() < LOW_BATTERY_GO_HOME_VOLTAGE);
}

