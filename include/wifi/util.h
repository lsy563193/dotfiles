#pragma once

//#include "robot.h"
#include "wifi/msg.h"

namespace wifi
{

class Util
{
public:
	static WorkMode to_work_mode( const ModeType m );

	/**
	 * Convert battery voltage level to percentage value
	 *
	 * @param volt
	 * @return
	 */
	static int to_battery_percentage( const int volt );
};

}
