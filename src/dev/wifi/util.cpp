#include <algorithm>
#include <ros/ros.h>
//#include "robot.hpp"
#include "wifi/msg.h"
#include "wifi/util.h"

namespace
{

constexpr int BATTERY_100_VOLT = 164;
constexpr int BATTERY_1_VOLT = 135;

}

namespace wifi
{

WorkMode Util::to_work_mode( const ModeType a_m )
{
	switch ( a_m )
	{
	case M_INTERFACE:
		return WorkMode::IDLE;

	case M_NAVIGATION:
		return WorkMode::PLAN1;

	case M_CHARGING:
		return WorkMode::CHARGE;

	case M_BACKCHARGER:
	case M_EXPLORATION:
	case M_GOHOME:
		return WorkMode::HOMING;

	case M_SLEEP:
		return WorkMode::SLEEP;

	case M_REMOTE:
		return WorkMode::REMOTE;

	case M_WALLFOLLOW:
		return WorkMode::WALL_FOLLOW;

	case M_SPOT:
		return WorkMode::SPOT;

	default:
		ROS_WARN( "wifi::Util::to_work_mode", "Unknown mode: %d", a_m );
		return WorkMode::IDLE;
	}
}

int Util::to_battery_percentage( const int a_volt )
{
	const int p = ( a_volt - BATTERY_1_VOLT ) * 100 / ( BATTERY_100_VOLT
			- BATTERY_1_VOLT );
	return std::min( std::max( 0, p ), 100 );
}

}
