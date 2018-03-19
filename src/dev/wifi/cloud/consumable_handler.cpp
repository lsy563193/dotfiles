#include <chrono>
#include "preference.h"
#include "cloud/consumable_handler.h"

using namespace std;

namespace cloud
{

ConsumableHandler::ConsumableHandler()
		: p_s_robot_(Robot::instance())
{}

ConsumableHandler::~ConsumableHandler()
{}

void ConsumableHandler::onStart( void )
{
	stopwatch_.start();
}

void ConsumableHandler::onStop( void )
{
	stopwatch_.stop();
	const auto work_min = stopwatch_.get<chrono::minutes>();
	Preference &pref = Preference::inst();
	pref.setTotalSideBrushMin( pref.getTotalSideBrushMin( 0 ) + work_min );
	pref.setTotalMainBrushMin( pref.getTotalMainBrushMin( 0 ) + work_min );
	if ( p_s_robot_->isWaterTank() )
	{
		pref.setTotalClothMin( pref.getTotalClothMin( 0 ) + work_min );
	}
	else
	{
		pref.setTotalFilterMin( pref.getTotalFilterMin( 0 ) + work_min );
	}
	pref.setTotalBatteryMin( pref.getTotalBatteryMin( 0 ) + work_min );
	pref.commit();
}

}
