#include <chrono>
#include "preference.h"
#include "robot.h"
#include "stopwatch.h"
#include "wifi/msg.h"
#include "wifi/tx.h"
#include "cloud/accumulated_work_time_handler.h"

using namespace std;

namespace cloud
{

AccumulatedWorkTimeHandler::AccumulatedWorkTimeHandler()
		: p_s_robot_(Robot::instance())
{}

AccumulatedWorkTimeHandler::~AccumulatedWorkTimeHandler()
{}

void AccumulatedWorkTimeHandler::onStart( void )
{
	stopwatch_.start();
}

void AccumulatedWorkTimeHandler::onStop( void )
{
	stopwatch_.stop();
	const auto work_min = stopwatch_.get<chrono::minutes>();
	Preference &pref = Preference::inst();
	pref.setTotalWorkMin( pref.getTotalWorkMin( 0 ) + work_min );
	pref.setTotalWaterTankWorkMin( pref.getTotalWaterTankWorkMin( 0 ) + work_min );
	pref.setTotalDustBoxWorkMin( pref.getTotalDustBoxWorkMin( 0 ) + work_min );
	pref.commit();

	wifi::AccumulatedWorkTimeUploadTxMsg msg(
			pref.getTotalWorkMin( 0 ) / 60,
			pref.getTotalWaterTankWorkMin( 0 ) / 60,
			pref.getTotalDustBoxWorkMin( 0 ) / 60,
			0,
			p_s_robot_->wifiTxManager().nextSeqNum() );
	p_s_robot_->wifiTxManager().push( std::move( msg ) );
}

}
