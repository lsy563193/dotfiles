#include <cassert>
#include <chrono>
#include <deque>
#include <pthread.h>
#include <sys/time.h>
#include "data_robot.h"
//#include "log.h"
#include <ros.h>
#include "position.h"
#include "robot.h"
#include "stopwatch.h"
#include "wifi/msg.h"
#include "wifi/tx.h"
#include "cloud/realtime_map_handler.h"

using namespace std;

namespace cloud
{

RealtimeMapHandler::RealtimeMapHandler()
		: p_s_robot_(Robot::instance()),
		  observe_id_(0),
		  thread_(0),
		  is_thread_running_(false),
		  should_thread_run_(true),
		  should_broadcast_(true)
{}

RealtimeMapHandler::~RealtimeMapHandler()
{
	// Safety measure, make sure it's stopped
	onStop();
}

void RealtimeMapHandler::onStart( void )
{
	ROS_INFO( "RealtimeMapHandler", "onStart()" );
	assert( !is_thread_running_ );
	if ( is_thread_running_ )
	{
		ROS_INFO( "RealtimeMapHandler::onStart",
				"Thread already running, did you call onStart() twice?" );
		return;
	}

	if ( !p_s_robot_->isContinueCleaning() )
	{
		// Clear old realtime map
		ROS_INFO( "RealtimeMapHandler", "Clear map data on cloud" );
		wifi::ClearRealtimeMapTxMsg msg( true,
				p_s_robot_->wifiTxManager().nextSeqNum() );
		p_s_robot_->wifiTxManager().push( std::move( msg ) ).commit();
		clearAppMap();
		stopwatch_.start();
	}
	else
	{
		stopwatch_.resume();
	}

	should_broadcast_ = p_s_robot_->isEnableCloudRealtimeMap().val();
	observe_id_ = p_s_robot_->isEnableCloudRealtimeMap().observe( [this]( const bool &a_v ) {
		should_broadcast_ = a_v;
	} );

	int err;
	if ((err = pthread_create( &thread_, nullptr, threadMainWrapper, this)) != 0)
	{
		ROS_INFO( "RealtimeMapHandler::onStart", "Failed while pthread_create" );
		return;
	}
	is_thread_running_ = true;
}

void RealtimeMapHandler::onStop( void )
{
	ROS_INFO( "RealtimeMapHandler", "onStop()" );
	should_thread_run_ = false;
	if ( is_thread_running_ )
	{
		pthread_join( thread_, nullptr );
		is_thread_running_ = false;
	}
	// Send the remaining data
	if ( !positions_.empty() )
	{
		updateCloudMap();
	}

	stopwatch_.pause();
	p_s_robot_->isEnableCloudRealtimeMap().unobserve( observe_id_ );
	reset();
}

void* RealtimeMapHandler::threadMainWrapper( void *a_p_that )
{
	RealtimeMapHandler *const that =
			static_cast<RealtimeMapHandler*>( a_p_that );
	that->threadMain();
	return nullptr;
}

void RealtimeMapHandler::threadMain( void )
{
	unsigned time_i = 0;
	while ( should_thread_run_ )
	{
		DataRobot d;
		if ( p_s_robot_->getRobotData( &d ) )
		{
			auto p = d.s_exact_cell_;
			for ( auto x = -1; x < 2; ++x )
			{
				for ( auto y = -1; y < 2; ++y )
				{
					positions_.push_back( Point16_t( d.s_exact_cell_.x_ + x,
							d.s_exact_cell_.y_ + y ) );
				}
			}
		}
		usleep( 250000 );
		// 250ms * 20 = 5s
		if ( ++time_i >= 20 )
		{
			time_i = 0;
			updateCloudMap();
		}
	}
	ROS_INFO( "RealtimeMapHandler::threadMain", "Thread quitting" );
}

void RealtimeMapHandler::reset( void )
{
	positions_.clear();
	thread_ = 0;
	is_thread_running_ = false;
	should_thread_run_ = true;
	should_broadcast_ = true;
}

void RealtimeMapHandler::updateCloudMap( void )
{
	if ( broadcastPosition() )
	{
		positions_.clear();
	}
}

bool RealtimeMapHandler::broadcastPosition( void )
{
	if ( !should_broadcast_ )
	{
		return false;
	}

	timeval tv;
	gettimeofday( &tv, nullptr );
	const auto session_diff_s = stopwatch_.get<chrono::seconds>();
	// 255(max length) - 14(fixed data) = 241(max map data size per packet)
	const auto packet_count = ( positions_.size() * 4 + 239 ) / 240;
	ROS_INFO( "RealtimeMapHandler::broadcastPosition",
			"Sending %u position data in %u packet", positions_.size(),
			packet_count );

	auto it = positions_.cbegin();
	for ( size_t i = 0; i < packet_count; ++i )
	{
		vector<uint8_t> data = {
			0, 10,
			static_cast<uint8_t>(( session_diff_s & 0xFF00 ) >> 8 ),
			static_cast<uint8_t>( session_diff_s & 0xFF )
		};
		for ( auto j = 0; j < 240 / 4 && it != positions_.cend(); ++j )
		{
			data.push_back( it->x_ >> 8 );
			data.push_back( it->x_ );
			data.push_back( it->y_ >> 8 );
			data.push_back( it->y_ );
			++it;
		}
		wifi::RealtimeMapUploadTxMsg msg( tv.tv_sec, i + 1, packet_count, data,
				p_s_robot_->wifiTxManager().nextSeqNum() );
		p_s_robot_->wifiTxManager().push( std::move( msg ) ).commit();
	}
	return true;
}

void RealtimeMapHandler::clearAppMap( void )
{
	const vector<uint8_t> data = {0, 0, 0, 0, 0x7F, 0xFF, 0x7F, 0xFF};
	wifi::RealtimeMapUploadTxMsg msg( 0, 1, 1, data,
			p_s_robot_->wifiTxManager().nextSeqNum() );
}

}
