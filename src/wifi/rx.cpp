#include <cstdint>
#include <functional>
#include <list>
#include <memory>
#include <typeinfo>
#include <vector>
#include <pthread.h>
#include <ros/ros.h>
#include "mutex_lock.h"
#include "wifi/dev.h"
#include "wifi/packet.h"
#include "wifi/rx.h"

using namespace std;

namespace wifi
{

vector<uint8_t> Rx::operator()()
{
	return Dev::instance()->rx();
}

RxManager::RxManager()
		: thread_(0),
		  is_thread_running_(false),
		  should_thread_run_(true),
		  on_new_msg_mutex_(PTHREAD_MUTEX_INITIALIZER)
{
	s_parser_.on_new_packet_ = [this]( Packet &&p ) {
		handleMsg( std::move( p ) );
	};
	int err;
	if ((err = pthread_create( &thread_, nullptr, threadMainWrapper, this)) != 0)
	{
		ROS_ERROR( "wifi::RxManager::RxManager, Failed while pthread_create" );
		return;
	}
	is_thread_running_ = true;
}

RxManager::~RxManager()
{
	should_thread_run_ = false;
	if ( is_thread_running_ )
	{
		pthread_join( thread_, NULL );
	}
	pthread_mutex_destroy( &on_new_msg_mutex_ );
}

void RxManager::handleMsg( Packet &&a_packet )
{
	switch ( a_packet.msg_code() )
	{
	case FactoryTestRxMsg::MSG_CODE:
		handleMsg( FactoryTestRxMsg( std::move( a_packet )) );
		break;

	case RegDeviceRequestRxMsg::MSG_CODE:
		handleMsg( RegDeviceRequestRxMsg( std::move( a_packet )) );
		break;

	case CloudConnectedNotifRxMsg::MSG_CODE:
		handleMsg( CloudConnectedNotifRxMsg( std::move( a_packet )) );
		break;

	case CloudDisconnectedNotifRxMsg::MSG_CODE:
		handleMsg( CloudDisconnectedNotifRxMsg( std::move( a_packet )) );
		break;

	case QueryDeviceStatusRxMsg::MSG_CODE:
		handleMsg( QueryDeviceStatusRxMsg( std::move( a_packet )) );
		break;

	case QueryScheduleStatusRxMsg::MSG_CODE:
		handleMsg( QueryScheduleStatusRxMsg( std::move( a_packet )) );
		break;

	case QueryConsumableStatusRxMsg::MSG_CODE:
		handleMsg( QueryConsumableStatusRxMsg( std::move( a_packet )) );
		break;

	case SetModeRxMsg::MSG_CODE:
		handleMsg( SetModeRxMsg( std::move( a_packet )) );
		break;

	case SetRoomModeRxMsg::MSG_CODE:
		handleMsg( SetRoomModeRxMsg( std::move( a_packet )) );
		break;

	case SetMaxCleanPowerRxMsg::MSG_CODE:
		handleMsg( SetMaxCleanPowerRxMsg( std::move( a_packet )) );
		break;

	case RemoteControlRxMsg::MSG_CODE:
		handleMsg( RemoteControlRxMsg( std::move( a_packet )) );
		break;

	case ResetConsumableStatusRxMsg::MSG_CODE:
		handleMsg( ResetConsumableStatusRxMsg( std::move( a_packet )) );
		break;

	case SyncClockRxMsg::MSG_CODE:
		handleMsg( SyncClockRxMsg( std::move( a_packet )) );
		break;

	case RealtimeStatusRequestRxMsg::MSG_CODE:
		handleMsg( RealtimeStatusRequestRxMsg( std::move( a_packet )) );
		break;

	case SetDoNotDisturbRxMsg::MSG_CODE:
		handleMsg( SetDoNotDisturbRxMsg( std::move( a_packet )) );
		break;

	case FactoryResetRxMsg::MSG_CODE:
		handleMsg( FactoryResetRxMsg( std::move( a_packet )) );
		break;

	case DeviceStatusUploadAckMsg::MSG_CODE:
		handleMsg( DeviceStatusUploadAckMsg( std::move( a_packet )) );
		break;

	case RealtimeMapUploadAckMsg::MSG_CODE:
		handleMsg( RealtimeMapUploadAckMsg( std::move( a_packet )) );
		break;

	case AccumulatedWorkTimeUploadAckMsg::MSG_CODE:
		handleMsg( AccumulatedWorkTimeUploadAckMsg( std::move( a_packet )) );
		break;

	case ClearRealtimeMapAckMsg::MSG_CODE:
		handleMsg( ClearRealtimeMapAckMsg( std::move( a_packet )) );
		break;

	default:
		ROS_WARN( "wifi::RxManager::handleMsg", "Unknown msg_code: %u", a_packet.msg_code() );
		break;
	}
}

template<typename T>
void RxManager::handleMsg( const T &a_msg )
{
	ROS_INFO( "%s,%d,wifi::RxManager::handleMsg", "[%d] %s",__FUNCTION__,__LINE__, a_msg.seq_num(),
			a_msg.describe().c_str() );
	MutexLock lock( &on_new_msg_mutex_ );
	for ( const auto &p : on_new_msg_listeners_[typeid( a_msg )] )
	{
		// Handler exists
		p.second( a_msg );
	}
}

void* RxManager::threadMainWrapper( void *a_p_that )
{
	RxManager *const that = static_cast<RxManager*>(a_p_that);
	that->threadMain();
	return nullptr;
}

void RxManager::threadMain()
{
	while ( should_thread_run_ )
	{
		const auto &data = rx_();
		s_parser_.push( data );
	}
}

}
