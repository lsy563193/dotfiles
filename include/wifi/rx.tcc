#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <utility>
#include "mutex_lock.h"
#include "wifi/rx.h"

namespace wifi
{

template<typename T>
unsigned RxManager::regOnNewMsgListener( const OnNewMsgListener &l )
{
	if ( !l )
	{
		return -1;
	}
	MutexLock lock( &on_new_msg_mutex_ );
	on_new_msg_listeners_[typeid( T )].push_back( std::make_pair( listener_id_,
			l ));
	return listener_id_++;
}

template<typename T>
void RxManager::unregOnNewMsgListener( const unsigned a_id )
{
	if ( a_id == static_cast<unsigned>( -1 ))
	{
		// Invalid id
		return;
	}
	MutexLock lock( &on_new_msg_mutex_ );
	on_new_msg_listeners_[typeid( T )].remove_if(
			[&]( const decltype( on_new_msg_listeners_ )::mapped_type::value_type &rhs ) {
				return a_id == rhs.first;
			});
}

}
