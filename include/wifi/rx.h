#pragma once

#include <functional>
#include <list>
#include <memory>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>
#include <pthread.h>
#include "wifi/msg.h"
#include "wifi/packet.h"

namespace wifi
{

class Rx
{
public:
	std::vector<uint8_t> operator()();
};

class RxManager
{
public:
	typedef std::function<void(const RxMsg&)> OnNewMsgListener;

	RxManager();
	~RxManager();

	/** * Reg a listener to be notified when a new msg received. You must not call
	 * this inside a listener which will result in a deadlock
	 *
	 * @param l
	 * @return Unique ID of the listener, used to unreg
	 * @see unregOnNewMsgListener()
	 */
	template<typename T>
	unsigned regOnNewMsgListener( const OnNewMsgListener &l );
	template<typename T>
	void unregOnNewMsgListener( const unsigned id );

private:
	static void* threadMainWrapper( void *that );
	void threadMain();

	/**
	 * Convert a Packet to a RxMsg, if possible
	 *
	 * @param packet
	 * @return
	 */
	std::unique_ptr<RxMsg> toMsg( Packet &&packet );

	void handleMsg( Packet &&packet );
	template<typename T> void handleMsg( const T &msg );

	pthread_t thread_;
	Rx rx_;
	Packet::Parser s_parser_;
	volatile bool is_thread_running_;
	volatile bool should_thread_run_;

	pthread_mutex_t on_new_msg_mutex_;
	std::unordered_map<std::type_index, std::list< std::pair<unsigned, OnNewMsgListener> > >on_new_msg_listeners_;
	unsigned listener_id_;
};

}

#include "wifi/rx.tcc"
