#pragma once

#include <deque>
#include <pthread.h>
#include "stopwatch.h"

class Point16_t;
class Robot;

namespace cloud
{

class RealtimeMapHandler
{
public:
	RealtimeMapHandler();
	~RealtimeMapHandler();

	void onStart( void );
	void onStop( void );

private:
	static void* threadMainWrapper( void* );
	void threadMain( void );

	void reset( void );
	void updateCloudMap( void );
	bool broadcastPosition( void );
	/**
	 * Clear realtime map data in the mobile App
	 */
	void clearAppMap( void );

	Robot *p_s_robot_;
	std::deque<Point16_t> positions_;
	Stopwatch stopwatch_;
	unsigned observe_id_;

	pthread_t thread_;
	volatile bool is_thread_running_;
	volatile bool should_thread_run_;
	volatile bool should_broadcast_;
};

}
