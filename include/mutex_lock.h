#pragma once

#include <pthread.h>

/**
 * Provides a RAII way to lock a mutex
 */
class MutexLock
{
public:
	explicit MutexLock( pthread_mutex_t *const a_p_mutex )
			: mutex_(a_p_mutex)
	{
		pthread_mutex_lock(mutex_);
	}

	~MutexLock( void )
	{
		pthread_mutex_unlock(mutex_);
	}

private:
	pthread_mutex_t *const mutex_;
};
