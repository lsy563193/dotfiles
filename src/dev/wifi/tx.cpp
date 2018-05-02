#include <iomanip>
#include <sstream>
#include <pthread.h>
#include "mutex_lock.h"
//#include "log.h"
#include <ros/ros.h>
#include "wifi/dev.h"
#include "wifi/packet.h"
#include "wifi/tx.h"

using namespace std;

namespace wifi
{

bool Tx::operator()( const Packet &a_packet )
{
	const auto data = a_packet.serialize();
//	stringstream ss;
//	ss << std::hex << setfill('0');
//	for ( size_t i = 0; i < data.size(); ++i )
//	{
//		ss << setw(2) << (unsigned)data[i] << ' ';
//	}
//	LOGE2( "Tx::operator()", "%s", ss.str().c_str() );
	return Dev::instance()->tx( data );
}

TxManager::PushOp::PushOp( TxManager *const a_p_s_manager, Packet &&a_packet )
		: p_s_parent_(a_p_s_manager),
		  has_commited_(false),
		  packet_(std::move( a_packet ))
{}

TxManager::PushOp::PushOp( PushOp &&rhs )
		: p_s_parent_(rhs.p_s_parent_),
		  has_commited_(rhs.has_commited_),
		  packet_(std::move(rhs.packet_)),
		  on_next_(std::move(rhs.on_next_)),
		  on_error_(std::move(rhs.on_error_))
{
	rhs.p_s_parent_ = nullptr;
	rhs.has_commited_ = true;
}

TxManager::PushOp::~PushOp()
{
	if ( !has_commited_ )
	{
		stringstream ss;
		ss << "Did you forget to commit? " << std::hex << setfill('0');
		const auto data = packet_.serialize();
		for ( size_t i = 0; i < data.size() && i < 20; ++i )
		{
			ss << setw(2) << (unsigned)data[i] << ' ';
		}
		ROS_INFO( "wifi::TxManager::PushOp::~PushOp", "%s", ss.str().c_str() );
	}
}

void TxManager::PushOp::commit( void )
{
	has_commited_ = true;
	p_s_parent_->push( std::move( *this ));
}

TxManager::QueueObj::QueueObj( Packet &&p, const OnNextListener &on_next,
		const OnErrorListener &on_error )
		: p_( std::move( p )),
		  on_next_( on_next ),
		  on_error_( on_error )
{}

TxManager::QueueObj::QueueObj( QueueObj &&rhs )
		: p_( std::move( rhs.p_ )),
		  on_next_( rhs.on_next_ ),
		  on_error_( rhs.on_error_ )
{}

TxManager::TxManager()
		: thread_(0),
		  is_thread_running_(false),
		  should_thread_run_(true),
		  queue_lock_(PTHREAD_MUTEX_INITIALIZER),
		  seq_num_lock_(PTHREAD_MUTEX_INITIALIZER),
		  seq_num_(0)
{
	int err;
	if ((err = sem_init( &queue_semaphore_, 0, 0 )) != 0)
	{
		ROS_ERROR( "wifi::TxManager::TxManager", "Failed while sem_init: %d", err );
		return;
	}
	if ((err = pthread_create( &thread_, nullptr, threadMainWrapper, this)) != 0)
	{
		ROS_ERROR( "wifi::TxManager::TxManager", "Failed while pthread_create: %d",
				err );
		return;
	}
	is_thread_running_ = true;
}

TxManager::~TxManager()
{
	should_thread_run_ = false;
	if ( is_thread_running_ )
	{
		sem_post( &queue_semaphore_ );
		pthread_join( thread_, NULL );
		sem_destroy( &queue_semaphore_ );
	}
	printf("%s %d: Exit.\n", __FUNCTION__, __LINE__);
	pthread_mutex_destroy( &queue_lock_ );
	printf("%s %d: Exit.\n", __FUNCTION__, __LINE__);
}

TxManager::PushOp TxManager::push( Packet &&a_packet )
{
	return PushOp( this, std::move( a_packet ) );
}

void TxManager::push( PushOp &&a_op )
{
	MutexLock lock( &queue_lock_ );
	s_queue_.push_back(QueueObj{
		std::move( a_op.packet_ ),
		a_op.on_next_,
		a_op.on_error_
	});
	sem_post( &queue_semaphore_ );
}

void TxManager::ack( const Packet &a_packet, const OnNextListener &a_on_next,
		const OnErrorListener &a_on_error )
{
	wifi::Packet p( -1, a_packet.seq_num(), a_packet.add_code(),
			a_packet.msg_code(), a_packet.data() );
	MutexLock lock( &queue_lock_ );
	s_queue_.push_back(QueueObj{
		std::move( p ),
		a_on_next,
		a_on_error
	});
	sem_post( &queue_semaphore_ );
}

void* TxManager::threadMainWrapper( void *a_p_that )
{
	TxManager *const that = static_cast<TxManager*>(a_p_that);
	that->threadMain();
	return nullptr;
}

void TxManager::threadMain()
{
	while ( should_thread_run_ )
	{
		const QueueObj obj = popBlocking();
		if ( !obj )
		{
			continue;
		}

		if ( tx_( obj.p_ ) )
		{
			uint32_t sleep_nsec = obj.p_.length() > 250 ? static_cast<uint32_t>(obj.p_.length() * 1000) : 250000;
//			ROS_INFO("%s %d: Packet length:%d, usleep nsec:%d.", __FUNCTION__, __LINE__, obj.p_.length(), sleep_nsec);
			usleep(sleep_nsec);
			if ( obj.on_next_ )
			{
				obj.on_next_( obj.p_ );
			}
		}
		else
		{
			if ( obj.on_error_ )
			{
				obj.on_error_( obj.p_ );
			}
		}
	}
}

TxManager::QueueObj TxManager::popBlocking( void )
{
	// Wait until there's a packet
	sem_wait( &queue_semaphore_ );
	MutexLock lock( &queue_lock_ );
	// Consume the packet
	if ( s_queue_.empty() )
	{
		// ???
		ROS_WARN( "wifi::TxManager::popBlocking",
				"semaphore signaled w/o actual data" );
		return {};
	}
	QueueObj obj = std::move( s_queue_.front() );
	s_queue_.pop_front();
	return obj;
}

uint8_t TxManager::nextSeqNum( void )
{
	MutexLock lock( &seq_num_lock_ );
	return seq_num_++;
}

}
