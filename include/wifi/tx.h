#pragma once

#include <deque>
#include <functional>
#include <pthread.h>
#include <semaphore.h>
#include "wifi/packet.h"

namespace wifi
{

class Tx
{
public:
	bool operator()( const Packet &packet );
};

class TxManager
{
public:
	typedef std::function<void(const Packet&)> OnNextListener;
	typedef std::function<void(const Packet&)> OnErrorListener;

	struct PushOp
	{
		friend class TxManager;

	public:
		PushOp( TxManager *const p_s_manager, Packet &&packet );
		PushOp( PushOp &&rhs );
		~PushOp();

		/**
		 * Set a listener to be called when the op has succeeded
		 *
		 * @param l
		 * @return this ref
		 */
		PushOp& onNext( const OnNextListener &l )
		{
			on_next_ = l;
			return *this;
		}

		/**
		 * Set a listener to be called when the op has failed
		 *
		 * @param l
		 * @return this ref
		 */
		PushOp& onError( const OnNextListener &l )
		{
			on_error_ = l;
			return *this;
		}

		/**
		 * Perform the op. You must only commit once per object
		 */
		void commit( void );

	private:
		TxManager *p_s_parent_;
		bool has_commited_;
		Packet packet_;
		OnNextListener on_next_;
		OnErrorListener on_error_;
	};

	TxManager();
	~TxManager();

	/**
	 * Prepare a new Tx operation with the supplied packet as data
	 *
	 * @param packet
	 * @return A push operation for @a packet
	 */
	PushOp push( Packet &&packet );
	/**
	 * Perform a push operation which will put the bundled packet in queue and
	 * ready to be sent out
	 *
	 * @param op
	 */
	void push( PushOp &&op );

	/**
	 * A generic ACK routine. Basically it just send @a packet back to the cloud
	 *
	 * @param packet
	 * @param on_next Called when the op succeeded
	 * @param on_error Called when the op failed
	 */
	void ack( const Packet &packet, const OnNextListener &on_next = nullptr,
			const OnErrorListener &on_error = nullptr );

	/**
	 * Return the next sequence number for Tx message
	 *
	 * @return
	 */
	uint8_t nextSeqNum( void );

private:
	struct QueueObj
	{
		QueueObj() = default;
		QueueObj(Packet &&p, const OnNextListener &on_next = nullptr,
				const OnErrorListener &on_error = nullptr);
		QueueObj(const QueueObj&) = delete;
		QueueObj(QueueObj &&rhs);

		explicit operator bool() const
		{
			return static_cast<bool>( p_ );
		}

		Packet p_;
		OnNextListener on_next_;
		OnErrorListener on_error_;
	};

	static void* threadMainWrapper( void *that );
	void threadMain( void );
	QueueObj popBlocking( void );

	pthread_t thread_;
	volatile bool is_thread_running_;
	volatile bool should_thread_run_;
	Tx tx_;
	pthread_mutex_t queue_lock_;
	sem_t queue_semaphore_;
	std::deque<QueueObj> s_queue_;

	pthread_mutex_t seq_num_lock_;
	volatile uint8_t seq_num_;
};

}
