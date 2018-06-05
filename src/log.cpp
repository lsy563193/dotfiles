#include <cstdarg>
#include <cstdlib>
#include <cerrno>
#include <algorithm>
#include <chrono>
#include <deque>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <dirent.h>
#include <pthread.h>
#include <semaphore.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include "mutex_lock.h"
#include "log.h"

using namespace std;

#define COLOUR_NONE			"\033[m"
#define COLOUR_RED			"\033[0;32;31m"
#define COLOUR_LIGHT_RED	"\033[1;31m"
#define COLOUR_GREEN		"\033[0;32;32m"
#define COLOUR_LIGHT_GREEN	"\033[1;32m"
#define COLOUR_BLUE			"\033[0;32;34m"
#define COLOUR_LIGHT_BLUE	"\033[1;34m"
#define COLOUR_DARY_GRAY	"\033[1;30m"
#define COLOUR_CYAN			"\033[0;36m"
#define COLOUR_LIGHT_CYAN	"\033[1;36m"
#define COLOUR_PURPLE		"\033[0;35m"
#define COLOUR_LIGHT_PURPLE	"\033[1;35m"
#define COLOUR_BROWN		"\033[0;33m"
#define COLOUR_YELLOW		"\033[1;33m"
#define COLOUR_LIGHT_GRAY	"\033[0;37m"
#define COLOUR_WHITE		"\033[1;37m"

namespace
{

constexpr const char *DIR = "/mnt/UDISK/log";
constexpr const unsigned LINE_LIMIT = 300000;
constexpr const unsigned FILE_LIMIT = 30;

class FileBackend : public Log::Backend
{
public:
	FileBackend();

	void operator()( const Log::Level, const std::string &str ) override;

private:
	/**
	 * Instead of starting in the next file, we resume from the last file
	 */
	void resumeFile( void );

	void ensureFile( void );
	/**
	 * Begin writing in the next file
	 *
	 * @return true if successful, false otherwise
	 */
	bool rotateFile( void );
	/**
	 * Prevent too many files from existing
	 */
	void limitFiles( void );
	/**
	 * Remove all logs
	 */
	void clearAll( void );

	static std::fstream openFile( const unsigned id,
			const std::ios_base::openmode mode );
	static std::fstream openFile( const unsigned id )
	{
		return openFile( id, std::ios_base::out | std::ios_base::trunc );
	}

	/**
	 * Return the last log ID actually being used by querying the file system
	 *
	 * @return Log ID, or 0 if no logs are found on the file system
	 */
	static unsigned queryLogId( void );

	std::fstream file_;
	unsigned line_written_;
	unsigned log_id_;
};

class ConsoleBackend : public Log::Backend
{
public:
	void operator()( const Log::Level l, const std::string &str ) override;
};

class JsonFormatter : public Log::Formatter
{
public:
	string operator()( const Log::DataObj &d ) override;
};

class TextFormatter : public Log::Formatter
{
public:
	string operator()( const Log::DataObj &d ) override;
};

}

Log::DataObj::DataObj( const Level a_level, const string &a_tag,
		const string &a_msg )
		: level_( a_level ),
		  tag_( a_tag ),
		  msg_( a_msg )
{}

Log::Log()
		: thread_( 0 ),
		  is_thread_running_( false ),
		  should_thread_run_( true ),
		  level_( Level::DEBUG_ )
{
	int err;
	if ((err = pthread_create( &thread_, nullptr, threadMainWrapper, this)) != 0)
	{
		printf( "[Log::Log] Failed while pthread_create\n" );
		return;
	}
	is_thread_running_ = true;

	consumers_.push_back( Log::Consumer{
		unique_ptr<Formatter>( new TextFormatter ),
		unique_ptr<Backend>( new FileBackend )
	} );
	consumers_.push_back( Log::Consumer{
		unique_ptr<Formatter>( new TextFormatter ),
		unique_ptr<Backend>( new ConsoleBackend )
	} );
}

Log::~Log()
{
	should_thread_run_ = false;
	if ( is_thread_running_ )
	{
		sem_post( &queue_semaphore_ );
		pthread_join( thread_, nullptr );
		sem_destroy( &queue_semaphore_ );
	}
	pthread_mutex_destroy( &queue_lock_ );
}

Log& Log::inst( void )
{
	static Log instance;
	return instance;
}

void Log::setProcessName( const char *a_name )
{
	prctl( PR_SET_NAME, a_name );
}

void Log::log( const Level a_l, const char *a_tag, const char *a_fmt,
		va_list a_args )
{
	if ( (int)a_l < (int)level_ )
	{
		return;
	}

	va_list args_copy;
	va_copy( args_copy, a_args );
	const int msg_sz = vsnprintf( nullptr, 0, a_fmt, args_copy );
	va_end( args_copy );
	if ( msg_sz < 0 )
	{
		printf( "[Log::log] Failed while vsnprintf, error: %d\n", msg_sz );
		return;
	}
	unique_ptr<char[]> msg( new char[msg_sz + 1] );
	int err;
	if ( ( err = vsnprintf( msg.get(), msg_sz + 1, a_fmt, a_args ) ) != msg_sz )
	{
		printf( "[Log::log] Failed while vsnprintf2, error: %d\n", err );
		return;
	}

	MutexLock lock( &queue_lock_ );
	queue_.push_back( DataObj( a_l, a_tag, string( msg.get() ) ) );
	sem_post( &queue_semaphore_ );
}

void Log::fatal( const char *a_tag, const char *a_fmt, ...)
{
	va_list args;
	va_start( args, a_fmt );
	log( Level::FATAL, a_tag, a_fmt, args );
	va_end( args );
}

void Log::error( const char *a_tag, const char *a_fmt, ...)
{
	va_list args;
	va_start( args, a_fmt );
	log( Level::ERROR, a_tag, a_fmt, args );
	va_end( args );
}

void Log::warn( const char *a_tag, const char *a_fmt, ...)
{
	va_list args;
	va_start( args, a_fmt );
	log( Level::WARN, a_tag, a_fmt, args );
	va_end( args );
}

void Log::info( const char *a_tag, const char *a_fmt, ...)
{
	va_list args;
	va_start( args, a_fmt );
	log( Level::INFO, a_tag, a_fmt, args );
	va_end( args );
}

void Log::debug( const char *a_tag, const char *a_fmt, ...)
{
	va_list args;
	va_start( args, a_fmt );
	log( Level::DEBUG_, a_tag, a_fmt, args );
	va_end( args );
}

void* Log::threadMainWrapper( void *a_p_that )
{
	Log *const that = static_cast<Log*>( a_p_that );
	that->threadMain();
	return nullptr;
}

void Log::threadMain()
{
	setProcessName( "log" );

	while ( should_thread_run_ )
	{
		DataObj obj = popBlocking();
		if ( !obj )
		{
			continue;
		}

		for ( auto &c: consumers_ )
		{
			// We surely don't want the program to crash while logging
			try
			{
				const auto &str = ( *c.formatter_ )( obj );
				( *c.backend_ )( obj.level_, str );
			}
			catch ( const exception &e )
			{
				printf( "[Log::threadMain] Exception while logging\n%s",
						e.what() );
			}
			catch ( ... )
			{
				printf( "[Log::threadMain] Unknown exception while logging" );
			}
		}
	}
}

Log::DataObj Log::popBlocking( void )
{
	// Wait until there's a packet
	sem_wait( &queue_semaphore_ );
	MutexLock lock( &queue_lock_ );
	// Consume the packet
	if ( queue_.empty() )
	{
		// ???
		printf( "[Log::popBlocking] semaphore signaled w/o actual data\n" );
		return {};
	}
	DataObj obj = std::move( queue_.front() );
	queue_.pop_front();
	return obj;
}

namespace
{

FileBackend::FileBackend( void )
		: line_written_( 0 ),
		  log_id_( 0 )
{
	system( ( string( "mkdir -p " ) + DIR ).c_str() );
	resumeFile();
}

void FileBackend::operator()( const Log::Level, const string &str )
{
	ensureFile();
	if ( file_ )
	{
		file_ << str << '\n';
		if ( ++line_written_ % 10 == 0 )
		{
			file_.flush();
		}
	}
}

void FileBackend::resumeFile( void )
{
	const unsigned id = queryLogId();
	if ( id != 0 )
	{
		log_id_ = id;
		// Append from the prev file
		auto out = openFile( log_id_, ios_base::out | ios_base::app );
		auto in = openFile( log_id_, ios_base::in );
		if ( !out.is_open() || !in.is_open() )
		{
			return;
		}

		// Then we count the number of line in the file
		line_written_ = std::count( istreambuf_iterator<char>( in ),
				istreambuf_iterator<char>(), '\n' );
		file_ = std::move( out );
		file_ << "\n== == *O.O* == PIKA? == *O.O* == ==\n";
	}
}

void FileBackend::ensureFile( void )
{
	if ( !file_ || !file_.is_open() || line_written_ >= LINE_LIMIT )
	{
		if ( rotateFile() )
		{
			line_written_ = 0;
		}
	}
}

bool FileBackend::rotateFile( void )
{
	if ( log_id_ == 0 )
	{
		log_id_ = queryLogId();
	}
	if ( log_id_ == std::numeric_limits<unsigned>::max() )
	{
		// Last file, optimally we should make it possible to warp around,
		// but... oh well, this works anyway :P
		// We need to clear to prevent the max id finding thingy preceding us
		// from failing to realize that 0 actually > 2^32...
		clearAll();
	}
	auto next = openFile( ++log_id_ );
	if ( !next || !next.is_open() )
	{
		return false;
	}
	file_ = std::move( next );
	limitFiles();
	return true;
}

void FileBackend::limitFiles( void )
{
	if ( log_id_ <= FILE_LIMIT )
	{
		return;
	}

	auto dir = opendir( DIR );
	if ( !dir )
	{
		printf( "[log::FileBackend::limitFiles] Failed while open(%s): %d\n",
				DIR, errno );
		return;
	}

	vector<string> removes;
	for ( auto d = readdir( dir ); d; d = readdir( dir ) )
	{
		const unsigned id = strtoul( &d->d_name[4], nullptr, 10 );
		if ( id <= log_id_ - FILE_LIMIT )
		{
			// Including 0
			removes.push_back( string( DIR ) + '/' + d->d_name );
		}
	}
	closedir( dir );
	for ( const auto &r: removes )
	{
		remove( r.c_str() );
		printf( "[log::FileBackend::limitFiles] Remove file: %s\n", r.c_str() );
	}
}

void FileBackend::clearAll( void )
{
	LOGI2( "log::FileBackend", "clearAll()" );
	auto dir = opendir( DIR );
	if ( !dir )
	{
		printf( "[log::FileBackend::clearAll] Failed while open(%s): %d\n", DIR,
				errno );
		return;
	}

	vector<string> removes;
	for ( auto d = readdir( dir ); d; d = readdir( dir ) )
	{
		removes.push_back( string( DIR ) + '/' + d->d_name );
	}
	closedir( dir );
	for ( const auto &r: removes )
	{
		remove( r.c_str() );
	}
}

fstream FileBackend::openFile( const unsigned a_id,
		const ios_base::openmode a_mode )
{
	stringstream ss;
	ss << DIR << "/log." << a_id << ".txt";
	fstream product( ss.str(), a_mode );
	if ( !product )
	{
		printf( "[log::FileBackend::openFile] Failed while open: %s\n",
				ss.str().c_str() );
	}
	return product;
}

unsigned FileBackend::queryLogId( void )
{
	unsigned product = 0;
	auto dir = opendir( DIR );
	if ( !dir )
	{
		printf( "[log::FileBackend::resumeFile] Failed while opendir(%s): %d\n",
				DIR, errno );
		return 0;
	}

	for ( auto d = readdir( dir ); d; d = readdir( dir ) )
	{
		const unsigned id = strtoul( &d->d_name[4], nullptr, 10 );
		product = std::max( product, id );
	}
	closedir( dir );
	return product;
}

void ConsoleBackend::operator()( const Log::Level a_l, const string &a_str )
{
	const char *col = nullptr;
	switch ( a_l )
	{
		case Log::Level::DEBUG_:
			col = COLOUR_GREEN;
			break;

		case Log::Level::INFO:
		default:
			col = COLOUR_WHITE;
			break;

		case Log::Level::WARN:
			col = COLOUR_YELLOW;
			break;

		case Log::Level::ERROR:
			col = COLOUR_LIGHT_RED;
			break;

		case Log::Level::FATAL:
			col = COLOUR_PURPLE;
			break;
	}
	fprintf( stderr, "%s%s\n", col, a_str.c_str() );
}

string JsonFormatter::operator()( const Log::DataObj &a_d )
{
	stringstream ss;
	ss << '{';
	const auto tp_ms = chrono::time_point_cast<chrono::microseconds>( a_d.time_ );
	ss << "\"time\":" << tp_ms.time_since_epoch().count();
	ss << ",\"level\":" << static_cast<int>( a_d.level_ );
	if ( !a_d.tag_.empty() )
	{
		ss << ",\"tag\":\"" << a_d.tag_ << '\"';
	}
	ss << ",\"msg\":\"";
	for ( const auto c: a_d.msg_ )
	{
		if ( c == '\"' )
		{
			ss << "\\\"";
		}
		else
		{
			ss << c;
		}
	}
	ss << "\"},";
	return ss.str();
}

string TextFormatter::operator()( const Log::DataObj &a_d )
{
	stringstream ss;

	char lv = ' ';
	switch ( a_d.level_ )
	{
		case Log::Level::DEBUG_:
			lv = 'D';
			break;

		case Log::Level::INFO:
		default:
			lv = 'I';
			break;

		case Log::Level::WARN:
			lv = 'W';
			break;

		case Log::Level::ERROR:
			lv = 'E';
			break;

		case Log::Level::FATAL:
			lv = 'F';
			break;
	}
	ss << lv;

	// [DD/MM HH:MM:SS.ssssss]
	const auto time = a_d.time_.time_since_epoch();
	timeval tv;
	tv.tv_sec = chrono::duration_cast<chrono::seconds>( time ).count();
	tv.tv_usec = chrono::duration_cast<chrono::microseconds>( time ).count()
			% 1000000;
	const auto tm = localtime( &tv.tv_sec );
	char time_str[25] = {};
	snprintf( time_str, 25, "[%02d/%02d %02d:%02d:%02d.%06ld] ", tm->tm_mday,
			tm->tm_mon + 1, tm->tm_hour, tm->tm_min, tm->tm_sec, tv.tv_usec );
	ss << time_str;

	if ( a_d.tag_.empty() )
	{
		ss << a_d.msg_;
	}
	else
	{
		ss << '[' << a_d.tag_ << "] " << a_d.msg_;
	}
	return ss.str();
}

}

