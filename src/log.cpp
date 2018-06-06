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
#include <unistd.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <ros/ros.h>
#include "mutex_lock.h"
#include "log.h"

using namespace std;

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

	void Flush() override;
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

	void Flush() override;
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

Log::DataObj::DataObj( const Level a_level, const string &a_tag,const string &color,
		const string &a_msg )
		: level_( a_level ),
		  tag_( a_tag ),
		  color_(color),
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

void Log::log( const Level a_l, const char *a_tag, const char* color,const char *a_fmt,
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
	queue_.push_back( DataObj( a_l, a_tag, color,string( msg.get() ) ) );
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

void Log::info( const char *a_tag, const char *color,const char *a_fmt, ...)
{
	va_list args;
	va_start( args, a_fmt );
	log( Level::INFO, a_tag, color,a_fmt, args );
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
void Log::Flush()
{
	printf("menual flush!\n");
	for ( auto &c: consumers_ )
	{
		(*c.backend_).Flush();
		break;
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

void FileBackend::Flush()
{
	file_.flush();
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
		file_ << "\n== == *O.O* == PIKA? == *O.O* == ==\033[0m\n";
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
	fprintf( stdout, "%s%s\033[0m\n", col, a_str.c_str() );
}
void ConsoleBackend::Flush()
{
	fsync((int)stdout);
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

	if ( ! a_d.tag_.empty() )
	{
		ss << "["<<a_d.tag_<<"]";
	}
	if ( a_d.color_.empty() )
	{
		ss <<a_d.msg_;
	}
	else
	{
		if(a_d.color_== "BLUE")
			ss << COLOUR_BLUE<< a_d.msg_;
		else if(a_d.color_== "LIGHT_BLUE")
			ss << COLOUR_LIGHT_BLUE<< a_d.msg_;
		else if(a_d.color_ == "RED")
			ss << COLOUR_RED<< a_d.msg_;
		else if(a_d.color_== "LIGHT_RED")
			ss << COLOUR_LIGHT_RED<< a_d.msg_;
		else if(a_d.color_ == "DARY_GRAY")
			ss << COLOUR_DARY_GRAY<< a_d.msg_;
		else if(a_d.color_ == "LIGHT_GRAY")
			ss <<COLOUR_LIGHT_GRAY<< a_d.msg_;
		else if(a_d.color_ == "GREEN")
			ss << COLOUR_GREEN<< a_d.msg_;
		else if(a_d.color_ == "LIGHT_GREEN")
			ss << COLOUR_LIGHT_GREEN<< a_d.msg_;
		else if(a_d.color_ == "CYAN")
			ss << COLOUR_CYAN<< a_d.msg_;
		else if(a_d.color_== "LIGHT_CYAN")
			ss << COLOUR_LIGHT_CYAN<< a_d.msg_;
		else if(a_d.color_ == "PURPLE")
			ss << COLOUR_PURPLE<< a_d.msg_;
		else if(a_d.color_ == "LIGHT_PURPLE")
			ss << COLOUR_LIGHT_PURPLE<< a_d.msg_;
		else if(a_d.color_ == "WHITE")
			ss << COLOUR_WHITE<< a_d.msg_;
		else if(a_d.color_ == "YELLOW")
			ss <<COLOUR_YELLOW<< a_d.msg_;
		else if(a_d.color_ == "LIGHT_YELLOW")
			ss <<COLOUR_LIGHT_YELLOW<< a_d.msg_;

	}
	ss<<COLOUR_NONE;
	return ss.str();
}

}

void INFO_PRINT(int type,const char* arg,...)
{
	va_list va, va_cp;
	va_start(va,arg);
	va_copy(va_cp,va);
	const int count = vsnprintf(nullptr,0,arg,va_cp);
	if(count <0)
	{
		fprintf(stderr,"vsnprintf error");
		return;
	}
	char buffer[count+1];
	va_end(va_cp);
	vsprintf(buffer,arg,va);
	va_end(va);
	switch(type)
	{
		case ERROR:
			ROS_ERROR("%s", buffer);
			break;
		case WARN:
			ROS_WARN("%s", buffer);
			break;
		case NORMAL:
			ROS_INFO("%s",buffer);
			break;
		case WHITE:
			ROS_INFO("\033[37m%s",buffer);
			break;
		case BLUE:
			ROS_INFO("\033[34m%s",buffer);
			break;
		case RED:
			ROS_INFO("\033[31m%s",buffer);
			break;
		case GREEN:
			ROS_INFO("\033[32m%s",buffer);
			break;
		case PURPLE:
			ROS_INFO("\033[35m%s",buffer);
			break;
		case CYAN:
			ROS_INFO("\033[36m%s",buffer);
			break;
		case BLACK:
			ROS_INFO("\033[30m%s",buffer);
			break;
		case YEllOW:
			ROS_INFO("\033[33m%s",buffer);
			break;
		default:
			ROS_INFO("%s",buffer);
	}
}
