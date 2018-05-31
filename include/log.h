#pragma once

#include <chrono>
#include <deque>
#include <memory>
#include <string>
#include <vector>
#include <pthread.h>
#include <semaphore.h>

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

class Log
{
public:
	enum struct Level
	{
		DEBUG_ = 0,
		INFO,
		WARN,
		ERROR,
		FATAL,
	};

	struct DataObj
	{
		DataObj() = default;
		DataObj( const Level level, const std::string &tag,
				const std::string &msg );
		DataObj( const DataObj& ) = delete;
		DataObj( DataObj&& ) = default;
		DataObj& operator=( const DataObj& ) = delete;
		DataObj& operator=( DataObj&& ) = default;

		operator bool() const
		{
			return !msg_.empty();
		}

		std::chrono::system_clock::time_point time_ =
				std::chrono::system_clock::now();
		Level level_ = Level::DEBUG_;
		std::string tag_;
		std::string msg_;
	};

	class Backend
	{
	public:
		virtual ~Backend()
		{}

		virtual void operator()( const Level l, const std::string &str ) = 0;
	};

	class Formatter
	{
	public:
		virtual ~Formatter()
		{}

		virtual std::string operator()( const DataObj &d ) = 0;
	};

	struct Consumer
	{
		std::unique_ptr<Log::Formatter> formatter_;
		std::unique_ptr<Log::Backend> backend_;
	};

	/**
	 * Get instance
	 *
	 * @return  pointer to instance
	 */
	static Log& inst( void );

	void setLevel( const Level l )
	{
		level_ = l;
	}

	Level level( void ) const
	{
		return level_;
	}

	/**
	 * Set the name of the process, so that the name will be shown when types the
	 * 'ps -T' command in terminal
	 *
	 * @param name	Name of the process to be set
	 */
	void setProcessName( const char *name );

	/**
	 * Log a msg
	 *
	 * @param tag
	 * @param fmt
	 */
	void fatal( const char *tag, const char *fmt, ...);
	void error( const char *tag, const char *fmt, ...);
	void warn( const char *tag, const char *fmt, ...);
	void info( const char *tag, const char *fmt, ...);
	void debug( const char *tag, const char *fmt, ...);

private:
	/**
	 * Constructor, set the current debug level
	 *
	 * @param log level
	 */
	Log();

	/**
	 * Destructor
	 */
	~Log();

	static void* threadMainWrapper( void *that );
	void threadMain();

	void log( const Level l, const char *tag, const char *fmt, va_list args );

	DataObj popBlocking( void );

	pthread_t thread_;
	volatile bool is_thread_running_;
	volatile bool should_thread_run_;

	pthread_mutex_t queue_lock_;
	sem_t queue_semaphore_;
	std::deque<DataObj> queue_;

	Level level_;
	std::vector<Log::Consumer> consumers_;
};

#define LOGD(...) Log::inst().debug("", __VA_ARGS__);
#define LOGI(...) Log::inst().info("", __VA_ARGS__);
#define LOGW(...) Log::inst().warn("", __VA_ARGS__);
#define LOGE(...) Log::inst().error("", __VA_ARGS__);
#define LOGF(...) Log::inst().fatal("", __VA_ARGS__);

#define LOGD2(tag, fmt...) Log::inst().debug(tag, fmt);
#define LOGI2(tag, fmt...) Log::inst().info(tag, fmt);
#define LOGW2(tag, fmt...) Log::inst().warn(tag, fmt);
#define LOGE2(tag, fmt...) Log::inst().error(tag, fmt);
#define LOGF2(tag, fmt...) Log::inst().fatal(tag, fmt);
