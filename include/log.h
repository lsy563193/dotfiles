#ifndef __LOG_H__
#define __LOG_H__

typedef enum {
	LOG_NONE = 0,
	LOG_TRACE,
	LOG_VERBOSE,
	LOG_FATAL,
	LOG_ERROR,
} LOG_TYPE;

int log_init();
void log_deinit();

void log_set_level(int level);
void log_msg(int level, const char *fmt, ...);

#endif
