#include <stdarg.h>
#include <stdio.h>

#include "common.h"
#include "log.h"

static int	log_enabled = 0;
static int	log_level = 0;
static FILE	*fd = NULL;

int log_init()
{
	const char	*path = "ilife.log";

	log_enabled = 0;
	if ((fd = fopen(path, "a+")) == NULL) {
		ilife_error("Failed to open log file to write:", path);
		return -1;
	}

	log_enabled = 1;
	return 0;
}

void log_deinit()
{
	if (fd != NULL) {
		fclose(fd);
		fd = NULL;
	}
}

void log_set_level(int level)
{
	log_level = level;
}

void log_msg(int level, const char *fmt, ...)
{
	va_list	ap;

	if (log_enabled == 0) {
		return;
	}

	printf("%s %d: %d %d\n", __FUNCTION__, __LINE__, log_level, level);
	if (log_level > level)
		return;

	va_start(ap, fmt);
	vfprintf(fd, fmt, ap);
	va_end(ap);
	fflush(fd);
}
