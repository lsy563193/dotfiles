#ifndef __COMMON_H__
#define __COMMON_H__

#define	PROGNAME	"ilife"

void ilife_error(const char *fmt, ...);
void datetime(char *dt, int milli);
int map(int val,int mmin,int mmax,int amin,int amax);
#endif
