#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/timeb.h>
#include <time.h>

#include "common.h"

void datetime(char *dt, int milli) {
	struct tm	tb;
	struct timeb	t;

	if (dt == NULL)
		return;

	ftime(&t);
	if (!localtime_r(&t.time, &tb)) {
		memset(&tb, 0, sizeof(tb));
	}

	/* XXX */
	if(t.millitm >= 1000)
		t.millitm = 999;

	if (milli) {
		sprintf(dt, "%04d/%02d/%02d %02d:%02d:%02d.%03d",
				tb.tm_year+1900, tb.tm_mon+1, tb.tm_mday, tb.tm_hour, tb.tm_min,
				tb.tm_sec, t.millitm );
	} else {
		sprintf(dt, "%04d/%02d/%02d %02d:%02d:%02d", tb.tm_year+1900,
				tb.tm_mon+1, tb.tm_mday, tb.tm_hour, tb.tm_min, tb.tm_sec);
	}
}

int map(int val,int mmin,int mmax,int amin,int amax){
	if(val < mmin)val = mmin;
	if(val > mmax)val = mmax;
	float ratio = (float)(amax-amin)/(float)(mmax-mmin);
	return (int)(ratio*val);
}
