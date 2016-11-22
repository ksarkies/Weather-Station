/* Martin Thomas 4/2009 */

#include "integer.h"
#include "fattime.h"
#include "../libs/hardware.h"
#include <time.h>

/* This provides a 32 bit integer construct of time based on years since 1980.
Seconds are in 2 second increments. The tm struct gives years since 1900. */
DWORD get_fattime (void)
{
	DWORD res;
	time_t currentTime = (time_t)getTimeCounter();
    struct tm *rtc = localtime(&currentTime);

	res =  (((DWORD)rtc->tm_year - 80) << 25)
			| ((DWORD)rtc->tm_mon << 21)
			| ((DWORD)rtc->tm_mday << 16)
			| (WORD)(rtc->tm_hour << 11)
			| (WORD)(rtc->tm_min << 5)
			| (WORD)(rtc->tm_sec >> 1);

	return res;
}

