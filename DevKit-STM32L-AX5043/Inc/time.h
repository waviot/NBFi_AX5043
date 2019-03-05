//
// time.h
//
// Time routines
//
// Copyright (C) 2002 Michael Ringgaard. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the project nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
// OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
// OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGE.
//

#ifndef TIME_H
#define TIME_H

#include "stdio.h"

#define CLOCKS_PER_SEC  20000000

typedef uint32_t time_t;
//typedef uint8_t size_t;
typedef uint32_t clock_t;

struct tm {
  int tm_sec;                   // Seconds after the minute [0, 59]
  int tm_min;                   // Minutes after the hour [0, 59]
  int tm_hour;                  // Hours since midnight [0, 23]
  int tm_mday;                  // Day of the month [1, 31]
  int tm_mon;                   // Months since January [0, 11]
  int tm_year;                  // Years since 1900
  int tm_wday;                  // Days since Sunday [0, 6]
  int tm_yday;                  // Days since January 1 [0, 365]
  int tm_isdst;                 // Daylight Saving Time flag
  int tm_gmtoff;                // Seconds east of UTC
//  char *tm_zone;                // Timezone abbreviation
};

struct timeval {
  long tv_sec;                  // Seconds
  long tv_usec;                 // Microseconds
};

struct timespec {
  long tv_sec;
  long tv_nsec;
};

struct timezone {
  int tz_minuteswest;           // Minutes west of Greenwich
  int tz_dsttime;               // Type of daylight saving correction
};

extern int _daylight;     // Non-zero if daylight savings time is used
extern long _dstbias;     // Offset for Daylight Saving Time
extern long _timezone;    // Difference in seconds between GMT and local time
//extern const char __code *_tzname[2];  // Standard/daylight savings time zone names

extern const char *_days[];
extern const char *_days_abbrev[];
extern const char *_months[];
extern const char *_months_abbrev[];

#define difftime(time2, time1) ((time2) - (time1))

struct tm *gmtime_r(time_t *timer, struct tm *tmbuf);
struct tm *localtime_r(time_t *timer, struct tm *tmbuf);

struct tm *gmtime(time_t *timer);
struct tm *localtime(time_t *timer);

time_t mktime(struct tm *tmbuf);

size_t strftime(char *s, size_t maxsize, const char *format, const struct tm *t);

clock_t clock();
time_t time(time_t *timeptr);

char *asctime_r(const struct tm *tm, char *buf);
char *ctime_r(const time_t *timer, char *buf);

char *asctime(const struct tm *tm);
char *ctime(const time_t *timer);

char *_strdate(char *s);
char *_strtime(char *s);
void _tzset();

int nanosleep(const struct timespec *req, struct timespec *rem);

void RTC_Init();
time_t RTC_Time();

#endif

