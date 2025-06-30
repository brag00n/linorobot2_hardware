#ifndef SYSLOG_H_
#define SYSLOG_H_ 

const bool isSyslog=true;

#include <Syslog.h>
#include "config.h"

#ifdef USE_SYSLOG
void syslog(uint16_t priority, const char *fmt, ...);
#else
void syslog(uint16_t priority, const char *fmt, ...);
#endif

#endif
