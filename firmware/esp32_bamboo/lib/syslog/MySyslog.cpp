#include <Arduino.h>
#include "config.h"

#ifdef USE_SYSLOG
#include <WiFiUdp.h>
#include <Syslog.h>
WiFiUDP udpClient;
Syslog syslogv(udpClient, SYSLOG_SERVER, SYSLOG_PORT, DEVICE_HOSTNAME, APP_NAME, LOG_KERN);
void if (isSyslog) syslog(uint16_t priority, const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  syslogv.vlogf(priority, fmt, args);
  va_end(args);
};
#else

#define LOG_EMERG 0 /* system is unusable */
#define LOG_ALERT 1 /* action must be taken immediately */
#define LOG_CRIT  2 /* critical conditions */
#define LOG_ERR   3 /* error conditions */
#define LOG_WARNING 4 /* warning conditions */
#define LOG_NOTICE  5 /* normal but significant condition */
#define LOG_INFO  6 /* informational (default) */
#define LOG_DEBUG 7 /* debug-level messages */

//#define log_xx(format, ...) printf("[E][%s:%u]: ",pathToFileName(__FILE__), __LINE__)

#ifndef LOG_LEVEL 
  #define LOG_LEVEL LOG_INFO 
#endif

void syslog(uint16_t priority, const char *fmt, ...) {
#ifdef ENABLE_SYSLOG
  if (priority > LOG_LEVEL) return;

  if (priority == LOG_ERR) {
    Serial.printf("[E] ");
  } else if (priority == LOG_WARNING) {
    Serial.printf("[W] ");
  } else if (priority == LOG_INFO) {
    Serial.printf("[I] ");
  } else if (priority == LOG_DEBUG) {
    Serial.printf("[D] ");
  }
  va_list args;
  va_start(args, fmt);
  Serial.printf(fmt, args);
  va_end(args);
#endif //  ENABLE_SYSLOG
}
#endif
