#include "time.h"

#include <sys/time.h>

#include "esp_log.h"
#include "esp_timer.h"
int64_t get_timestamp_us() {
  // struct timeval tv_now;
  // gettimeofday(&tv_now, NULL);
  // int64_t time_us = (int64_t) tv_now.tv_sec * 1000000L + (int64_t) tv_now.tv_usec;
  // return time_us;
  return esp_timer_get_time();
}
int64_t get_timestamp_ms() {
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  int64_t time_ms = (int64_t) tv_now.tv_sec * 1000L + (int64_t) tv_now.tv_usec / 1000;
  return time_ms;
}