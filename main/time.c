#include "time.h"

#include <sys/time.h>

#include "esp_log.h"
int64_t get_timestamp_us() {
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  int64_t time_us = (int64_t) tv_now.tv_sec * 1000000L + (int64_t) tv_now.tv_usec;
  ESP_LOGI("time", "time us: %" PRId64, time_us);
  return time_us;
}
int64_t get_timestamp_ms() {
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  int64_t time_ms = (int64_t) tv_now.tv_sec * 1000L + (int64_t) tv_now.tv_usec / 1000;
  ESP_LOGI("time", "time ms: %" PRId64, time_ms);
  return time_ms;
}