#include "time.h"

#include <sys/time.h>
int64_t get_timestamp() {
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  int64_t time_us = (int64_t) tv_now.tv_sec * 1000000L + (int64_t) tv_now.tv_usec;
  return time_us;
}