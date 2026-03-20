#ifndef _H_SYS_TIME_H
#define _H_SYS_TIME_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
int64_t get_timestamp_us();
int64_t get_timestamp_ms();
#ifdef __cplusplus
}
#endif
#endif