#ifndef _H_CONFIG_H
#define _H_CONFIG_H

#define ENABLE_IMU        0
#define ENABLE_AD7689     1
#define BUF_SIZE          (1024)
#define AD7689_BASE_V     (0xFF0)
#define SEND_DATA_BLUE    0
#define SEND_DATA_UART    1

#define WORK_MODE_DEBUG   0
#define WORK_MODE_RELEASE 1

#ifndef WORK_MODE
// #define WORK_MODE WORK_MODE_DEBUG
#define WORK_MODE WORK_MODE_RELEASE
#endif

#include <stdbool.h>
typedef struct {
  bool enable_send_data;  // enable/disable sned data
  bool enable_imu_raw;    // enable imu raw data /enable imu float data
} sys_config_t;
extern sys_config_t g_sys_config;
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif