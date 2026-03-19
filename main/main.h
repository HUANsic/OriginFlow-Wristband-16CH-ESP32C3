#ifndef _H_MAIN_H
#define _H_MAIN_H

#define ENABLE_IMU        1
#define ENABLE_AD7689     1
#define BUF_SIZE          (1024)
#define AD7689_BASE_V     (0xFF0)
#define SEND_DATA_BLUE    0
#define SEND_DATA_UART    1

#define WORK_MODE_DEBUG   0
#define WORK_MODE_RELEASE 1

#ifndef WORK_MODE
#define WORK_MODE WORK_MODE_RELEASE
#endif

#endif