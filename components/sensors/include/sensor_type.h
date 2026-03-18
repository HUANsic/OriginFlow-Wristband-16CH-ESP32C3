/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _SENSOR_TYPES_H_
#define _SENSOR_TYPES_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_VERSION "0.0.1"

/**
 * @brief Define error number of codec device module
 *        Inherit from `esp_err_t`
 */
#define SENSOR_OK                         (0)
#define SENSOR_DRV_ERR                    (ESP_FAIL)
#define SENSOR_INVALID_ARG                (ESP_ERR_INVALID_ARG)
#define SENSOR_NO_MEM                     (ESP_ERR_NO_MEM)
#define SENSOR_NOT_SUPPORT                (ESP_ERR_NOT_SUPPORTED)
#define SENSOR_NOT_FOUND                  (ESP_ERR_NOT_FOUND)
#define SENSOR_WRONG_STATE                (ESP_ERR_INVALID_STATE)
#define SENSOR_WRITE_FAIL                 (0x10D)
#define SENSOR_READ_FAIL                  (0x10E)

#define SENSOR_MAKE_CHANNEL_MASK(channel) ((uint16_t) 1 << (channel))

/**
 * @brief Codec Device type
 */
typedef enum {
  SENSOR_TYPE_NONE,
  SENSOR_TYPE_IN = (1 << 0),                               /*!< Codec input device like ADC (capture data from microphone) */
  SENSOR_TYPE_OUT = (1 << 1),                              /*!< Codec output device like DAC (output analog signal to speaker) */
  SENSOR_TYPE_IN_OUT = (SENSOR_TYPE_IN | SENSOR_TYPE_OUT), /*!< Codec input and output device */
} sensor_type_t;

/**
 * @brief Codec working mode
 */
typedef enum {
  SENSOR_WORK_MODE_NONE,
  SENSOR_WORK_MODE_ADC = (1 << 0),                                       /*!< Enable ADC, only support input */
  SENSOR_WORK_MODE_DAC = (1 << 1),                                       /*!< Enable DAC, only support output */
  SENSOR_WORK_MODE_BOTH = (SENSOR_WORK_MODE_ADC | SENSOR_WORK_MODE_DAC), /*!< Support both DAC and ADC */
  SENSOR_WORK_MODE_LINE = (1 << 2),                                      /*!< Line mode */
} sensor_work_mode_t;

#ifdef __cplusplus
}
#endif

#endif
