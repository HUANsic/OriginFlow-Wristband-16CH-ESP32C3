/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _H_SENSOR_IF_H_
#define _H_SENSOR_IF_H_

#include "sensor_type.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct sensor_if_t sensor_if_t;

/**
 * @brief Structure for codec interface
 */
struct sensor_if_t {
  int (*set_reg)(const sensor_if_t *h, int reg, int value);                        /*!< Set register value to codec */
  int (*get_reg)(const sensor_if_t *h, int reg, int *value);                       /*!< Get register value from codec */
  int (*write_read_reg)(const sensor_if_t *h, int reg, int value, void *out_data); /*!< Get register value from codec */
  int (*read_channel_data)(const sensor_if_t *h, int channel, void *data);         /*!< Read data from codec */
  int (*read_all_channel_data)(const sensor_if_t *h, void *data);                  /*!< Read data from codec */
  void (*dump_reg)(const sensor_if_t *h);                                          /*!< Dump all register settings */
};

/**
 * @brief         Delete codec interface instance
 * @param         codec_if: Codec interface
 * @return        ESP_CODEC_DEV_OK: Delete success
 *                ESP_CODEC_DEV_INVALID_ARG: Input is NULL pointer
 */
int sensor_delete_codec_if(const sensor_if_t *codec_if);

#ifdef __cplusplus
}
#endif

#endif
