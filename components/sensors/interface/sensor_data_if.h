/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _H_SENSOR_DATA_IF_H_
#define _H_SENSOR_DATA_IF_H_

#include "sensor_type.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct sensor_data_if_t sensor_data_if_t;

/**
 * @brief Audio codec data interface structure
 */
struct sensor_data_if_t {
  int (*open)(const sensor_data_if_t *h, void *data_cfg, int cfg_size);                        /*!< Open data interface */
  bool (*is_open)(const sensor_data_if_t *h);                                                  /*!< Check whether data interface is opened */
  int (*enable)(const sensor_data_if_t *h, sensor_type_t dev_type, bool enable);               /*!< Enable input or output channel */
  int (*read)(const sensor_data_if_t *h, uint8_t *data, int size);                             /*!< Read data from data interface */
  int (*write)(const sensor_data_if_t *h, uint8_t *data, int size);                            /*!< Write data to data interface */
  int (*close)(const sensor_data_if_t *h);                                                     /*!< Close data interface */
};

/**
 * @brief         Delete codec data interface instance
 * @param         data_if: Codec data interface
 * @return        ESP_CODEC_DEV_OK: Delete success
 *                ESP_CODEC_DEV_INVALID_ARG: Input is NULL pointer
 */
int sensor_delete_data_if(const sensor_data_if_t *data_if);

#ifdef __cplusplus
}
#endif

#endif
