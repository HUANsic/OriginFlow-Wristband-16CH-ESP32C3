/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _H_SENSOR_GPIO_IF_H_
#define _H_SENSOR_GPIO_IF_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GPIO drive mode
 */
typedef enum {
  SENSOR_GPIO_MODE_FLOAT,                /*!< Float */
  SENSOR_GPIO_MODE_PULL_UP = (1 << 0),   /*!< Internally pullup */
  SENSOR_GPIO_MODE_PULL_DOWN = (1 << 1), /*!< Internally pulldown */
} sensor_gpio_mode_t;

/**
 * @brief GPIO direction type
 */
typedef enum {
  SENSOR_GPIO_DIR_OUT, /*!< Output GPIO */
  SENSOR_GPIO_DIR_IN,  /*!< Input GPIO */
} sensor_gpio_dir_t;

/**
 * @brief Codec GPIO interface structure
 */
typedef struct {
  int (*setup)(int16_t gpio, sensor_gpio_dir_t dir, sensor_gpio_mode_t mode); /*!< Setup GPIO */
  int (*set)(int16_t gpio, bool high);                                        /*!< Set GPIO level */
  bool (*get)(int16_t gpio);                                                  /*!< Get GPIO level */
} sensor_gpio_if_t;

/**
 * @brief         Delete GPIO interface instance
 * @param         gpio_if: GPIO interface
 * @return        ESP_CODEC_DEV_OK: Delete success
 *                ESP_CODEC_DEV_INVALID_ARG: Input is NULL pointer
 */
int sensor_delete_gpio_if(const sensor_gpio_if_t *gpio_if);

#ifdef __cplusplus
}
#endif

#endif
