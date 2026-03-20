#ifndef _H_SENSOR_CTRL_IF_H_
#define _H_SENSOR_CTRL_IF_H_

#include "sensor_type.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct sensor_ctrl_if_t sensor_ctrl_if_t;

/**
 * @brief sensor control interface structure
 */
struct sensor_ctrl_if_t {
  int (*open)(const sensor_ctrl_if_t *ctrl, void *cfg, int cfg_size);                             /*!< Open sensor control interface */
  bool (*is_open)(const sensor_ctrl_if_t *ctrl);                                                  /*!< Check whether sensor control opened or not */
  int (*read_reg)(const sensor_ctrl_if_t *ctrl, int reg, int reg_len, void *data, int data_len);  /*!< Read data from sensor register */
  int (*write_reg)(const sensor_ctrl_if_t *ctrl, int reg, int reg_len, void *data, int data_len); /*!< Write data to sensor register */
  int (*write_read_reg)(const sensor_ctrl_if_t *ctrl, void *tx_data, int tx_len, void *rx_data,
                        int rx_len); /*!< Write data and read data to sensor register. tx_len = rx_len */
  void (*enable)(const sensor_ctrl_if_t *ctrl);
  void (*disable)(const sensor_ctrl_if_t *ctrl);
  int (*close)(const sensor_ctrl_if_t *ctrl); /*!< Close sensor control interface */
};

/**
 * @brief         Delete sensor control interface instance
 * @param         ctrl_if: Audio sensor interface
 * @return        ESP_sensor_DEV_OK: Delete success
 *                ESP_sensor_DEV_INVALID_ARG: Input is NULL pointer
 */
int sensor_delete_ctrl_if(const sensor_ctrl_if_t *ctrl_if);

#ifdef __cplusplus
}
#endif

#endif
