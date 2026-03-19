/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "sensor_ctrl_if.h"
#include "sensor_default.h"

#define TAG "SPI_If"
static volatile bool spi_is_open = false;
typedef struct {
  sensor_ctrl_if_t base;
  bool is_open;
  uint8_t port;
  void (*enable)();  /* SPI CS 0 */
  void (*disable)(); /* SPI CS 1 */
  spi_device_handle_t spi_handle;
} spi_ctrl_t;
spi_device_handle_t s_spi_handle;

int _spi_ctrl_open(const sensor_ctrl_if_t *ctrl, void *cfg, int cfg_size) {
  if (ctrl == NULL || cfg == NULL) {
    return SENSOR_INVALID_ARG;
  }
  if (cfg_size != sizeof(sensor_spi_cfg_t)) {
    return SENSOR_INVALID_ARG;
  }
  spi_ctrl_t *spi_ctrl = (spi_ctrl_t *) ctrl;
  sensor_spi_cfg_t *spi_cfg = (sensor_spi_cfg_t *) cfg;
  int speed = spi_cfg->clock_speed ? spi_cfg->clock_speed : 1000000;
  spi_device_interface_config_t dev_cfg = {
      .clock_speed_hz = speed,  // Clock out at 10 MHz
      .command_bits = 0,        // no command phase, only data
      .address_bits = 0,        // no address phase, only data
      .dummy_bits = 0,          // no dummy phase
      .mode = 0,                // SPI mode 0
      .queue_size = 6,          // queue 7 transactions at a time
  };
  dev_cfg.spics_io_num = -1;
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 4))
  spi_host_device_t host_id = SPI2_HOST;
#elif (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0))
  spi_host_device_t host_id = SPI3_HOST;
#else
  spi_host_device_t host_id = HSPI_HOST;
#endif
  int ret = spi_bus_add_device(host_id, &dev_cfg, &s_spi_handle);
  spi_ctrl->is_open = true;
  spi_ctrl->spi_handle = s_spi_handle;
  spi_is_open = true;
  ESP_LOGI(TAG, "%s success", __func__);
  return ret == 0 ? SENSOR_OK : SENSOR_DRV_ERR;
}

bool _spi_ctrl_is_open(const sensor_ctrl_if_t *ctrl) {
  if (ctrl) {
    spi_ctrl_t *spi_ctrl = (spi_ctrl_t *) ctrl;
    return spi_ctrl->is_open;
  }
  return false;
}
void _spi_ctrl_enable(const spi_ctrl_t *spi_ctrl) {
  if (spi_ctrl && spi_ctrl->enable) {
    spi_ctrl->enable();
  } else {
    ESP_LOGE(TAG, "%s SPI CS not set", __func__);
  }
}
void _spi_ctrl_disable(const spi_ctrl_t *spi_ctrl) {
  if (spi_ctrl && spi_ctrl->disable) {
    spi_ctrl->disable();
  } else {
    ESP_LOGE(TAG, "%s SPI CS not set", __func__);
  }
}

static int _spi_ctrl_read_reg(const sensor_ctrl_if_t *ctrl, int addr, int addr_len, void *data, int data_len) {
  if (ctrl == NULL) {
    return SENSOR_INVALID_ARG;
  }
  spi_ctrl_t *spi_ctrl = (spi_ctrl_t *) ctrl;
  if (spi_ctrl->is_open == false) {
    return SENSOR_WRONG_STATE;
  }
  esp_err_t ret = ESP_OK;
  _spi_ctrl_enable(spi_ctrl);
  if (addr_len) {
    uint16_t *v = (uint16_t *) &addr;
    while (addr_len >= 2) {
      spi_transaction_t t = {0};
      t.length = 2 * 8;
      t.tx_buffer = v;
      ret = spi_device_transmit(spi_ctrl->spi_handle, &t);
      v++;
      addr_len -= 2;
    }
  }
  if (data_len) {
    spi_transaction_t t = {0};
    t.length = data_len * 8;
    t.rxlength = data_len * 8;
    t.rx_buffer = data;
    ret = spi_device_transmit(spi_ctrl->spi_handle, &t);
  }
  _spi_ctrl_disable(spi_ctrl);
  return ret == ESP_OK ? SENSOR_OK : SENSOR_DRV_ERR;
}

static int _spi_ctrl_write_reg(const sensor_ctrl_if_t *ctrl, int addr, int addr_len, void *data, int data_len) {
  spi_ctrl_t *spi_ctrl = (spi_ctrl_t *) ctrl;
  if (ctrl == NULL) {
    return SENSOR_INVALID_ARG;
  }
  if (spi_ctrl->is_open == false) {
    return SENSOR_WRONG_STATE;
  }
  esp_err_t ret = ESP_OK;
  _spi_ctrl_enable(spi_ctrl);
  if (addr_len) {
    uint16_t *v = (uint16_t *) &addr;
    while (addr_len >= 2) {
      spi_transaction_t t = {0};
      t.length = 2 * 8;
      t.tx_buffer = v;
      ret = spi_device_transmit(spi_ctrl->spi_handle, &t);
      v++;
      addr_len -= 2;
    }
  }
  if (data_len) {
    spi_transaction_t t = {0};
    t.length = data_len * 8;
    t.tx_buffer = data;
    ret = spi_device_transmit(spi_ctrl->spi_handle, &t);
  }
  _spi_ctrl_disable(spi_ctrl);
  return ret == ESP_OK ? SENSOR_OK : SENSOR_DRV_ERR;
}

static int _spi_ctrl_write_read_reg(const sensor_ctrl_if_t *ctrl, void *tx_data, int tx_len, void *rx_data, int rx_len) {
  spi_ctrl_t *spi_ctrl = (spi_ctrl_t *) ctrl;
  if (ctrl == NULL || tx_data == NULL || rx_data == NULL || (tx_len != rx_len)) {
    ESP_LOGE(TAG, "%s, Invalid argument", __func__);
    return SENSOR_INVALID_ARG;
  }
  if (spi_ctrl->is_open == false) {
    ESP_LOGE(TAG, "%s, SPI not open", __func__);
    return SENSOR_WRONG_STATE;
  }
  _spi_ctrl_enable(spi_ctrl);
  spi_transaction_t trans;
  memset(&trans, 0, sizeof(spi_transaction_t));
  trans.cmd = 0;
  trans.tx_buffer = tx_data;
  trans.length = (8 * tx_len);
  trans.rx_buffer = rx_data;
  trans.rxlength = (8 * rx_len);
  esp_err_t ret = spi_device_transmit(spi_ctrl->spi_handle, &trans);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "%s: Fail to write to dev %s", __func__, esp_err_to_name(ret));
  }

  _spi_ctrl_disable(spi_ctrl);
  return ret == ESP_OK ? SENSOR_OK : SENSOR_DRV_ERR;
}

int _spi_ctrl_close(const sensor_ctrl_if_t *ctrl) {
  spi_ctrl_t *spi_ctrl = (spi_ctrl_t *) ctrl;
  if (ctrl == NULL) {
    return SENSOR_INVALID_ARG;
  }
  int ret = 0;
  if (spi_ctrl->spi_handle) {
    ret = spi_bus_remove_device(spi_ctrl->spi_handle);
  }
  spi_ctrl->is_open = false;
  return ret == ESP_OK ? SENSOR_OK : SENSOR_DRV_ERR;
}

sensor_ctrl_if_t *sensor_new_spi_ctrl(sensor_spi_cfg_t *spi_cfg) {
  spi_ctrl_t *ctrl = calloc(1, sizeof(spi_ctrl_t));
  if (ctrl == NULL) {
    ESP_LOGE(TAG, "No memory for instance");
    return NULL;
  }
  ctrl->base.open = _spi_ctrl_open;
  ctrl->base.is_open = _spi_ctrl_is_open;
  ctrl->base.read_reg = _spi_ctrl_read_reg;
  ctrl->base.write_reg = _spi_ctrl_write_reg;
  ctrl->base.write_read_reg = _spi_ctrl_write_read_reg;
  ctrl->base.close = _spi_ctrl_close;
  ctrl->enable = spi_cfg->enable;
  ctrl->disable = spi_cfg->disable;
  if (!spi_is_open) {
    int ret = _spi_ctrl_open(&ctrl->base, spi_cfg, sizeof(sensor_spi_cfg_t));
    if (ret != 0) {
      ESP_LOGE(TAG, "Fail to open SPI driver");
      free(ctrl);
      return NULL;
    }
  } else {
    ctrl->spi_handle = s_spi_handle;
    ctrl->is_open = true;
  }
  return &ctrl->base;
}
