#include "main.h"

#include <driver/gpio.h>

#include "bluetooth.h"
#include "config.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "gatts.h"
#include "imu.h"
#include "nvs_flash.h"
#include "packet_generate.hpp"
#include "protocol.h"
#include "sEMG.h"
#include "sensor.h"
#include "time.h"


static const char *TAG = "app_main";
static void ble_data_cb(uint8_t *data, uint32_t len) {
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, len, ESP_LOG_INFO);
}

uint8_t usb_rx[BUF_SIZE];
uint8_t packet_rx[BUF_SIZE];

extern "C" void app_main(void) {
  // Initialize NVS.
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  sensor_init();

  bluetooth_init();
  ble_gatt_set_cb(ble_data_cb);
  set_protocol_get_timestamp(get_timestamp_us);
#if SEND_DATA_UART == 1
  esp_log_level_set("*", ESP_LOG_NONE);
  // Configure USB SERIAL JTAG
  usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
      .tx_buffer_size = BUF_SIZE,
      .rx_buffer_size = BUF_SIZE,
  };

  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
#endif

  sEMG_init();
  imu_init();
  PROTOCODEC::Codec recv = PROTOCODEC::Codec();

#if defined(PROTOCOL_DEBUG)
  recv.Printf_format();
#endif
  while (1) {
#if SEND_DATA_UART == 1
    int len = usb_serial_jtag_read_bytes(usb_rx, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
    if (len) {
      ESP_LOG_BUFFER_HEX_LEVEL(TAG, usb_rx, len, ESP_LOG_INFO);
      dt_err_t err = recv.Parse_from_data(usb_rx, len, packet_rx);
      if (err != dt_ok) {
        ESP_LOGE(TAG, "dt parse failure is %d", err);
      } else {
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, packet_rx, recv.Len(), ESP_LOG_DEBUG);
        switch (recv.dt.Cmd) {
          case dt_cmd_data_start:
            g_sys_config.enable_send_data = true;
            ESP_LOGI(TAG, "g_sys_config.enable_send_data is %s", g_sys_config.enable_send_data ? "true" : "false");
            break;
          case dt_cmd_data_end:
            g_sys_config.enable_send_data = false;
            ESP_LOGI(TAG, "g_sys_config.enable_send_data is %s", g_sys_config.enable_send_data ? "true" : "false");
            break;
          case dt_cmd_imu:
            if (recv.Len() == DT_FIXED_LEN + 2) {
              if (recv.Data()[0] == dt_imu_ctrl_data_type) {
                g_sys_config.enable_imu_raw = recv.Data()[1];
                ESP_LOGI(TAG, "g_sys_config.enable_imu_raw is %s", g_sys_config.enable_imu_raw ? "true" : "false");
              }
            }
            break;
          case dt_cmd_log:
            if (recv.Len() == DT_FIXED_LEN + 1) {
              int log_level = recv.Data()[0];
              if (log_level >= ESP_LOG_NONE && log_level < ESP_LOG_MAX) {
                esp_log_level_set("*", (esp_log_level_t) log_level);
              }
              ESP_LOGI(TAG, "esp_log_level is %d", log_level);
            }
            break;
          default:
            break;
        }
      }
    }
#else
    vTaskDelay(1000 / portTICK_PERIOD_MS);
#endif
  }
}
