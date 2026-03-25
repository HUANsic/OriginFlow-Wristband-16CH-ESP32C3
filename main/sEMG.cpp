#include "sEMG.h"

#include <driver/gptimer.h>
#include <string.h>

#include "ad7689.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "packet_generate.hpp"
#include "rom/ets_sys.h"
#include "sensor.h"
#include "sensor_default.h"
#include "sensor_if.h"
#include "soc/gpio_num.h"
#include "time.h"
// AD7689配置寄存器
static TaskHandle_t s_semg_task_handle = NULL;
static sensor_if_t *s_ad7689[2];
static const char *TAG = "sEMG";
#if ENABLE_AD7689
void ad7689_1_enable() {
  gpio_set_level((gpio_num_t) ADC1_CS_GPIO_NUM, 1);
  ets_delay_us(3);  // Delay for 10 microseconds before the next reading
  gpio_set_level((gpio_num_t) ADC1_CS_GPIO_NUM, 0);
}

void ad7689_1_disable() {
  gpio_set_level((gpio_num_t) ADC1_CS_GPIO_NUM, 1);
}
void ad7689_2_enable() {
  gpio_set_level((gpio_num_t) ADC2_CS_GPIO_NUM, 1);
  ets_delay_us(3);  // Delay for 10 microseconds before the next reading
  gpio_set_level((gpio_num_t) ADC2_CS_GPIO_NUM, 0);
}

void ad7689_2_disable() {
  gpio_set_level((gpio_num_t) ADC2_CS_GPIO_NUM, 1);
}

const sensor_if_t *init_ad7689_1() {
  static sensor_spi_cfg_t sensor_spi_cfg = {SENSORS_SPI_HOST, SPI_MASTER_FREQ_26M, ad7689_1_enable, ad7689_1_disable};
  static sensor_ctrl_if_t *out_ctrl_if = sensor_new_spi_ctrl(&sensor_spi_cfg);
  ad7689_cfg_t ad7689_cfg;
  memset(&ad7689_cfg, 0, sizeof(ad7689_cfg_t));
  ad7689_cfg.ctrl_if = out_ctrl_if;
  ad7689_cfg.reg_cfg.bw = 1;
  ad7689_cfg.reg_cfg.cfg = 1;
  ad7689_cfg.reg_cfg.seq = 0;
  // ad7689_cfg.reg_cfg.ref = 0b111;
  ad7689_cfg.reg_cfg.ref = 0;
  ad7689_cfg.reg_cfg.innc = 0b111;
  ad7689_cfg.reg_cfg.inx = 7;
  ad7689_cfg.reg_cfg.rb = 0;

  const sensor_if_t *_ad7689_if = ad7689_new(&ad7689_cfg);
  ESP_LOGI(TAG, "init s_ad7689 1 is success");

  return _ad7689_if;
}

const sensor_if_t *init_ad7689_2() {
  static sensor_spi_cfg_t sensor_spi_cfg = {SENSORS_SPI_HOST, SPI_MASTER_FREQ_26M, ad7689_2_enable, ad7689_2_disable};
  static sensor_ctrl_if_t *out_ctrl_if = sensor_new_spi_ctrl(&sensor_spi_cfg);
  ad7689_cfg_t ad7689_cfg;
  memset(&ad7689_cfg, 0, sizeof(ad7689_cfg_t));
  ad7689_cfg.ctrl_if = out_ctrl_if;
  ad7689_cfg.reg_cfg.bw = 1;
  ad7689_cfg.reg_cfg.cfg = 1;
  ad7689_cfg.reg_cfg.seq = 0;
  ad7689_cfg.reg_cfg.ref = 0b111;
  // ad7689_cfg.reg_cfg.ref = 0;
  ad7689_cfg.reg_cfg.innc = 0b111;
  ad7689_cfg.reg_cfg.inx = 0;

  const sensor_if_t *_ad7689_if = ad7689_new(&ad7689_cfg);
  uint16_t dummy;
  _ad7689_if->read_channel_data(_ad7689_if, 7, &dummy);
  vTaskDelay(1 / portTICK_PERIOD_MS);
  _ad7689_if->write_read_reg(_ad7689_if, 0, 0, &dummy);
  gpio_set_level((gpio_num_t) ADC2_CS_GPIO_NUM, 1);
  return _ad7689_if;
}
#endif

void ad_timer_callback(void *arg) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (s_semg_task_handle != NULL) {
    vTaskNotifyGiveFromISR(s_semg_task_handle, &xHigherPriorityTaskWoken);
  }

  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}
bool ad_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (s_semg_task_handle != NULL) {
    vTaskNotifyGiveFromISR(s_semg_task_handle, &xHigherPriorityTaskWoken);
  }

  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
  return true;
}

void sEMG_task(void *param) {
  uint16_t values[2][8];
  float voltage[2][8];
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 8; j++) {
      values[i][j] = 0;
      voltage[i][j] = 0;
    }
  }
  PacketGenerate _generate;
  uint8_t semg_data[48] = {0};
  while (1) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    for (int idx = 0; idx < 1; idx++) {
      s_ad7689[idx]->read_all_channel_data(s_ad7689[idx], (void *) values[idx]);
      for (int i = 0; i < 8; i++) {
        voltage[idx][i] = ((values[idx][i] - AD7689_BASE_V) / 65536.0) * 2.5 / 0.35;
        semg_data[idx * 24 + i * 3] = values[idx][i] & 0xFF;
        semg_data[idx * 24 + i * 3 + 1] = (values[idx][i] >> 8) & 0xFF;
        semg_data[idx * 24 + i * 3 + 2] = 0;
      }
    }
    for (int idx = 0; idx < 1; idx++) {
      s_ad7689[idx]->read_all_channel_data(s_ad7689[idx], (void *) values[idx]);
      for (int i = 0; i < 8; i++) {
        // ys测试板
        // if (values[idx][i] > AD7689_BASE_V) {
        //   values[idx][i] -= AD7689_BASE_V;
        // }
        // voltage[idx][i] = ((values[idx][i] - AD7689_BASE_V) / 65536.0) * 2.5 / 0.35;
        voltage[idx][i] = (values[idx][i] / 65536.0) * 2.5 / 0.35;
        semg_data[idx * 24 + i * 3] = values[idx][i] & 0xFF;
        semg_data[idx * 24 + i * 3 + 1] = (values[idx][i] >> 8) & 0xFF;
        semg_data[idx * 24 + i * 3 + 2] = 0;
      }
    }
    if (g_sys_config.enable_send_data) {
      _generate.packet_generate_semg(semg_data, 48);
#if SEND_DATA_BLUE == 1
      ble_tx_rx_send_notify((uint8_t *) _generate.data(), _generate.length());
#endif
#if SEND_DATA_UART == 1
      usb_serial_jtag_write_bytes((uint8_t *) _generate.data(), _generate.length(), 1 / portTICK_PERIOD_MS);
#endif
    }
    // 统计fps
    static uint32_t frame_count = 0;
    static int64_t fps_start_time = 0;
    const int64_t FPS_UPDATE_INTERVAL_US = 1000000;  // Update FPS every 1 second
    static float current_fps = 0.0f;
    int64_t end_time = get_timestamp_us();
    if (frame_count == 0 && fps_start_time == 0) {
      fps_start_time = end_time;
    }
    frame_count++;
    int64_t elapsed_time = end_time - fps_start_time;
    if (elapsed_time >= FPS_UPDATE_INTERVAL_US) {
      current_fps = (float) frame_count * 1000000.0f / (float) elapsed_time;
      ESP_LOGI("FPS", "Current ad FPS: %.2f,%d", current_fps, frame_count);
      frame_count = 0;
      fps_start_time = end_time;
    }
#if WORK_MODE == WORK_MODE_DEBUG
    ESP_LOGI("AD READ", "ADC[1] values: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f ADC[2] values: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f", voltage[0][0],
             voltage[0][1], voltage[0][2], voltage[0][3], voltage[0][4], voltage[0][5], voltage[0][6], voltage[0][7], voltage[1][0], voltage[1][1],
             voltage[1][2], voltage[1][3], voltage[1][4], voltage[1][5], voltage[1][6], voltage[1][7]);
#endif
  }
}

int sEMG_init() {
  s_ad7689[0] = (sensor_if_t *) init_ad7689_1();
  //   s_ad7689[1] = (sensor_if_t *) init_ad7689_2();
  xTaskCreate(sEMG_task, "semg_task", 4096, NULL, 5, &s_semg_task_handle);
  // const esp_timer_create_args_t periodic_timer_args = {.callback = &ad_timer_callback, .dispatch_method = ESP_TIMER_ISR, .name = "semg_timer"};

  // esp_timer_handle_t periodic_timer;
  // ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
  // ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 500));

  gptimer_handle_t gptimer = NULL;
  gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,  // 选择默认的时钟源
      .direction = GPTIMER_COUNT_UP,       // 计数方向为向上计数
      .resolution_hz = 1 * 1000 * 1000,    // 分辨率为 1 MHz，即 1 次滴答为 1 微秒
  };
  // 创建定时器实例
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
  gptimer_alarm_config_t alarm_config = {
      .alarm_count = 500,
      .reload_count = 0,  // 当警报事件发生时，定时器会自动重载到 0
                          // .flags.auto_reload_on_alarm = true, // 使能自动重载功能
  };
  alarm_config.flags.auto_reload_on_alarm = true;
  // 设置定时器的警报动作
  ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

  gptimer_event_callbacks_t cbs = {
      .on_alarm = ad_alarm_cb,
  };
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
  // 使能定时器
  ESP_ERROR_CHECK(gptimer_enable(gptimer));
  // 启动定时器
  ESP_ERROR_CHECK(gptimer_start(gptimer));

  ESP_LOGI(TAG, "sEMG init success");
  return 0;
}