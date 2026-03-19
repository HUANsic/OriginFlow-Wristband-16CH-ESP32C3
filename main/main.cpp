#include "main.h"

#include <driver/gpio.h>
#include <inttypes.h>
#include <stdio.h>

#include <map>

#include "LIS3MDL.h"
#include "ad7689.h"
#include "bluetooth.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "gatts.h"
#include "lsm6ds3.h"
#include "nvs_flash.h"
#include "packet_generate.hpp"
#include "protocol.h"
#include "rom/ets_sys.h"
#include "sensor.h"
#include "sensor_ctrl_if.h"
#include "sensor_default.h"
#include "time.h"

static volatile bool s_send_data = false;
static const char *TAG = "app_main";
static void ble_data_cb(uint8_t *data, uint32_t len) {
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, len, ESP_LOG_INFO);
}

const std::map<lsm6ds3tr_c_fs_g_t, const char *> dps_map = {
    {LSM6DS3TR_C_250dps,  "250dps" },
    {LSM6DS3TR_C_125dps,  "125dps" },
    {LSM6DS3TR_C_500dps,  "500dps" },
    {LSM6DS3TR_C_1000dps, "1000dps"},
    {LSM6DS3TR_C_2000dps, "2000dps"},
};
const std::map<lsm6ds3tr_c_fs_xl_t, const char *> xl_map = {
    {LSM6DS3TR_C_2g,  "2g" },
    {LSM6DS3TR_C_4g,  "4g" },
    {LSM6DS3TR_C_8g,  "8g" },
    {LSM6DS3TR_C_16g, "16g"},
};

#define OUT_XYZ_SIZE 6
static stmdev_ctx_t mag_ctx;
static stmdev_ctx_t dev_ctx;
typedef union {
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;
#define LOG_DEBUG 0

// AD7689配置寄存器
extern spi_device_handle_t _sensors_hspidevice;

#if ENABLE_AD7689
void ad7689_1_enable() {
  gpio_set_level((gpio_num_t) ADC1_CS_GPIO_NUM, 1);
  ets_delay_us(2);  // Delay for 10 microseconds before the next reading
  gpio_set_level((gpio_num_t) ADC1_CS_GPIO_NUM, 0);
}

void ad7689_1_disable() {
  gpio_set_level((gpio_num_t) ADC1_CS_GPIO_NUM, 1);
}
void ad7689_2_enable() {
  gpio_set_level((gpio_num_t) ADC2_CS_GPIO_NUM, 1);
  ets_delay_us(2);  // Delay for 10 microseconds before the next reading
  gpio_set_level((gpio_num_t) ADC2_CS_GPIO_NUM, 0);
}

void ad7689_2_disable() {
  gpio_set_level((gpio_num_t) ADC2_CS_GPIO_NUM, 1);
}

const sensor_if_t *init_ad7689_1() {
  static sensor_spi_cfg_t sensor_spi_cfg = {SENSORS_SPI_HOST, 1000000, ad7689_1_enable, ad7689_1_disable};
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
  uint16_t dummy;
  _ad7689_if->read_channel_data(_ad7689_if, 7, &dummy);
  vTaskDelay(1 / portTICK_PERIOD_MS);
  _ad7689_if->write_read_reg(_ad7689_if, 0, 0, &dummy);
  gpio_set_level((gpio_num_t) ADC1_CS_GPIO_NUM, 1);
  ESP_LOGI(TAG, "init ad7689 1 is success");
  return _ad7689_if;
}

const sensor_if_t *init_ad7689_2() {
  static sensor_spi_cfg_t sensor_spi_cfg = {SENSORS_SPI_HOST, 1000000, ad7689_2_enable, ad7689_2_disable};
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
/*
 * Read data byte from internal register of a slave device connected
 * to master I2C interface
 */
static int32_t lsm6ds3tr_c_read_lis3mdl_cx(void *ctx, uint8_t reg, uint8_t *data, uint16_t len) {
  axis3bit16_t data_raw_acceleration;
  int32_t mm_error;
  uint8_t drdy;
  uint8_t i;
  lsm6ds3tr_c_reg_t reg_endop;
  uint8_t sh_reg[18];
  lsm6ds3tr_c_sh_cfg_read_t val = {
      .slv_add = LIS3MDL_I2C_ADD_H,
      .slv_subadd = reg,
      .slv_len = (uint8_t) len,
  };
  (void) ctx;
  /* Configure Sensor Hub to read LIS3MDL */
  mm_error = lsm6ds3tr_c_sh_slv0_cfg_read(&dev_ctx, &val);
#if LOG_DEBUG == 1
  ESP_LOGD("lis3mdl", "lsm6ds3tr_c_sh_slv0_cfg_read: %d", mm_error);
#endif
  lsm6ds3tr_c_sh_num_of_dev_connected_set(&dev_ctx, LSM6DS3TR_C_SLV_0_1);
  lsm6ds3tr_c_func_en_set(&dev_ctx, PROPERTY_ENABLE);
  /* Enable I2C Master and I2C master Pull Up */
  int32_t ret = lsm6ds3tr_c_sh_pin_mode_set(&dev_ctx, LSM6DS3TR_C_INTERNAL_PULL_UP);
#if LOG_DEBUG == 1
  ESP_LOGD("lis3mdl", "lsm6ds3tr_c_sh_pin_mode_set: %d", ret);
#endif
  lsm6ds3tr_c_sh_master_set(&dev_ctx, PROPERTY_ENABLE);
  /* Enable accelerometer to trigger Sensor Hub operation */
  lsm6ds3tr_c_xl_data_rate_set(&dev_ctx, LSM6DS3TR_C_XL_ODR_6k66Hz);
  // /* Wait Sensor Hub operation flag set */
  ret = lsm6ds3tr_c_acceleration_raw_get(&dev_ctx, data_raw_acceleration.i16bit);
#if LOG_DEBUG == 1
  ESP_LOGD(TAG, "lsm6ds3tr_c_acceleration_raw_get: %d", ret);
#endif
  do {
    lsm6ds3tr_c_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

#if LOG_DEBUG == 1
  ESP_LOGD(TAG, "lsm6ds3tr_c_xl_flag_data_ready_get: %d", ret);
#endif
  do {
    imu_read_regs(&dev_ctx, LSM6DS3TR_C_FUNC_SRC1, &reg_endop.byte, 1);
  } while (!reg_endop.func_src1.sensorhub_end_op);

  lsm6ds3tr_c_xl_data_rate_set(&dev_ctx, LSM6DS3TR_C_XL_ODR_OFF);
  lsm6ds3tr_c_sh_read_data_raw_get(&dev_ctx, (lsm6ds3tr_c_emb_sh_read_t *) &sh_reg);
  lsm6ds3tr_c_func_en_set(&dev_ctx, PROPERTY_DISABLE);
  lsm6ds3tr_c_sh_master_set(&dev_ctx, PROPERTY_DISABLE);

  for (i = 0; i < len; i++) {
    data[i] = sh_reg[i];
  }

  return mm_error;
}

/*
 * Write data byte to internal register of a slave device connected
 * to master I2C interface
 */
static int32_t lsm6ds3tr_c_write_lis3mdl_cx(void *ctx, uint8_t reg, const uint8_t *data, uint16_t len) {
  axis3bit16_t data_raw_acceleration;
  int32_t mm_error;
  uint8_t drdy;
  lsm6ds3tr_c_reg_t reg_endop;
  lsm6ds3tr_c_sh_cfg_write_t val = {
      .slv0_add = LIS3MDL_I2C_ADD_H,
      .slv0_subadd = reg,
      .slv0_data = *data,
  };
  (void) ctx;
  (void) len;
  /* Disable accelerometer */
  lsm6ds3tr_c_xl_data_rate_set(&dev_ctx, LSM6DS3TR_C_XL_ODR_OFF);
  /* Configure Sensor Hub to write */
  mm_error = lsm6ds3tr_c_sh_cfg_write(&dev_ctx, &val);
  /* Enable I2C Master and I2C master Pull Up */
  lsm6ds3tr_c_func_en_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6ds3tr_c_sh_master_set(&dev_ctx, PROPERTY_ENABLE);
  /* Enable accelerometer to trigger Sensor Hub operation */
  lsm6ds3tr_c_xl_data_rate_set(&dev_ctx, LSM6DS3TR_C_XL_ODR_6k66Hz);
  /* Wait Sensor Hub operation flag set */
  lsm6ds3tr_c_acceleration_raw_get(&dev_ctx, data_raw_acceleration.i16bit);

  do {
    lsm6ds3tr_c_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  do {
    imu_read_regs(&dev_ctx, LSM6DS3TR_C_FUNC_SRC1, &reg_endop.byte, 1);
  } while (!reg_endop.func_src1.sensorhub_end_op);

  lsm6ds3tr_c_xl_data_rate_set(&dev_ctx, LSM6DS3TR_C_XL_ODR_OFF);
  lsm6ds3tr_c_func_en_set(&dev_ctx, PROPERTY_DISABLE);
  lsm6ds3tr_c_sh_master_set(&dev_ctx, PROPERTY_DISABLE);
  return mm_error;
}
uint8_t slave_address = LSM6DS3TR_C_I2C_ADD_L;
static void platform_delay(uint32_t ms) {
  vTaskDelay(ms / portTICK_PERIOD_MS);
}

#define PI 3.14159265358979323846f

float calculate_heading(float mag_x, float mag_y) {
  // 使用 atan2 自动处理坐标象限，返回值为弧度 (-PI 到 PI)
  float heading = atan2(mag_y, mag_x);

  // 转换为角度 (0 到 360 度)
  float declination = -6.1f;  // 如果知道当地磁偏角（如北京约 -8度），在此处加上
  heading = heading * (180.0f / PI) + declination;

  // 归一化到 0-360 度
  if (heading < 0) {
    heading += 360.0f;
  } else if (heading >= 360.0f) {
    heading -= 360.0f;
  }

  return heading;
}
void sensor_read_task(void *param) {
  uint16_t values[2][8];
  float voltage[2][8];
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 8; j++) {
      values[i][j] = 0;
      voltage[i][j] = 0;
    }
  }

#if ENABLE_IMU == 1
  uint8_t who_am_i;
  uint8_t reg = LSM6DS3TR_C_WHO_AM_I;
  dev_ctx.write_reg = (stmdev_write_ptr) imu_write_regs;
  dev_ctx.read_reg = (stmdev_read_ptr) imu_read_regs;
  dev_ctx.mdelay = platform_delay;
  imu_read_regs(NULL, LSM6DS3TR_C_WHO_AM_I, &who_am_i, 1);
  ESP_LOGI("IMU READ", "LSM6DS3 WHOAMI: 0x%02X (dummy)", who_am_i);
  imu_read_regs(NULL, LSM6DS3TR_C_WHO_AM_I, &who_am_i, 1);
  ESP_LOGI("IMU READ", "LSM6DS3 WHOAMI: 0x%02X", who_am_i);
  lsm6ds3tr_c_sh_cfg_read_t val = {
      .slv_add = LIS3MDL_I2C_ADD_L,
      .slv_subadd = LIS3MDL_OUT_X_L,
      .slv_len = OUT_XYZ_SIZE,
  };

  static uint8_t whoamI, rst;
  /* Restore default configuration */
  lsm6ds3tr_c_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6ds3tr_c_reset_get(&dev_ctx, &rst);
  } while (rst);

  mag_ctx.read_reg = lsm6ds3tr_c_read_lis3mdl_cx;
  mag_ctx.write_reg = lsm6ds3tr_c_write_lis3mdl_cx;

  /* Check device ID */
  while (1) {
    lis3mdl_device_id_get(&mag_ctx, &whoamI);
    ESP_LOGI("lis3mdl", "LIS3MDL_WHO_AM_I: 0x%x", whoamI);
    if (whoamI == LIS3MDL_ID) {
      break;
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  /* Restore default configuration */
  lis3mdl_reset_set(&mag_ctx, PROPERTY_ENABLE);

  do {
    lis3mdl_reset_get(&mag_ctx, &rst);
  } while (rst);

  /* Set XL full scale and Gyro full scale */
  lsm6ds3tr_c_xl_full_scale_set(&dev_ctx, LSM6DS3TR_C_2g);
  lsm6ds3tr_c_gy_full_scale_set(&dev_ctx, LSM6DS3TR_C_2000dps);
  /* Configure LIS3MDL on the I2C master line */
  lis3mdl_block_data_update_set(&mag_ctx, PROPERTY_ENABLE);
  lis3mdl_full_scale_set(&dev_ctx, LIS3MDL_16_GAUSS);
  lis3mdl_temperature_meas_set(&mag_ctx, PROPERTY_ENABLE);
  lis3mdl_data_rate_set(&mag_ctx, LIS3MDL_LP_1kHz);
  lis3mdl_operating_mode_set(&mag_ctx, LIS3MDL_CONTINUOUS_MODE);
  /* Prepare sensor hub to read data from external sensor */
  lsm6ds3tr_c_sh_slv0_cfg_read(&dev_ctx, &val);
  /* Configure Sensor Hub to read one slave */
  lsm6ds3tr_c_sh_num_of_dev_connected_set(&dev_ctx, LSM6DS3TR_C_SLV_0);
  /* Enable master and XL trigger */
  lsm6ds3tr_c_func_en_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6ds3tr_c_sh_master_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set XL and Gyro Output Data Rate */
  lsm6ds3tr_c_xl_data_rate_set(&dev_ctx, LSM6DS3TR_C_XL_ODR_52Hz);
  lsm6ds3tr_c_gy_data_rate_set(&dev_ctx, LSM6DS3TR_C_GY_ODR_26Hz);

  vTaskDelay(1000 / portTICK_PERIOD_MS);

  int16_t outaccel[3], outgyro[3];
  lsm6ds3tr_c_fs_g_t gyro_scale = LSM6DS3TR_C_2000dps;
  lsm6ds3tr_c_gy_full_scale_get(NULL, &gyro_scale);
  ESP_LOGI(TAG, "gyro scale: %s", dps_map.at(gyro_scale));
  float gyro_sensitivity = lsm6ds3tr_c_gyroscope_range_to_sensitivity(gyro_scale);
  lsm6ds3tr_c_fs_xl_t accel_scale = LSM6DS3TR_C_4g;
  lsm6ds3tr_c_xl_full_scale_get(NULL, &accel_scale);
  ESP_LOGI(TAG, "accel scale: %s", xl_map.at(accel_scale));
  float accel_sensitivity = lsm6ds3tr_c_accelerometer_range_to_sensitivity(accel_scale);

  uint8_t emb_sh[18];
  uint8_t imu_data[36 + 7];
  PacketGenerate _generate;
  int16_t data_raw_magnetic[3];
  float magnetic_mG[3] = {0};
  int16_t data_raw_temperature = 0;
  float temperature_degC = 0, temp_lsm6ds3tr = 0;
  uint8_t drdy;
#endif
  sensor_if_t *ad7689[2];
  ad7689[0] = (sensor_if_t *) init_ad7689_1();
  ad7689[1] = (sensor_if_t *) init_ad7689_2();
  uint32_t read_count = 0;
#if WORK_MODE == WORK_MODE_DEBUG
  int64_t start_time = get_timestamp();
  int64_t last_time = get_timestamp();
#endif
  while (1) {
#if WORK_MODE == WORK_MODE_DEBUG
    start_time = get_timestamp();
#endif
#if ENABLE_IMU == 1
    if (++read_count % 5 == 0) {
      lsm6ds3tr_c_xl_flag_data_ready_get(NULL, &drdy);
      if (drdy) {
        lsm6ds3tr_c_acceleration_raw_get(NULL, outaccel);
        // lsm6ds3tr_c_sh_read_data_raw_get(&dev_ctx, (lsm6ds3tr_c_emb_sh_read_t *) &emb_sh);
        // data_raw_magnetic[0] = emb_sh[0] | (emb_sh[1] << 8);
        // data_raw_magnetic[1] = emb_sh[2] | (emb_sh[3] << 8);
        // data_raw_magnetic[2] = emb_sh[4] | (emb_sh[5] << 8);
        // ESP_LOG_BUFFER_HEX_LEVEL(TAG, emb_sh, 6, ESP_LOG_INFO);
        lis3mdl_magnetic_raw_get(&mag_ctx, (int16_t *) data_raw_magnetic);
        magnetic_mG[0] = 1000 * lis3mdl_from_fs16_to_gauss(data_raw_magnetic[0]);
        magnetic_mG[1] = 1000 * lis3mdl_from_fs16_to_gauss(data_raw_magnetic[1]);
        magnetic_mG[2] = 1000 * lis3mdl_from_fs16_to_gauss(data_raw_magnetic[2]);
#if WORK_MODE == WORK_MODE_DEBUG
        last_time = get_timestamp();
        ESP_LOGI(TAG, "3 last_time - start_time: %" PRId64 "us", last_time - start_time);
#endif
      }
      lsm6ds3tr_c_gy_flag_data_ready_get(NULL, &drdy);

      if (drdy) {
        lsm6ds3tr_c_angular_rate_raw_get(NULL, outgyro);
      }
      lsm6ds3tr_c_temp_flag_data_ready_get(NULL, &drdy);
#if WORK_MODE == WORK_MODE_DEBUG
      last_time = get_timestamp();
      ESP_LOGI(TAG, "6 last_time - start_time: %" PRId64 "us", last_time - start_time);
#endif
      if (drdy) {
        data_raw_temperature = 0;
        lsm6ds3tr_c_temperature_raw_get(NULL, &data_raw_temperature);
        temp_lsm6ds3tr = lsm6ds3tr_c_from_lsb_to_celsius(data_raw_temperature);
#if WORK_MODE == WORK_MODE_DEBUG
        last_time = get_timestamp();
        ESP_LOGI(TAG, "7 last_time - start_time: %" PRId64 "us", last_time - start_time);
#endif
      }
      data_raw_temperature = 0;
      lis3mdl_temperature_raw_get(&mag_ctx, &data_raw_temperature);
      temperature_degC = lis3mdl_from_lsb_to_celsius(data_raw_temperature);
#if WORK_MODE == WORK_MODE_DEBUG
      ESP_LOGI(TAG, "data_raw_temperature: %.3f,%.3f", temperature_degC, temp_lsm6ds3tr);
      last_time = get_timestamp();
      ESP_LOGI(TAG, "imu read use: %" PRId64 "us", last_time - start_time);
#endif
      float heading = calculate_heading(magnetic_mG[0], magnetic_mG[1]);
      float gyro_x = outgyro[0] * gyro_sensitivity;
      float gyro_y = outgyro[1] * gyro_sensitivity;
      float gyro_z = outgyro[2] * gyro_sensitivity;
      float accel_x = outaccel[0] * accel_sensitivity;
      float accel_y = outaccel[1] * accel_sensitivity;
      float accel_z = outaccel[2] * accel_sensitivity;
      float mag_x = magnetic_mG[0];
      float mag_y = magnetic_mG[1];
      float mag_z = magnetic_mG[2];

#if WORK_MODE == WORK_MODE_DEBUG
      ESP_LOGI("IMU READ", "Magnetic: X=%d, Y=%d, Z=%d; Magnetic: X=%.2f, Y=%.2f, Z=%.2f, heading: %.2f", data_raw_magnetic[0], data_raw_magnetic[1],
               data_raw_magnetic[2], magnetic_mG[0], magnetic_mG[1], magnetic_mG[2], heading);
      ESP_LOGI("IMU READ", "Gyro: X=%d, Y=%d, Z=%d; Accel: X=%d, Y=%d, Z=%d, Gyro: X=%.2f, Y=%.2f, Z=%.2f, Accel: X=%.2f, Y=%.2f, Z=%.2f,%.1f,%.1f", outgyro[0],
               outgyro[1], outgyro[2], outaccel[0], outaccel[1], outaccel[2], gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, gyro_sensitivity,
               accel_sensitivity);
#endif
      if (s_send_data) {
        _generate.packet_generate_imu(gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, mag_x, mag_y, mag_z);
#if SEND_DATA_BLUE == 1
        ble_tx_rx_send_notify((uint8_t *) _generate.data(), _generate.length());
#endif
#if SEND_DATA_UART == 1
        usb_serial_jtag_write_bytes((uint8_t *) _generate.data(), _generate.length(), 50 / portTICK_PERIOD_MS);
#endif
      }
#endif
    }

    uint16_t dummy;
    int channel = 0;
    extern spi_device_handle_t s_spi_handle;
    for (int idx = 0; idx < 2; idx++) {
      for (int i = 0; i < 8; i++) {
        channel = ad7689[idx]->read_channel_data(ad7689[idx], i, &dummy);
        if (channel >= 0 && channel < 8) {
          values[idx][channel] = dummy;
          voltage[idx][channel] = ((values[idx][channel] - AD7689_BASE_V) / 65536.0) * 2.5 / 0.35;
        }
      }
    }

#if WORK_MODE == WORK_MODE_DEBUG
    ESP_LOGI("AD READ", "ADC[1] values: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f ADC[2] values: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f", voltage[0][0],
             voltage[0][1], voltage[0][2], voltage[0][3], voltage[0][4], voltage[0][5], voltage[0][6], voltage[0][7], voltage[1][0], voltage[1][1],
             voltage[1][2], voltage[1][3], voltage[1][4], voltage[1][5], voltage[1][6], voltage[1][7]);
#endif
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
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
  set_protocol_get_timestamp(get_timestamp_ms);
#if SEND_DATA_UART == 1
  esp_log_level_set("*", ESP_LOG_NONE);
  // Configure USB SERIAL JTAG
  usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
      .tx_buffer_size = BUF_SIZE,
      .rx_buffer_size = BUF_SIZE,
  };

  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
#endif

  xTaskCreate(&sensor_read_task, "sensor_read_task", 4096, NULL, 5, NULL);
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
          case dt_cmd_imu:
            if (recv.Len() == DT_FIXED_LEN + 1) {
              s_send_data = recv.Data()[0];
              ESP_LOGI(TAG, "s_send_data is %s", s_send_data ? "true" : "false");
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
