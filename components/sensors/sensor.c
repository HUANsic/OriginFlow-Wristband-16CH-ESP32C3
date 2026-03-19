#include "sensor.h"

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_check.h>
#include <esp_log.h>
#include <stdio.h>

#include "LIS3MDL.h"
#include "driver/i2c_master.h"
#include "lsm6ds3.h"
#define IMU_REG_LOG 0
#if IMU_SERIAL_INTERFACE_MODE == IMU_SERIAL_INTERFACE_MODE_SPI
spi_device_handle_t _sensors_hspidevice;
uint8_t _sensors_tx_buffer[16];
uint8_t _sensors_rx_buffer[16];
#else
static i2c_master_dev_handle_t i2c_dev_handle;
static i2c_master_bus_handle_t i2c_bus_handle;
#endif

/* Interface Functions */
int32_t imu_read_regs(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len) {
#if IMU_SERIAL_INTERFACE_MODE == IMU_SERIAL_INTERFACE_MODE_SPI
  // pull CS low
  gpio_set_level(IMU_LSM6DS3_CS_GPIO_NUM, 0);

  // read sequence: send register address with MSB set to 1, then read data
  uint8_t tx[16] = {0};
  tx[0] = reg | 0x80;  // set MSB to 1 for read operation
  uint8_t recv[16] = {0};
  spi_transaction_t trans = {
      .cmd = 0,
      .tx_buffer = &tx,
      .rx_buffer = recv,
      .length = 8 * (len + 1),    // total length includes the register address byte
      .rxlength = 8 * (len + 1),  // we will read until the end of the transaction
  };
  ESP_ERROR_CHECK(spi_device_transmit(_sensors_hspidevice, &trans));

  // release CS after transaction
  gpio_set_level(IMU_LSM6DS3_CS_GPIO_NUM, 1);
#if IMU_REG_LOG == 1
  ESP_LOGI(__func__, "reg:[%02X] ", reg);
  ESP_LOG_BUFFER_HEX_LEVEL("imu read regs", tx, 2, ESP_LOG_INFO);
  ESP_LOG_BUFFER_HEX_LEVEL(__func__, recv, (len + 1), ESP_LOG_INFO);
#endif
  memcpy(data, recv + 1, len);  // skip the first byte (register address)
#else                           // IMU_SERIAL_INTERFACE_MODE == IMU_SERIAL_INTERFACE_MODE_I2C

  uint8_t out_buf[1];
  out_buf[0] = reg;
  if (ctx == NULL) {
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_dev_handle, out_buf, sizeof(out_buf), data, len, I2C_TICKS_TO_WAIT));
  } else {
    ESP_ERROR_CHECK(i2c_master_transmit_receive((i2c_master_dev_handle_t) ctx->handle, out_buf, sizeof(out_buf), data, len, I2C_TICKS_TO_WAIT));
  }
#endif
  return 0;
}
int32_t imu_read_reg(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data) {
#if IMU_SERIAL_INTERFACE_MODE == IMU_SERIAL_INTERFACE_MODE_SPI
  // pull CS low
  gpio_set_level(IMU_LSM6DS3_CS_GPIO_NUM, 0);

  // read sequence: send register address with MSB set to 1, then read data
  uint8_t tx[2] = {0};
  tx[0] = reg | 0x80;  // set MSB to 1 for read operation
  uint8_t recv[2] = {0};
  spi_transaction_t trans = {
      .cmd = 0,
      .tx_buffer = &tx,
      .rx_buffer = recv,
      .length = 8 * 2,    // total length includes the register address byte
      .rxlength = 8 * 2,  // we will read until the end of the transaction
  };
  ESP_ERROR_CHECK(spi_device_transmit(_sensors_hspidevice, &trans));

  // release CS after transaction
  gpio_set_level(IMU_LSM6DS3_CS_GPIO_NUM, 1);
  *data = recv[1];
#if IMU_REG_LOG == 1
  ESP_LOGI(__func__, "reg:[%02X] Read 0x%02X", reg, *data);
#endif
#else  // IMU_SERIAL_INTERFACE_MODE == IMU_SERIAL_INTERFACE_MODE_I2C

  uint8_t out_buf[1];
  out_buf[0] = reg;
  if (ctx == NULL) {
    ESP_ERROR_CHECK(i2c_master_transmit_receive(i2c_dev_handle, out_buf, sizeof(out_buf), data, len, I2C_TICKS_TO_WAIT));
  } else {
    ESP_ERROR_CHECK(i2c_master_transmit_receive((i2c_master_dev_handle_t) ctx->handle, out_buf, sizeof(out_buf), data, len, I2C_TICKS_TO_WAIT));
  }
#endif
  return 0;
}

int32_t imu_write_reg(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t data) {
#if IMU_SERIAL_INTERFACE_MODE == IMU_SERIAL_INTERFACE_MODE_SPI
  // pull CS low
  gpio_set_level(IMU_LSM6DS3_CS_GPIO_NUM, 0);

  // write sequence: send register address with MSB set to 1, then write data
  _sensors_tx_buffer[0] = reg & ~0x80;  // set MSB to 0 for write operation
  _sensors_tx_buffer[1] = data;
  spi_transaction_t trans = {
      .cmd = 0,
      .tx_buffer = _sensors_tx_buffer,
      .rx_buffer = _sensors_rx_buffer,
      .length = 8 * (2),  // total length includes the register address byte
      .rxlength = 0,      // we will read until the end of the transaction, but in write operation we don't care about the received data
  };
  ESP_ERROR_CHECK(spi_device_transmit(_sensors_hspidevice, &trans));

  // release CS after transaction
  gpio_set_level(IMU_LSM6DS3_CS_GPIO_NUM, 1);
#else
  uint8_t buff[2];
  buff[0] = reg;
  buff[1] = data;
  if (ctx == NULL) {
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev_handle, buff, 2, I2C_TICKS_TO_WAIT));
  } else {
    ESP_ERROR_CHECK(i2c_master_transmit((i2c_master_dev_handle_t) ctx->handle, buff, 2, I2C_TICKS_TO_WAIT));
  }
#endif
  return 0;
}

/**
 * @brief  Write generic device register
 *
 * @param  ctx   read / write interface definitions(ptr)
 * @param  reg   register to write
 * @param  data  pointer to data to write in register reg(ptr)
 * @param  len   number of consecutive register to write
 * @retval       interface status (MANDATORY: return 0 -> no Error)
 *
 */
int32_t imu_write_regs(const stmdev_ctx_t *ctx, uint8_t reg, uint8_t *data, uint16_t len) {
#if IMU_SERIAL_INTERFACE_MODE == IMU_SERIAL_INTERFACE_MODE_SPI
  // pull CS low
  gpio_set_level(IMU_LSM6DS3_CS_GPIO_NUM, 0);

  // write sequence: send register address with MSB set to 1, then write data
  _sensors_tx_buffer[0] = reg & ~0x80;  // set MSB to 0 for write operation
  memcpy(_sensors_tx_buffer + 1, data, len);

  spi_transaction_t trans = {
      .cmd = 0,
      .tx_buffer = _sensors_tx_buffer,
      .rx_buffer = _sensors_rx_buffer,
      .length = 8 * (len + 1),  // total length includes the register address byte
      .rxlength = 0,            // we will read until the end of the transaction, but in write operation we don't care about the received data
  };
  ESP_ERROR_CHECK(spi_device_transmit(_sensors_hspidevice, &trans));
#if IMU_REG_LOG == 1
  ESP_LOGI(__func__, "reg:[%02X] ", reg);
  ESP_LOG_BUFFER_HEX_LEVEL(__func__, _sensors_tx_buffer, (len + 1), ESP_LOG_INFO);
#endif
  // release CS after transaction
  gpio_set_level(IMU_LSM6DS3_CS_GPIO_NUM, 1);
#else
  uint8_t *out_buf;
  out_buf = (uint8_t *) malloc(len + 1);
  if (out_buf == NULL) {
    ESP_LOGE(__FUNCTION__, "malloc fail");
    return false;
  }
  out_buf[0] = reg;
  memcpy(out_buf + 1, data, len);
  if (ctx == NULL) {
    ESP_ERROR_CHECK(i2c_master_transmit(i2c_dev_handle, out_buf, len + 1, I2C_TICKS_TO_WAIT));
  } else {
    ESP_ERROR_CHECK(i2c_master_transmit((i2c_master_dev_handle_t) ctx->handle, out_buf, len + 1, I2C_TICKS_TO_WAIT));
  }
  free(out_buf);
#endif
  return 0;
}

void IMU_Init(void) {
  // Initialize LSM6DS3
}
#if IMU_SERIAL_INTERFACE_MODE == IMU_SERIAL_INTERFACE_MODE_I2C
i2c_master_dev_handle_t addDevice(uint16_t devAddr, uint32_t clkSpeed) {
  i2c_device_config_t dev_cfg = {};
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_cfg.device_address = devAddr;
  dev_cfg.scl_speed_hz = clkSpeed;

  i2c_master_dev_handle_t dev_handle;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &dev_handle));
  return dev_handle;
}
i2c_master_dev_handle_t get_master_i2c_handle() {
  return i2c_dev_handle;
}

#endif

void sensor_init(void) {
  //   // IMU INT
  gpio_config_t io_conf = {
      .pin_bit_mask = 1ULL << IMU_LSM6DS3_INT_GPIO_NUM,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  // IMU CSn
  io_conf.pin_bit_mask = 1ULL << IMU_LSM6DS3_CS_GPIO_NUM;
  io_conf.mode = GPIO_MODE_OUTPUT;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
#if IMU_SERIAL_INTERFACE_MODE == IMU_SERIAL_INTERFACE_MODE_SPI
  // SPI MISO
  io_conf.pin_bit_mask = 1ULL << SPI_MISO_GPIO_NUM;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  gpio_set_level((gpio_num_t) IMU_LSM6DS3_CS_GPIO_NUM, 0);  // active low
#else
  // i2c
  // SPI SDO
  io_conf.pin_bit_mask = 1ULL << SPI_MISO_GPIO_NUM;
  io_conf.mode = GPIO_MODE_OUTPUT;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  gpio_set_level((gpio_num_t) SPI_MISO_GPIO_NUM, 0);  // active high
  // IMU CSn
  gpio_set_level((gpio_num_t) IMU_LSM6DS3_CS_GPIO_NUM, 1);  // active low
#endif

  // ADC1 CSn
  io_conf.pin_bit_mask = 1ULL << ADC1_CS_GPIO_NUM;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  gpio_set_level((gpio_num_t) ADC1_CS_GPIO_NUM, 1);  // active low

  // ADC2 CSn
  io_conf.pin_bit_mask = 1ULL << ADC1_CS_GPIO_NUM;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  gpio_set_level((gpio_num_t) ADC2_CS_GPIO_NUM, 1);  // active low
#if IMU_SERIAL_INTERFACE_MODE == IMU_SERIAL_INTERFACE_MODE_SPI
  // Initialize SPI peripheral
  spi_bus_config_t buscfg = {
      .mosi_io_num = SPI_MOSI_GPIO_NUM,
      .miso_io_num = SPI_MISO_GPIO_NUM,
      .sclk_io_num = SPI_CLK_GPIO_NUM,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
  };
  ESP_LOGI("SENSORS INIT", "Performing bus initialization");
  ESP_ERROR_CHECK(spi_bus_initialize(SENSORS_SPI_HOST, &buscfg, SPI_DMA_DISABLED));
  // Lump all sensors into one device and switch between them using manual CS control
  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = 1 * 1000 * 1000,  // Clock out at 8 MHz
      .command_bits = 0,                   // no command phase, only data
      .address_bits = 0,                   // no address phase, only data
      .dummy_bits = 0,                     // no dummy phase
      .mode = 0,                           // SPI mode 0
      .spics_io_num = -1,                  // we will use manual CS control
      .queue_size = 6,
  };
  ESP_LOGI("SENSORS INIT", "Performing device initialization");
  ESP_LOGI("MEM", "Free heap: %d bytes", esp_get_free_heap_size());
  ESP_LOGI("MEM", "Minimum free heap: %d bytes", esp_get_minimum_free_heap_size());
  esp_err_t ret = spi_bus_add_device(SENSORS_SPI_HOST, &devcfg, &_sensors_hspidevice);
  if (ret != ESP_OK) {
    ESP_LOGE("SENSOR", "spi_bus_add_device failed: %s", esp_err_to_name(ret));
  }
  IMU_Init();
#else
  i2c_master_bus_config_t i2c_mst_config = {};
  i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
  i2c_mst_config.glitch_ignore_cnt = 7;
  i2c_mst_config.i2c_port = I2C_NUM;
  i2c_mst_config.scl_io_num = (gpio_num_t) I2C_GPIO_SCL;
  i2c_mst_config.sda_io_num = (gpio_num_t) I2C_GPIO_SDA;
  i2c_mst_config.flags.enable_internal_pullup = true;

  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle));
  i2c_dev_handle = addDevice(LSM6DS3TR_C_I2C_ADD_L >> 1, 400000);
#endif
}

int spi_write_regs(spi_device_handle_t hdev, uint8_t *data, uint16_t len) {
  if (hdev == NULL || data == NULL) {
    return -1;
  }

  spi_transaction_t trans = {
      .cmd = 0,
      .tx_buffer = data,
      .length = (8 * len),
  };
  ESP_ERROR_CHECK(spi_device_transmit(hdev, &trans));

  ESP_LOG_BUFFER_HEX_LEVEL("spi write regs", data, len, ESP_LOG_INFO);

  return 0;  // Success
}

int spi_write_read_regs(spi_device_handle_t hdev, uint8_t *tx_data, size_t tx_len, uint8_t *rx_data, size_t rx_len) {
  if (hdev == NULL || tx_data == NULL || rx_data == NULL) {
    return -1;
  }

  spi_transaction_t trans;
  memset(&trans, 0, sizeof(spi_transaction_t));
  trans.cmd = 0;
  trans.tx_buffer = tx_data;
  trans.length = (8 * tx_len);
  trans.rx_buffer = rx_data;
  trans.rxlength = (8 * rx_len);

  ESP_ERROR_CHECK(spi_device_transmit(hdev, &trans));


  return 0;  // Success
}