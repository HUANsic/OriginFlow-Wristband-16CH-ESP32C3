#include <stdio.h>
#include "Sensors.h"
#include "LSM6DS3.h"
#include "LIS3MDL.h"
#include "AD7689.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_check.h>
#include <esp_log.h>

#define _SENSORS_SPI_HOST SPI2_HOST

// #define _IMU_LSM6DS3_CS_GPIO_NUM 9
// #define _ADC1_CS_GPIO_NUM 8
// #define _ADC2_CS_GPIO_NUM 4

spi_device_handle_t _sensors_hspidevice;
uint8_t _sensors_tx_buffer[16];
uint8_t _sensors_rx_buffer[16];

/* Interface Functions */
int32_t lsm6ds3_read_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                         uint8_t *data,
                         uint16_t len)
{
    // pull CS low
    gpio_set_level(_IMU_LSM6DS3_CS_GPIO_NUM, 0);

    // read sequence: send register address with MSB set to 1, then read data
    _sensors_tx_buffer[0] = reg | 0x80; // set MSB to 1 for read operation

    spi_transaction_t trans = {
        .cmd = 0,
        .tx_buffer = _sensors_tx_buffer,
        .rx_buffer = _sensors_rx_buffer,
        .length = 8 * (len + 1),   // total length includes the register address byte
        .rxlength = 8 * (len + 1), // we will read until the end of the transaction
    };
    ESP_ERROR_CHECK(spi_device_transmit(_sensors_hspidevice, &trans));

    // release CS after transaction
    gpio_set_level(_IMU_LSM6DS3_CS_GPIO_NUM, 1);

    memcpy(data, _sensors_rx_buffer + 1, len); // skip the first byte (register address)

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
int32_t lsm6ds3_write_reg(const stmdev_ctx_t *ctx, uint8_t reg,
                          uint8_t *data,
                          uint16_t len)
{
    // pull CS low
    gpio_set_level(_IMU_LSM6DS3_CS_GPIO_NUM, 0);

    // write sequence: send register address with MSB set to 1, then write data
    _sensors_tx_buffer[0] = reg & ~0x80; // set MSB to 0 for write operation
    memcpy(_sensors_tx_buffer + 1, data, len);

    spi_transaction_t trans = {
        .cmd = 0,
        .tx_buffer = _sensors_tx_buffer,
        .rx_buffer = _sensors_rx_buffer,
        .length = 8 * (len + 1), // total length includes the register address byte
        .rxlength = 0,           // we will read until the end of the transaction, but in write operation we don't care about the received data
    };
    ESP_ERROR_CHECK(spi_device_transmit(_sensors_hspidevice, &trans));

    // release CS after transaction
    gpio_set_level(_IMU_LSM6DS3_CS_GPIO_NUM, 1);

    return 0;
}

void IMU_Init(void)
{
    // Initialize LSM6DS3
    // IMU INT
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << _IMU_LSM6DS3_INT_GPIO_NUM,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    // IMU CS
    io_conf.pin_bit_mask = 1ULL << _IMU_LSM6DS3_CS_GPIO_NUM;
    io_conf.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(_IMU_LSM6DS3_CS_GPIO_NUM, 1); // active low
    // ADC1 CS
    io_conf.pin_bit_mask = 1ULL << _ADC1_CS_GPIO_NUM;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(_ADC1_CS_GPIO_NUM, 1); // active low
    // ADC2 CS
    io_conf.pin_bit_mask = 1ULL << _ADC2_CS_GPIO_NUM;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(_ADC2_CS_GPIO_NUM, 1); // active low
}

void Sensors_Init(void)
{
    // Initialize SPI peripheral
    spi_bus_config_t buscfg = {
        .mosi_io_num = 7,
        .miso_io_num = 5,
        .sclk_io_num = 6,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_LOGI("SENSORS INIT", "Performing bus initialization");
    ESP_ERROR_CHECK(spi_bus_initialize(_SENSORS_SPI_HOST, &buscfg, SPI_DMA_DISABLED));
    // Lump all sensors into one device and switch between them using manual CS control
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8 * 1000 * 1000, // Clock out at 8 MHz
        .command_bits = 0,                 // no command phase, only data
        .address_bits = 0,                 // no address phase, only data
        .dummy_bits = 0,                   // no dummy phase
        .mode = 2,                         // SPI mode 2
        .spics_io_num = -1,                // we will use manual CS control
        .queue_size = 2,
    };
    ESP_LOGI("SENSORS INIT", "Performing device initialization");
    ESP_LOGI("MEM", "Free heap: %d bytes", esp_get_free_heap_size());
    ESP_LOGI("SENSORS INIT", "111");
    ESP_LOGI("MEM", "Minimum free heap: %d bytes", esp_get_minimum_free_heap_size());
    ESP_LOGI("SENSORS INIT", "222");
    // ESP_ERROR_CHECK(spi_bus_add_device(_SENSORS_SPI_HOST, &devcfg, &_sensors_hspidevice));
    esp_err_t ret = spi_bus_add_device(_SENSORS_SPI_HOST, &devcfg, &_sensors_hspidevice);
    if (ret != ESP_OK)
    {
        ESP_LOGE("SENSOR", "spi_bus_add_device failed: %s", esp_err_to_name(ret));
    }
    IMU_Init();
    // LIS3MDL_Init(hspi); // Initialize LIS3MDL, you can implement this function similarly to IMU_Init
}
