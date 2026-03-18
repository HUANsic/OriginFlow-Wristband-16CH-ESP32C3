#ifndef _H_SENSORS_H
#define _H_SENSORS_H
#include <stdint.h>
#include <sys/cdefs.h>

#include "driver/i2c_types.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_SERIAL_INTERFACE_MODE_I2C 0
#define IMU_SERIAL_INTERFACE_MODE_SPI 1

#define IMU_SERIAL_INTERFACE_MODE     IMU_SERIAL_INTERFACE_MODE_SPI
#if IMU_SERIAL_INTERFACE_MODE == IMU_SERIAL_INTERFACE_MODE_SPI
#define SENSORS_SPI_HOST         SPI2_HOST
#define IMU_LSM6DS3_CS_GPIO_NUM  (9)
#define IMU_LSM6DS3_INT_GPIO_NUM (10)
#define ADC1_CS_GPIO_NUM         (8)
#define ADC2_CS_GPIO_NUM         (4)
#define SPI_CLK_GPIO_NUM         (6)
#define SPI_MISO_GPIO_NUM        (5)
#define SPI_MOSI_GPIO_NUM        (7)

#else
#define I2C_NUM           I2C_NUM_0
#define I2C_TICKS_TO_WAIT (1000 / portTICK_PERIOD_MS)  // Maximum ticks to wait before issuing a timeout.

#define I2C_GPIO_SCL      (6)
#define I2C_GPIO_SDA      (7)
#endif

void sensor_init(void);

#if IMU_SERIAL_INTERFACE_MODE == IMU_SERIAL_INTERFACE_MODE_SPI
int spi_write_regs(spi_device_handle_t hdev, uint8_t *data, uint16_t len);
int spi_write_read_regs(spi_device_handle_t hdev, uint8_t *tx_data, size_t tx_len, uint8_t *rx_data, size_t rx_len);
#elif IMU_SERIAL_INTERFACE_MODE == IMU_SERIAL_INTERFACE_MODE_I2C
i2c_master_dev_handle_t addDevice(uint16_t devAddr, uint32_t clkSpeed);
i2c_master_dev_handle_t get_master_i2c_handle();
#endif

#ifdef __cplusplus
}
#endif

#endif