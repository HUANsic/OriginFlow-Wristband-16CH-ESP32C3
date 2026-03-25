#include <stdlib.h>
#include "AD7689.h"

#include <esp_check.h>
#include <esp_log.h>

uint8_t AD_Transaction(spi_device_handle_t hdev, AD_CFG_u *cfg, uint16_t *data)
{
    if (hdev == NULL || cfg == NULL || data == NULL)
    {
        return 1; // Error: Null pointer
    }

    uint8_t tx_buffer[2] = {0};
    tx_buffer[0] = (cfg->u16 >> 8) & 0xFF;
    tx_buffer[1] = cfg->u16 & 0xFF;
    // ESP_LOGI("AD7689", "Sending config word: 0x%04X", cfg->u16);
    uint8_t rx_buffer[2] = {0};

    spi_transaction_t trans = {
        .cmd = 0,
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer,
        .length = 16,
        .rxlength = 16};
    ESP_ERROR_CHECK(spi_device_transmit(hdev, &trans));

    *data = (rx_buffer[0] << 8) | rx_buffer[1];
    // ESP_LOGI("AD7689", "Received data: 0x%04X", *data);

    return 0; // Success
}
