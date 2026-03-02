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

    uint16_t config_word = cfg->u16;

    spi_transaction_t trans = {
        .cmd = 0,
        .tx_buffer = &config_word,
        .rx_buffer = data,
        .length = 16,
        .rxlength = 16};
    ESP_ERROR_CHECK(spi_device_transmit(hdev, &trans));

    return 0; // Success
}
