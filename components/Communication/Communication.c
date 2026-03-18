#include <stdio.h>
#include "Communication.h"

#include <esp_log.h>
#include "string.h"
#include "driver/usb_serial_jtag.h"

#define _USB_BUFF_SIZE 1024

static uint8_t _usb_tx_buffer[_USB_BUFF_SIZE];
static uint8_t _usb_rx_buffer[_USB_BUFF_SIZE];

esp_err_t USB_Init()
{
    // Configure USB SERIAL JTAG
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = _USB_BUFF_SIZE,
        .tx_buffer_size = _USB_BUFF_SIZE,
    };

    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));

    return ESP_OK;
}

void read_usb_serial()
{

    while (1)
    {

        // int len = usb_serial_jtag_read_bytes(data, (BUF_SIZE - 1), portMAX_DELAY);
        int len = usb_serial_jtag_read_bytes(_usb_rx_buffer, _USB_BUFF_SIZE, 0);

        // Write data back to the USB SERIAL JTAG
        if (len)
        {
            usb_serial_jtag_write_bytes(_usb_rx_buffer, len, 0);
        }
    }
}
