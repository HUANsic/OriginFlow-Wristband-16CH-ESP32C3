#include <stdio.h>
#include "testbench1.h"
#include "esp_system.h"

void app_main(void)
{
    // ESP_BREAKPOINT();
    // __asm__("ebreak");
    // blink_with_timer();
    // blink();
    // read_usb_serial();
    // test_usb_serial_max_speed();
    // battery_read();
    // imu_read_accel_gyro();
    test_ad_read();

    while (1)
    {
        // Just loop forever
    }
}
