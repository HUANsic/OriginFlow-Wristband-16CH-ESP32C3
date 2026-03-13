#include <stdio.h>
#include "testbench1.h"
#include "Utilities.h"
#include <esp_log.h>
#include <rom/ets_sys.h>

void debug_disable_pins();

void blink(void)
{
    // Code to blink the LED can be added here
    ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Hello, world!");
    Utility_Init(); // Initialize utilities with LED on GPIO 21 and button on GPIO 20
    ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Initialization complete. Starting LED blink loop.");
    while (1)
    {
        ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Red.");
        Utility_LED_SetColor(48, 0, 0); // Set LED to red
        Utility_Button_Update();        // Update button state
        ets_delay_us(100000);           // Delay for 100ms

        ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Green.");
        Utility_LED_SetColor(0, 48, 0); // Set LED to green
        Utility_Button_Update();        // Update button state
        ets_delay_us(100000);           // Delay for 100ms

        ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Blue.");
        Utility_LED_SetColor(0, 0, 48); // Set LED to blue
        Utility_Button_Update();        // Update button state
        ets_delay_us(100000);           // Delay for 100ms

        ESP_LOGI("AAAAAAAA!!!!!!!!!!", "White.");
        Utility_LED_SetColor(16, 16, 16); // Set LED to white
        Utility_Button_Update();          // Update button state
        ets_delay_us(100000);             // Delay for 100ms

        ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Off.");
        Utility_LED_SetColor(0, 0, 0); // Clear LED
        Utility_Button_Update();       // Update button state
        ets_delay_us(100000);          // Delay for 100ms
        Utility_Button_Update();       // Update button state
        ets_delay_us(100000);          // Delay for 100ms
        Utility_Button_Update();       // Update button state
        ets_delay_us(100000);          // Delay for 100ms
        Utility_Button_Update();       // Update button state
        ets_delay_us(100000);          // Delay for 100ms
        Utility_Button_Update();       // Update button state
        ets_delay_us(100000);          // Delay for 100ms
        Utility_Button_Update();       // Update button state
        ets_delay_us(100000);          // Delay for 100ms
    }
}

void Utility_Button_Pressed_CB()
{
    ESP_LOGI("Button", "Button Pressed!");
}

void Utility_Button_Released_CB()
{
    ESP_LOGI("Button", "Button Released!");
}

static void _delay_timer(uint32_t ms)
{
    ms *= 2; // Since the timer frequency is 2kHz, each tick is 0.5ms
    do
    {
        Utility_Timer_ClearReady();
        if (ms-- == 0)
        {
            break;
        }
    } while (!Utility_Timer_IsReady());
}

void blink_with_timer()
{
    // Code to blink the LED can be added here
    ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Hello, world!");
    Utility_Init(); // Initialize utilities with LED on GPIO 21 and button on GPIO 20
    ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Initialization complete. Starting LED blink loop.");
    while (1)
    {
        ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Red.");
        Utility_LED_SetColor(48, 0, 0); // Set LED to red
        Utility_Button_Update();        // Update button state
        _delay_timer(100);              // Delay for 100ms

        ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Green.");
        Utility_LED_SetColor(0, 48, 0); // Set LED to green
        Utility_Button_Update();        // Update button state
        _delay_timer(100);              // Delay for 100ms

        ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Blue.");
        Utility_LED_SetColor(0, 0, 48); // Set LED to blue
        Utility_Button_Update();        // Update button state
        _delay_timer(100);              // Delay for 100ms

        ESP_LOGI("AAAAAAAA!!!!!!!!!!", "White.");
        Utility_LED_SetColor(16, 16, 16); // Set LED to white
        Utility_Button_Update();          // Update button state
        _delay_timer(100);                // Delay for 100ms

        ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Off.");
        Utility_LED_SetColor(0, 0, 0); // Clear LED
        Utility_Button_Update();       // Update button state
        _delay_timer(100);             // Delay for 100ms
        Utility_Button_Update();       // Update button state
        _delay_timer(100);             // Delay for 100ms
        Utility_Button_Update();       // Update button state
        _delay_timer(100);             // Delay for 100ms
        Utility_Button_Update();       // Update button state
        _delay_timer(100);             // Delay for 100ms
        Utility_Button_Update();       // Update button state
        _delay_timer(100);             // Delay for 100ms
        Utility_Button_Update();       // Update button state
        _delay_timer(100);             // Delay for 100ms
    }
}

#include "esp_log.h"
#include "string.h"
#include "driver/usb_serial_jtag.h"

#define BUF_SIZE (1024)

void read_usb_serial()
{
    Utility_Init();                // Initialize utilities with LED on GPIO 21 and button on GPIO 20
    Utility_LED_SetColor(0, 0, 0); // Clear LED
    // Configure USB SERIAL JTAG
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = BUF_SIZE,
        .tx_buffer_size = BUF_SIZE,
    };

    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
    ESP_LOGI("usb_serial_jtag echo", "USB_SERIAL_JTAG init done");

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    if (data == NULL)
    {
        ESP_LOGE("usb_serial_jtag echo", "no memory for data");
        return;
    }

    while (1)
    {

        // int len = usb_serial_jtag_read_bytes(data, (BUF_SIZE - 1), portMAX_DELAY);
        int len = usb_serial_jtag_read_bytes(data, (BUF_SIZE - 1), 0);

        // Write data back to the USB SERIAL JTAG
        if (len)
        {
            // ESP_LOG_BUFFER_HEXDUMP("Recv str: ]", data, len, ESP_LOG_INFO);
            usb_serial_jtag_write_bytes((const char *)data, len, 0);
            data[len] = '\0';
            // ESP_LOG_BUFFER_HEXDUMP("Recv str: ", data, len, ESP_LOG_INFO);
        }
    }
}

uint8_t dummy[BUF_SIZE];
void test_usb_serial_max_speed()
{
    Utility_Init();
    Utility_LED_SetColor(0, 0, 0); // Clear LED
    // Configure USB SERIAL JTAG
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = BUF_SIZE,
        .tx_buffer_size = BUF_SIZE,
    };

    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
    ESP_LOGI("usb_serial_jtag echo", "USB_SERIAL_JTAG init done");

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    if (data == NULL)
    {
        ESP_LOGE("usb_serial_jtag echo", "no memory for data");
        return;
    }
    // Fill the dummy buffer with some data to send
    for (uint16_t i = 0; i < BUF_SIZE; i++)
    {
        dummy[i] = i % 256;
    }
    ets_delay_us(1000000);
    uint32_t byte_count = 0;
    uint8_t idx = 0;
    uint32_t ticks[100];
    uint32_t bytes[100];
    uint32_t lastTick = Utility_Timer_GetTick();
    uint32_t diffTick, diffBytes;
    float speedbps = 0, tempbps;
    // break after 10k iterations (~5s)
    for (uint16_t i = 0; i < 10000; i++)
    {
        while (lastTick >= Utility_Timer_GetTick())
        {
            // wait until the next tick and DO NOT BLOCK
            byte_count += usb_serial_jtag_write_bytes((const char *)dummy, BUF_SIZE, 0);
        }
        lastTick = Utility_Timer_GetTick();
        diffTick = lastTick - ticks[idx];
        diffBytes = byte_count - bytes[idx];
        tempbps = ((float)diffBytes) / ((float)diffTick) * 2000;
        speedbps = (speedbps + tempbps) / 2; // Moving average to smooth out the speed measurement
        ticks[idx] = lastTick;
        bytes[idx] = byte_count;
        idx = (idx + 1) % 100;
    }

    // uint32_t byte_count = 0;
    // uint32_t startTick = Utility_Timer_GetTick();
    // while (startTick + 1000 > Utility_Timer_GetTick()) // Run the test for 10 seconds
    // {
    //     byte_count += usb_serial_jtag_write_bytes((const char *)dummy, BUF_SIZE, 0);
    // }

    ets_delay_us(1000000);
    // Print the results
    ESP_LOGI("USB SERIAL SPEED TEST", "\n\nUSB SERIAL JTAG Speed Test Results:");
    ESP_LOGI("USB SERIAL SPEED TEST", "Total bytes sent: %u", byte_count);
    ESP_LOGI("USB SERIAL SPEED TEST", "Average speed: %.2f bytes/sec", speedbps);
}

void battery_read()
{
    // Code to read battery level can be added here
    Utility_Init();
    debug_disable_pins();
    Utility_LED_SetColor(0, 0, 0); // Clear LED
    while (1)
    {
        float voltage = Utility_ADC_Read();
        ESP_LOGI("BATTERY_READ", "Battery voltage: %.2f V", voltage);
        ets_delay_us(1000000); // Delay for 1 second before the next reading
    }
}

#include <driver/gpio.h>
void debug_disable_pins()
{
    // IMU INT
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << 10,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // SPI MISO
    io_conf.pin_bit_mask = 1ULL << 5;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // SPI SCK
    io_conf.pin_bit_mask = 1ULL << 6;
    io_conf.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(6, 0); // active high

    // SPI MOSI
    io_conf.pin_bit_mask = 1ULL << 7;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(7, 0); // active high

    // ADC1 CSn
    io_conf.pin_bit_mask = 1ULL << 8;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(8, 1); // active low

    // ADC2 CSn
    io_conf.pin_bit_mask = 1ULL << 4;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(4, 1); // active low

    // IMU CSn
    io_conf.pin_bit_mask = 1ULL << 9;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    gpio_set_level(9, 1); // active low
}

#include "Sensors.h"
#include "LSM6DS3.h"
#include <driver/gpio.h>
void imu_who_am_i()
{
    uint8_t who_am_i;
    ESP_LOGI("IMU WHOAMI", "Entered WHOAMI function");
    debug_disable_pins();
    ESP_LOGI("IMU WHOAMI", "Pins configured. Starting sensor initialization.");
    Sensors_Init(); // Initialize sensors, including the IMU
    ESP_LOGI("IMU WHOAMI", "Initialization complete. Starting WHOAMI read loop.");
    ets_delay_us(100000); // Delay for 100 milliseconds before the first reading
    while (1)
    {
        lsm6ds3_read_reg(NULL, 0x0F, &who_am_i, 1);
        ESP_LOGI("IMU WHOAMI", "LSM6DS3 WHOAMI: 0x%02X", who_am_i);
        ets_delay_us(1000000); // Delay for 1 second before the next reading
    }
}

void imu_read_accel_gyro()
{
    uint8_t who_am_i;
    ESP_LOGI("IMU READ", "Entered IMU read function");
    Utility_Init();
    debug_disable_pins();
    Utility_LED_SetColor(0, 0, 0); // Clear LED
    ESP_LOGI("IMU READ", "Pins configured. Starting sensor initialization.");
    Sensors_Init(); // Initialize sensors, including the IMU
    ESP_LOGI("IMU READ", "Initialization complete. Starting IMU read loop.");
    lsm6ds3_read_reg(NULL, 0x0F, &who_am_i, 1);
    ESP_LOGI("IMU READ", "LSM6DS3 WHOAMI: 0x%02X (dummy)", who_am_i);
    lsm6ds3_read_reg(NULL, 0x0F, &who_am_i, 1);
    ESP_LOGI("IMU READ", "LSM6DS3 WHOAMI: 0x%02X", who_am_i);
    lsm6ds3_xl_data_rate_set(NULL, LSM6DS3_XL_ODR_12Hz5);
    lsm6ds3_gy_data_rate_set(NULL, LSM6DS3_GY_ODR_12Hz5);
    ets_delay_us(10000); // Delay for 10 milliseconds before the first reading

    int16_t outaccel[3], outgyro[3];
    while (1)
    {
        lsm6ds3_angular_rate_raw_get(NULL, outgyro);
        lsm6ds3_acceleration_raw_get(NULL, outaccel);
        ESP_LOGI("IMU READ", "Gyro: X=%d, Y=%d, Z=%d; Accel: X=%d, Y=%d, Z=%d", outgyro[0], outgyro[1], outgyro[2], outaccel[0], outaccel[1], outaccel[2]);

        ets_delay_us(50000); // Delay for 50 milliseconds before the next reading
    }
}

#include "AD7689.h"
extern spi_device_handle_t _sensors_hspidevice;
void test_ad_read()
{
    uint16_t values[16];
    uint16_t dummy1;
    AD_CFG_u cfg;
    ESP_LOGI("AD READ", "Entered AD read function");
    Utility_Init();
    // debug_disable_pins();
    Utility_LED_SetColor(0, 0, 0); // Clear LED
    ESP_LOGI("AD READ", "Pins configured. Starting sensor initialization.");
    Sensors_Init(); // Initialize sensors, including the ADC
    ESP_LOGI("AD READ", "Initialization complete. Starting ADC read loop.");
    // Configure USB SERIAL JTAG
    usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
        .rx_buffer_size = BUF_SIZE,
        .tx_buffer_size = BUF_SIZE,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
    ESP_LOGI("AD READ", "USB_SERIAL_JTAG init done");

    while (1)
    {
        gpio_set_level(_ADC1_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(1, AD_INCC_BIP_SING, 0, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        // read using the Read After Conversion (RAC) method
        AD_Transaction(_sensors_hspidevice, &cfg, &dummy1); // dummy read
        gpio_set_level(_ADC1_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading
        gpio_set_level(_ADC1_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(1, AD_INCC_BIP_SING, 1, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        AD_Transaction(_sensors_hspidevice, &cfg, &dummy1); // dummy read
        gpio_set_level(_ADC1_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading
        gpio_set_level(_ADC1_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(1, AD_INCC_BIP_SING, 2, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        AD_Transaction(_sensors_hspidevice, &cfg, values);
        gpio_set_level(_ADC1_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading
        gpio_set_level(_ADC1_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(1, AD_INCC_BIP_SING, 3, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        AD_Transaction(_sensors_hspidevice, &cfg, values + 1);
        gpio_set_level(_ADC1_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading
        gpio_set_level(_ADC1_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(1, AD_INCC_BIP_SING, 4, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        AD_Transaction(_sensors_hspidevice, &cfg, values + 2);
        gpio_set_level(_ADC1_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading
        gpio_set_level(_ADC1_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(1, AD_INCC_BIP_SING, 5, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        AD_Transaction(_sensors_hspidevice, &cfg, values + 3);
        gpio_set_level(_ADC1_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading
        gpio_set_level(_ADC1_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(1, AD_INCC_BIP_SING, 6, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        AD_Transaction(_sensors_hspidevice, &cfg, values + 4);
        gpio_set_level(_ADC1_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading
        gpio_set_level(_ADC1_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(1, AD_INCC_BIP_SING, 7, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        AD_Transaction(_sensors_hspidevice, &cfg, values + 5);
        gpio_set_level(_ADC1_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading
        gpio_set_level(_ADC1_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(0, AD_INCC_BIP_SING, 7, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        AD_Transaction(_sensors_hspidevice, &cfg, values + 6);
        gpio_set_level(_ADC1_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading
        gpio_set_level(_ADC1_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(0, AD_INCC_BIP_SING, 7, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        AD_Transaction(_sensors_hspidevice, &cfg, values + 7);
        gpio_set_level(_ADC1_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading

        gpio_set_level(_ADC2_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(1, AD_INCC_BIP_SING, 0, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        // read using the Read After Conversion (RAC) method
        AD_Transaction(_sensors_hspidevice, &cfg, &dummy1); // dummy read
        gpio_set_level(_ADC2_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading
        gpio_set_level(_ADC2_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(1, AD_INCC_BIP_SING, 1, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        AD_Transaction(_sensors_hspidevice, &cfg, &dummy1); // dummy read
        gpio_set_level(_ADC2_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading
        gpio_set_level(_ADC2_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(1, AD_INCC_BIP_SING, 2, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        AD_Transaction(_sensors_hspidevice, &cfg, values + 8);
        gpio_set_level(_ADC2_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading
        gpio_set_level(_ADC2_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(1, AD_INCC_BIP_SING, 3, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        AD_Transaction(_sensors_hspidevice, &cfg, values + 9);
        gpio_set_level(_ADC2_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading
        gpio_set_level(_ADC2_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(1, AD_INCC_BIP_SING, 4, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        AD_Transaction(_sensors_hspidevice, &cfg, values + 10);
        gpio_set_level(_ADC2_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading
        gpio_set_level(_ADC2_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(1, AD_INCC_BIP_SING, 5, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        AD_Transaction(_sensors_hspidevice, &cfg, values + 11);
        gpio_set_level(_ADC2_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading
        gpio_set_level(_ADC2_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(1, AD_INCC_BIP_SING, 6, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        AD_Transaction(_sensors_hspidevice, &cfg, values + 12);
        gpio_set_level(_ADC2_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading
        gpio_set_level(_ADC2_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(1, AD_INCC_BIP_SING, 7, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        AD_Transaction(_sensors_hspidevice, &cfg, values + 13);
        gpio_set_level(_ADC2_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading
        gpio_set_level(_ADC2_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(0, AD_INCC_BIP_SING, 7, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        AD_Transaction(_sensors_hspidevice, &cfg, values + 14);
        gpio_set_level(_ADC2_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading
        gpio_set_level(_ADC2_CS_GPIO_NUM, 0);
        cfg = AD_BuildConfig(0, AD_INCC_BIP_SING, 7, 1, AD_REF_EXT_NOBUF_NOTEMP, AD_SEQ_DISABLE, 0);
        AD_Transaction(_sensors_hspidevice, &cfg, values + 15);
        gpio_set_level(_ADC2_CS_GPIO_NUM, 1);
        ets_delay_us(10); // Delay for 10 microseconds before the next reading

        sprintf((char *)dummy, "A1CH1:%d, A1CH2:%d, A1CH3:%d, A1CH4:%d, A1CH5:%d, A1CH6:%d, A1CH7:%d, A1CH8:%d, A2CH11:%d, A2CH2:%d, A2CH3:%d, A2CH4:%d, A2CH5:%d, A2CH6:%d, A2CH7:%d, A2CH8:%d \n", (int16_t)values[0], (int16_t)values[1], (int16_t)values[2], (int16_t)values[3], (int16_t)values[4], (int16_t)values[5], (int16_t)values[6], (int16_t)values[7], (int16_t)values[8], (int16_t)values[9], (int16_t)values[10], (int16_t)values[11], (int16_t)values[12], (int16_t)values[13], (int16_t)values[14], (int16_t)values[15]);
        usb_serial_jtag_write_bytes(dummy, strlen((char *)dummy), 0);
        // ESP_LOGI("AD7689", "A1CH1:%d, A1CH2:%d, A1CH3:%d, A1CH4:%d, A1CH5:%d, A1CH6:%d, A1CH7:%d, A1CH8:%d, A2CH11:%d, A2CH2:%d, A2CH3:%d, A2CH4:%d, A2CH5:%d, A2CH6:%d, A2CH7:%d, A2CH8:%d \n", (int16_t)values[0], (int16_t)values[1], (int16_t)values[2], (int16_t)values[3], (int16_t)values[4], (int16_t)values[5], (int16_t)values[6], (int16_t)values[7], (int16_t)values[8], (int16_t)values[9], (int16_t)values[10], (int16_t)values[11], (int16_t)values[12], (int16_t)values[13], (int16_t)values[14], (int16_t)values[15]);
        ets_delay_us(20000); // Delay for 0.02 second before the next round
    }
}
