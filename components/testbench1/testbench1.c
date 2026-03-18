#include "testbench1.h"

#include <esp_log.h>
#include <rom/ets_sys.h>
#include <stdio.h>

#include "Utilities.h"
#include "ad7689.h"
#include "driver/spi_master.h"


int ADC_Buff[IN_NUM] = {0};
int dbg = 0;

// AD7689配置寄存器
uint16_t AD_CH[8] = {INX_1, INX_2, INX_3, INX_4, INX_5, INX_6, INX_7, INX_0};

uint16_t AD7689_Config = (CFG_OVR | INCC_UNI | INX_0 | BW_FULL | REF_INT_OUT | SEQ_OFF | RB_DIS) << 2;

extern spi_device_handle_t _sensors_hspidevice;
void debug_disable_pins();

void blink(void) {
  // Code to blink the LED can be added here
  ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Hello, world!");
  Utility_Init();  // Initialize utilities with LED on GPIO 21 and button on GPIO 20
  ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Initialization complete. Starting LED blink loop.");
  while (1) {
    ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Red.");
    Utility_LED_SetColor(48, 0, 0);  // Set LED to red
    Utility_Button_Update();         // Update button state
    ets_delay_us(100000);            // Delay for 100ms

    ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Green.");
    Utility_LED_SetColor(0, 48, 0);  // Set LED to green
    Utility_Button_Update();         // Update button state
    ets_delay_us(100000);            // Delay for 100ms

    ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Blue.");
    Utility_LED_SetColor(0, 0, 48);  // Set LED to blue
    Utility_Button_Update();         // Update button state
    ets_delay_us(100000);            // Delay for 100ms

    ESP_LOGI("AAAAAAAA!!!!!!!!!!", "White.");
    Utility_LED_SetColor(16, 16, 16);  // Set LED to white
    Utility_Button_Update();           // Update button state
    ets_delay_us(100000);              // Delay for 100ms

    ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Off.");
    Utility_LED_SetColor(0, 0, 0);  // Clear LED
    Utility_Button_Update();        // Update button state
    ets_delay_us(100000);           // Delay for 100ms
    Utility_Button_Update();        // Update button state
    ets_delay_us(100000);           // Delay for 100ms
    Utility_Button_Update();        // Update button state
    ets_delay_us(100000);           // Delay for 100ms
    Utility_Button_Update();        // Update button state
    ets_delay_us(100000);           // Delay for 100ms
    Utility_Button_Update();        // Update button state
    ets_delay_us(100000);           // Delay for 100ms
    Utility_Button_Update();        // Update button state
    ets_delay_us(100000);           // Delay for 100ms
  }
}

void Utility_Button_Pressed_CB() {
  ESP_LOGI("Button", "Button Pressed!");
}

void Utility_Button_Released_CB() {
  ESP_LOGI("Button", "Button Released!");
}

static void _delay_timer(uint32_t ms) {
  ms *= 2;  // Since the timer frequency is 2kHz, each tick is 0.5ms
  do {
    Utility_Timer_ClearReady();
    if (ms-- == 0) {
      break;
    }
  } while (!Utility_Timer_IsReady());
}

void blink_with_timer() {
  // Code to blink the LED can be added here
  ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Hello, world!");
  Utility_Init();  // Initialize utilities with LED on GPIO 21 and button on GPIO 20
  ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Initialization complete. Starting LED blink loop.");
  while (1) {
    ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Red.");
    Utility_LED_SetColor(48, 0, 0);  // Set LED to red
    Utility_Button_Update();         // Update button state
    _delay_timer(100);               // Delay for 100ms

    ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Green.");
    Utility_LED_SetColor(0, 48, 0);  // Set LED to green
    Utility_Button_Update();         // Update button state
    _delay_timer(100);               // Delay for 100ms

    ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Blue.");
    Utility_LED_SetColor(0, 0, 48);  // Set LED to blue
    Utility_Button_Update();         // Update button state
    _delay_timer(100);               // Delay for 100ms

    ESP_LOGI("AAAAAAAA!!!!!!!!!!", "White.");
    Utility_LED_SetColor(16, 16, 16);  // Set LED to white
    Utility_Button_Update();           // Update button state
    _delay_timer(100);                 // Delay for 100ms

    ESP_LOGI("AAAAAAAA!!!!!!!!!!", "Off.");
    Utility_LED_SetColor(0, 0, 0);  // Clear LED
    Utility_Button_Update();        // Update button state
    _delay_timer(100);              // Delay for 100ms
    Utility_Button_Update();        // Update button state
    _delay_timer(100);              // Delay for 100ms
    Utility_Button_Update();        // Update button state
    _delay_timer(100);              // Delay for 100ms
    Utility_Button_Update();        // Update button state
    _delay_timer(100);              // Delay for 100ms
    Utility_Button_Update();        // Update button state
    _delay_timer(100);              // Delay for 100ms
    Utility_Button_Update();        // Update button state
    _delay_timer(100);              // Delay for 100ms
  }
}

#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "string.h"

#define BUF_SIZE (1024)

void read_usb_serial() {
  Utility_Init();                 // Initialize utilities with LED on GPIO 21 and button on GPIO 20
  Utility_LED_SetColor(0, 0, 0);  // Clear LED
  // Configure USB SERIAL JTAG
  usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
      .rx_buffer_size = BUF_SIZE,
      .tx_buffer_size = BUF_SIZE,
  };

  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
  ESP_LOGI("usb_serial_jtag echo", "USB_SERIAL_JTAG init done");

  // Configure a temporary buffer for the incoming data
  uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
  if (data == NULL) {
    ESP_LOGE("usb_serial_jtag echo", "no memory for data");
    return;
  }

  while (1) {
    // int len = usb_serial_jtag_read_bytes(data, (BUF_SIZE - 1), portMAX_DELAY);
    int len = usb_serial_jtag_read_bytes(data, (BUF_SIZE - 1), 0);

    // Write data back to the USB SERIAL JTAG
    if (len) {
      // ESP_LOG_BUFFER_HEXDUMP("Recv str: ]", data, len, ESP_LOG_INFO);
      usb_serial_jtag_write_bytes((const char *) data, len, 0);
      data[len] = '\0';
      // ESP_LOG_BUFFER_HEXDUMP("Recv str: ", data, len, ESP_LOG_INFO);
    }
  }
}

uint8_t dummy[BUF_SIZE];
void test_usb_serial_max_speed() {
  Utility_Init();
  Utility_LED_SetColor(0, 0, 0);  // Clear LED
  // Configure USB SERIAL JTAG
  usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
      .rx_buffer_size = BUF_SIZE,
      .tx_buffer_size = BUF_SIZE,
  };

  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
  ESP_LOGI("usb_serial_jtag echo", "USB_SERIAL_JTAG init done");

  // Configure a temporary buffer for the incoming data
  uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
  if (data == NULL) {
    ESP_LOGE("usb_serial_jtag echo", "no memory for data");
    return;
  }
  // Fill the dummy buffer with some data to send
  for (uint16_t i = 0; i < BUF_SIZE; i++) {
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
  for (uint16_t i = 0; i < 10000; i++) {
    while (lastTick >= Utility_Timer_GetTick()) {
      // wait until the next tick and DO NOT BLOCK
      byte_count += usb_serial_jtag_write_bytes((const char *) dummy, BUF_SIZE, 0);
    }
    lastTick = Utility_Timer_GetTick();
    diffTick = lastTick - ticks[idx];
    diffBytes = byte_count - bytes[idx];
    tempbps = ((float) diffBytes) / ((float) diffTick) * 2000;
    speedbps = (speedbps + tempbps) / 2;  // Moving average to smooth out the speed measurement
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

void battery_read() {
  // Code to read battery level can be added here
  Utility_Init();
  debug_disable_pins();
  Utility_LED_SetColor(0, 0, 0);  // Clear LED
  while (1) {
    float voltage = Utility_ADC_Read();
    ESP_LOGI("BATTERY_READ", "Battery voltage: %.2f V", voltage);
    ets_delay_us(1000000);  // Delay for 1 second before the next reading
  }
}

#include <driver/gpio.h>
void debug_disable_pins() {
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
  gpio_set_level(6, 0);  // active high

  // SPI MOSI
  io_conf.pin_bit_mask = 1ULL << 7;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  gpio_set_level(7, 0);  // active high

  // ADC1 CSn
  io_conf.pin_bit_mask = 1ULL << 8;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  gpio_set_level(8, 1);  // active low

  // ADC2 CSn
  io_conf.pin_bit_mask = 1ULL << 4;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  gpio_set_level(4, 1);  // active low

  // IMU CSn
  io_conf.pin_bit_mask = 1ULL << 9;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  gpio_set_level(9, 1);  // active low
}

#include <driver/gpio.h>

#include "lsm6ds3.h"
#include "sensor.h"

void imu_who_am_i() {
  uint8_t who_am_i;
  ESP_LOGI("IMU WHOAMI", "Entered WHOAMI function");
  debug_disable_pins();
  ESP_LOGI("IMU WHOAMI", "Pins configured. Starting sensor initialization.");
  sensor_init();  // Initialize sensors, including the IMU
  ESP_LOGI("IMU WHOAMI", "Initialization complete. Starting WHOAMI read loop.");
  ets_delay_us(100000);  // Delay for 100 milliseconds before the first reading
  while (1) {
    imu_read_regs(NULL, 0x0F, &who_am_i, 1);
    ESP_LOGI("IMU WHOAMI", "LSM6DS3 WHOAMI: 0x%02X", who_am_i);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ;  // Delay for 1 second before the next reading
  }
}
typedef union {
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;
static float_t magnetic_mG[3];
static axis3bit16_t data_raw_magnetic;

void imu_read_accel_gyro() {
  uint8_t who_am_i;
  ESP_LOGI("IMU READ", "Entered IMU read function");
  Utility_Init();
  debug_disable_pins();
  Utility_LED_SetColor(0, 0, 0);  // Clear LED
  ESP_LOGI("IMU READ", "Pins configured. Starting sensor initialization.");
  sensor_init();  // Initialize sensors, including the IMU
  ESP_LOGI("IMU READ", "Initialization complete. Starting IMU read loop.");
  imu_read_regs(NULL, 0x0F, &who_am_i, 1);
  ESP_LOGI("IMU READ", "LSM6DS3 WHOAMI: 0x%02X (dummy)", who_am_i);
  imu_read_regs(NULL, 0x0F, &who_am_i, 1);
  ESP_LOGI("IMU READ", "LSM6DS3 WHOAMI: 0x%02X", who_am_i);
  lsm6ds3tr_c_xl_data_rate_set(NULL, LSM6DS3TR_C_XL_ODR_12Hz5);
  lsm6ds3tr_c_gy_data_rate_set(NULL, LSM6DS3TR_C_XL_ODR_12Hz5);
  vTaskDelay(5000 / portTICK_PERIOD_MS);

  int16_t outaccel[3], outgyro[3];
  lsm6ds3tr_c_fs_g_t gyro_scale = 0;
  lsm6ds3tr_c_gy_full_scale_get(NULL, &gyro_scale);
  float gyro_sensitivity = lsm6ds3tr_c_gyroscope_range_to_sensitivity(gyro_scale);
  lsm6ds3tr_c_fs_xl_t accel_scale = 0;
  lsm6ds3tr_c_xl_full_scale_get(NULL, &accel_scale);
  float accel_sensitivity = lsm6ds3tr_c_accelerometer_range_to_sensitivity(accel_scale);
  // lsm6ds3tr_c_sh_read_t emb_sh={0};
  uint8_t emb_sh[18];

  while (1) {
    lsm6ds3tr_c_angular_rate_raw_get(NULL, outgyro);
    lsm6ds3tr_c_acceleration_raw_get(NULL, outaccel);
    lsm6ds3tr_c_sh_read_data_raw_get(NULL, (lsm6ds3tr_c_emb_sh_read_t *) &emb_sh);
    ESP_LOGI("IMU READ", "Gyro: X=%d, Y=%d, Z=%d; Accel: X=%d, Y=%d, Z=%d, Gyro: X=%.2f, Y=%.2f, Z=%.2f, Accel: X=%.2f, Y=%.2f, Z=%.2f", outgyro[0], outgyro[1],
             outgyro[2], outaccel[0], outaccel[1], outaccel[2], outgyro[0] * gyro_sensitivity, outgyro[1] * gyro_sensitivity, outgyro[2] * gyro_sensitivity,
             outaccel[0] * accel_sensitivity, outaccel[1] * accel_sensitivity, outaccel[2] * accel_sensitivity);

    vTaskDelay(50 / portTICK_PERIOD_MS);  // Delay for 50 milliseconds before the next reading
  }
}

void test_ad_read() {
  // uint16_t values[2][8];
  // float voltage[2][8];
  // for (int i = 0; i < 2; i++) {
  //   for (int j = 0; j < 8; j++) {
  //     values[i][j] = 0;
  //     voltage[i][j] = 0;
  //   }
  // }
  // uint16_t dummy;
  // ESP_LOGI("AD READ", "Entered AD read function");
  // Utility_Init();
  // debug_disable_pins();
  // Utility_LED_SetColor(0, 0, 0);  // Clear LED
  // ESP_LOGI("AD READ", "Pins configured. Starting sensor initialization.");
  // sensor_init();  // Initialize sensors, including the ADC
  // ESP_LOGI("AD READ", "Initialization complete. Starting ADC read loop.");
  // gpio_set_level(ADC1_CS_GPIO_NUM, 1);
  // ets_delay_us(10);  // Delay for 10 microseconds before the next reading
  // gpio_set_level(ADC1_CS_GPIO_NUM, 0);
  // ets_delay_us(2);  // Delay for 10 microseconds before the next reading
  // spi_write_regs(_sensors_hspidevice, (uint8_t *) &AD7689_Config, 2);
  // gpio_set_level(ADC1_CS_GPIO_NUM, 1);
  // ets_delay_us(2);  // Delay for 10 microseconds before the next reading
  // gpio_set_level(ADC1_CS_GPIO_NUM, 0);
  // vTaskDelay(1000 / portTICK_PERIOD_MS);
  // for (int i = 0; i < 2; i++) {
  //   AD7689_Config = (CFG_OVR | INCC_UNI | AD_CH[i] | BW_FULL | REF_INT_OUT | SEQ_OFF | (0UL << RB_POS)) << 2;
  //   gpio_set_level(ADC1_CS_GPIO_NUM, 1);
  //   ets_delay_us(2);  // Delay for 10 microseconds before the next reading
  //   gpio_set_level(ADC1_CS_GPIO_NUM, 0);
  //   ets_delay_us(1);  // Delay for 10 microseconds before the next reading
  //   ad7689_read_channel(_sensors_hspidevice, i, &values[0][i]);
  //   gpio_set_level(ADC1_CS_GPIO_NUM, 1);
  //   ets_delay_us(2);  // Delay for 10 microseconds before the next reading
  //   gpio_set_level(ADC1_CS_GPIO_NUM, 0);
  //   vTaskDelay(500 / portTICK_PERIOD_MS);
  // }
  // gpio_set_level(ADC1_CS_GPIO_NUM, 1);

  // gpio_set_level(ADC2_CS_GPIO_NUM, 1);
  // ets_delay_us(10);  // Delay for 10 microseconds before the next reading
  // gpio_set_level(ADC2_CS_GPIO_NUM, 0);
  // ets_delay_us(2);  // Delay for 10 microseconds before the next reading
  // spi_write_regs(_sensors_hspidevice, (uint8_t *) &AD7689_Config, 2);
  // gpio_set_level(ADC2_CS_GPIO_NUM, 1);
  // ets_delay_us(2);  // Delay for 10 microseconds before the next reading
  // gpio_set_level(ADC2_CS_GPIO_NUM, 0);
  // vTaskDelay(1000 / portTICK_PERIOD_MS);
  // for (int i = 0; i < 2; i++) {
  //   AD7689_Config = (CFG_OVR | INCC_UNI | AD_CH[i] | BW_FULL | REF_INT_OUT | SEQ_OFF | (0UL << RB_POS)) << 2;
  //   gpio_set_level(ADC2_CS_GPIO_NUM, 1);
  //   ets_delay_us(2);  // Delay for 10 microseconds before the next reading
  //   gpio_set_level(ADC2_CS_GPIO_NUM, 0);
  //   ets_delay_us(1);  // Delay for 10 microseconds before the next reading
  //   ad7689_read_channel(_sensors_hspidevice, i, &values[1][i]);
  //   gpio_set_level(ADC2_CS_GPIO_NUM, 1);
  //   ets_delay_us(2);  // Delay for 10 microseconds before the next reading
  //   gpio_set_level(ADC2_CS_GPIO_NUM, 0);
  //   vTaskDelay(500 / portTICK_PERIOD_MS);
  // }
  // while (1) {
  //   for (int i = 0; i < 8; i++) {
  //     gpio_set_level(ADC1_CS_GPIO_NUM, 1);
  //     ets_delay_us(2);  // Delay for 10 microseconds before the next reading
  //     gpio_set_level(ADC1_CS_GPIO_NUM, 0);
  //     ets_delay_us(1);  // Delay for 10 microseconds before the next reading
  //     ad7689_read_channel(_sensors_hspidevice, i, &values[0][i]);
  //     gpio_set_level(ADC1_CS_GPIO_NUM, 1);
  //     ets_delay_us(2);  // Delay for 10 microseconds before the next reading
  //     gpio_set_level(ADC1_CS_GPIO_NUM, 0);
  //     voltage[0][i] = (values[0][i] / 65536.0) * 2.5;
  //   }
  //   vTaskDelay(50 / portTICK_PERIOD_MS);
  //   gpio_set_level(ADC1_CS_GPIO_NUM, 1);

  //   for (int i = 0; i < 8; i++) {
  //     gpio_set_level(ADC2_CS_GPIO_NUM, 1);
  //     ets_delay_us(2);  // Delay for 10 microseconds before the next reading
  //     gpio_set_level(ADC2_CS_GPIO_NUM, 0);
  //     ets_delay_us(1);  // Delay for 10 microseconds before the next reading
  //     ad7689_read_channel(_sensors_hspidevice, i, &values[1][i]);
  //     gpio_set_level(ADC2_CS_GPIO_NUM, 1);
  //     ets_delay_us(2);  // Delay for 10 microseconds before the next reading
  //     gpio_set_level(ADC2_CS_GPIO_NUM, 0);
  //     voltage[1][i] = (values[1][i] / 65536.0) * 2.5;
  //   }
  //   vTaskDelay(50 / portTICK_PERIOD_MS);
  //   gpio_set_level(ADC1_CS_GPIO_NUM, 2);

  //   ESP_LOGI("AD READ", "ADC[1] values: %d %d %d %d %d %d %d %d ADC[2] values: %d %d %d %d %d %d %d %d", values[0][0], values[0][1], values[0][2],
  //   values[0][3],
  //            values[0][4], values[0][5], values[0][6], values[0][7], values[1][0], values[1][1], values[1][2], values[1][3], values[1][4], values[1][5],
  //            values[0][6], values[0][7]);
  //   ESP_LOGI("AD READ", "ADC[1] values: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f ADC[2] values: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f", voltage[0][0],
  //            voltage[0][1], voltage[0][2], voltage[0][3], voltage[0][4], voltage[0][5], voltage[0][6], voltage[0][7], voltage[1][0], voltage[1][1],
  //            voltage[1][2], voltage[1][3], voltage[1][4], voltage[1][5], voltage[0][6], voltage[0][7]);
  //   vTaskDelay(1000 / portTICK_PERIOD_MS);
  // }
}