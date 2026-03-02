#include "Utilities.h"
// #include <stdio.h>

/*******************************************************************************
 * Button Input
 ******************************************************************************/
#include <driver/gpio.h>

#define BUTTON_DEBOUNCE_CYCLES 5 // debounce time in cycles

uint8_t _button_countdown = 0;
bool _button_state = false, _button_state_brefore_cb = false;
gpio_num_t _button_gpio_num;

void __attribute__((weak)) Utility_Button_Pressed_CB()
{
    // Callback function for when button is pressed, define it in higher level code if needed
}

void __attribute__((weak)) Utility_Button_Released_CB()
{
    // Callback function for when button is released, define it in higher level code if needed
}

esp_err_t Utility_Button_Init(gpio_num_t gpio_num)
{
    // Initialization code for button input can be added here
    _button_gpio_num = gpio_num;
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << gpio_num,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    _button_state = gpio_get_level(gpio_num) == 0; // Active low button
    _button_state_brefore_cb = _button_state;
    return ESP_OK;
}

void Utility_Button_Update(void)
{
    // Code to update button state can be added here
    bool current_state = gpio_get_level(_button_gpio_num) == 0; // Active low button
    if (current_state != _button_state)
    {
        _button_countdown = BUTTON_DEBOUNCE_CYCLES;
        _button_state = current_state;
    }
    else if (_button_countdown > 0)
    {
        _button_countdown--;
        if (_button_countdown == 0)
        {
            _button_state_brefore_cb = _button_state;
            if (_button_state)
            {
                // Button is pressed, handle the event here
                Utility_Button_Pressed_CB();
            }
            else
            {
                // Button is released, handle the event here
                Utility_Button_Released_CB();
            }
        }
    }
}

/*******************************************************************************
 * Battery Voltage Monitoring using ADC
 ******************************************************************************/
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>

static adc_oneshot_unit_handle_t _adc_handle;
static const adc_channel_t _adc_channel = ADC_CHANNEL_2;
static adc_cali_handle_t _adc_cali_handle;
// static const float _adc_internal_vref = 1.1;
// static const float _adc_internal_vref = 0.75;

esp_err_t Utility_ADC_Init()
{
    // Initialization code for ADC can be added here
    // just use oneshot mode, which takes ~5us including calibration
    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_config, &_adc_handle));

    adc_oneshot_chan_cfg_t _adc_channel_config = {
        .atten = ADC_ATTEN_DB_12,    // set attenuation to support up to 3.3V input (NO! ONLY 2.5V!!!)
        .bitwidth = ADC_BITWIDTH_12, // so no need higher bitwidth (NO! ONLY 12 BIT IS SUPPORTED!!!)
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(_adc_handle, _adc_channel, &_adc_channel_config));

    // Calibrate ADC
    // adc_cali_scheme_ver_t scheme_mask;
    // adc_cali_check_scheme(&scheme_mask);
    // ESP_LOGI("ADC Calibration", "Supported calibration schemes: %s%s",
    //          (scheme_mask & ADC_CALI_SCHEME_VER_LINE_FITTING) ? "Line Fitting " : "",
    //          (scheme_mask & ADC_CALI_SCHEME_VER_CURVE_FITTING) ? "Curve Fitting " : "");
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = _adc_channel_config.atten,
        .bitwidth = _adc_channel_config.bitwidth,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &_adc_cali_handle));
    return ESP_OK;
}

float Utility_ADC_Read()
{
    // Code to read ADC value can be added here
    int raw_data;
    ESP_ERROR_CHECK(adc_oneshot_read(_adc_handle, _adc_channel, &raw_data));
    ESP_LOGI("ADC", "Raw ADC data: %d", raw_data);
    // Convert raw ADC value to voltage
    // float voltage = _adc_internal_vref * raw_data / 4096.0 * 16 / 3; // 4 is the attenuation factor for ADC_ATTEN_DB_12
    int voltage;
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(_adc_cali_handle, raw_data, &voltage));
    return ((float)voltage) * 4 / 3000; // mV to V, and compensate the resistor divider
}

/*******************************************************************************
 * WS2812B LED using RMT Peripheral
 ******************************************************************************/
#include <driver/rmt_tx.h>

static const rmt_symbol_word_t _WS2812_ZERO = {
    .level0 = 1,
    .duration0 = 3, // T0H=0.3us
    .level1 = 0,
    .duration1 = 9, // T0L=0.9us
};

static const rmt_symbol_word_t _WS2812_ONE = {
    .level0 = 1,
    .duration0 = 9, // T1H=0.9us
    .level1 = 0,
    .duration1 = 3, // T1L=0.3us
};

// reset defaults to 300us
static const rmt_symbol_word_t _WS2812_RESET = {
    .level0 = 0,
    .duration0 = 1500,
    .level1 = 0,
    .duration1 = 1500, // 1500 ticks = 150us
};

static rmt_channel_handle_t _led;
static rmt_encoder_handle_t _led_encoder;

static bool _led_ready = true;

static bool _Utility_LED_Tx_CB(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx)
{
    // Callback function for when transmission is done
    // __unused(user_ctx);
    _led_ready = true;
    return true;
}

static size_t _Utility_LED_Enc_CB(const void *data, size_t data_size,
                                  size_t symbols_written, size_t symbols_free,
                                  rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    // ESP_LOGI("LED Encoder", "Encoding data for LED, data_size: %d, symbols_written: %d, symbols_free: %d", data_size, symbols_written, symbols_free);
    // ESP_LOGI("LED Encoder", "Data: %02X %02X %02X", ((uint8_t *)data)[0], ((uint8_t *)data)[1], ((uint8_t *)data)[2]);
    if (data_size * 8 + 1 > symbols_free)
    {
        // ERROR! Not enough space to encode the data
        *done = false; // maybe block it?
        return 0;
    }
    for (size_t i = 0; i < data_size; i++)
    {
        for (uint8_t j = 7; j < 8; j--)
        {
            if (*((uint8_t *)data + i) & (1 << j))
                symbols[8 * i + (7 - j)] = _WS2812_ONE;
            else
                symbols[8 * i + (7 - j)] = _WS2812_ZERO;
        }
    }
    symbols[8 * data_size] = _WS2812_RESET; // Append reset symbol after each LED data
    *done = true;
    return data_size * 8 + 1; // number of symbols written
}

esp_err_t Utility_LED_Init(gpio_num_t gpio_num)
{
    // Initialization code for WS2812B LED can be added here
    // rmt_write_items(led_chan, &_WS2812_ZERO, 1, false); // Send a zero symbol to initialize the channel
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = gpio_num,
        .mem_block_symbols = 64,     // increase the block size can make the LED less flickering
        .resolution_hz = 10000000lu, // 10MHz resolution, each tick is 0.1us
        .trans_queue_depth = 4,      // set the number of transactions that can be pending in the background
        .flags.with_dma = 0,         // disable DMA mode
        .flags.invert_out = 0,       // do not invert the output signal
    };

    rmt_simple_encoder_config_t simple_encoder_config = {
        .callback = _Utility_LED_Enc_CB,
        .arg = NULL,
        .min_chunk_size = 0, // use default value
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &_led));

    // Register a callback to indicate when transmission is done and is ready for next transmission
    rmt_tx_event_callbacks_t cbs = {.on_trans_done = _Utility_LED_Tx_CB};
    ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(_led, &cbs, NULL));

    rmt_new_simple_encoder(&simple_encoder_config, &_led_encoder);
    rmt_enable(_led);

    return ESP_OK;
}

static const rmt_transmit_config_t _led_tx_config = {
    .flags.eot_level = 0, // set the output level when transmission is done
    .loop_count = 0,      // no looping
};

esp_err_t Utility_LED_SetColor(uint8_t red, uint8_t green, uint8_t blue)
{
    // Set the color of the WS2812B LED
    if (!_led_ready)
    {
        return ESP_FAIL; // Wait until the previous transmission is done
    }
    _led_ready = false;

    // WS2812B expects color data in GRB order
    uint8_t color_data[3] = {green, red, blue};
    ESP_ERROR_CHECK(rmt_transmit(_led, _led_encoder, color_data, 3, &_led_tx_config));

    return ESP_OK;
}

/*******************************************************************************
 * Periodic Timer using Precision Timer
 ******************************************************************************/
#include <esp_timer.h>

static uint32_t _timer_frequency_hz = 2000; // 2kHz timer frequency
static esp_timer_handle_t _periodic_timer;
static bool _timer_ready = true;
static uint32_t _timer_overflow_count = 0;
static uint32_t _timer_cycle_count = 0;

static void _Utility_Timer_CB(void *arg)
{
    // Callback function for periodic timer, this is where you can add code to be executed periodically
    _timer_cycle_count++;
    if (_timer_ready)
    {
        _timer_overflow_count++;
    }
    else
    {
        _timer_ready = true;
    }
}

esp_err_t Utility_Timer_Init()
{
    // Initialization code for periodic timer can be added here
    esp_timer_create_args_t periodic_timer_args = {
        .callback = _Utility_Timer_CB, // Set the callback function for the timer here if needed
        .name = "Periodic Timer",
        .dispatch_method = ESP_TIMER_ISR, // Use ISR dispatch method for high precision
    };
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &_periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(_periodic_timer, 1000000lu / _timer_frequency_hz));
    return ESP_OK;
}

uint32_t Utility_Timer_GetTick()
{
    return _timer_cycle_count;
}

uint32_t Utility_Timer_GetOverflow()
{
    return _timer_overflow_count;
}

bool Utility_Timer_IsReady()
{
    return _timer_ready;
}

void Utility_Timer_ClearReady()
{
    _timer_ready = false;
}

/*******************************************************************************
 * Utility Initialization
 ******************************************************************************/
esp_err_t Utility_Init()
{
    // Initialization code for utilities can be added here
    // LED initialization
    ESP_ERROR_CHECK(Utility_LED_Init(21));
    // Button initialization
    ESP_ERROR_CHECK(Utility_Button_Init(20));
    // ADC initialization
    ESP_ERROR_CHECK(Utility_ADC_Init());
    // Timer initialization
    ESP_ERROR_CHECK(Utility_Timer_Init());
    return ESP_OK;
}
