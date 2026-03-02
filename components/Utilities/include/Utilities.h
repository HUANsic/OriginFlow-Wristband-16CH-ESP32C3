#include <esp_err.h>
#include <esp_log.h>

esp_err_t Utility_Init();

esp_err_t Utility_LED_SetColor(uint8_t red, uint8_t green, uint8_t blue);

void Utility_Button_Update(void);

float Utility_ADC_Read();

uint32_t Utility_Timer_GetTick();

uint32_t Utility_Timer_GetOverflow();

bool Utility_Timer_IsReady();

void Utility_Timer_ClearReady();
