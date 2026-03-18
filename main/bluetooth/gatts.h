#ifndef _H_GATTS_
#define _H_GATTS_
#if CONFIG_BT_ENABLED
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
void ble_gatt_init(void);

typedef void (*ble_gatt_cb)(uint8_t *data, uint32_t len);
void ble_gatt_set_cb(ble_gatt_cb cb);
void ble_tx_rx_send_notify(uint8_t *data, uint32_t len);
#ifdef __cplusplus
}
#endif

#endif

#endif