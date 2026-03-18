#if CONFIG_BT_ENABLED
#if CONFIG_BT_BLUEDROID_ENABLED

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "gatts.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#define TAG "GATTS_DEMO"

/// Declare the static function

#define ADV_CONFIG_FLAG      (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static uint8_t adv_config_done = 0;
uint8_t service_uuid[] = {0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
uint8_t service_uuid2[] = {0xFB, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00};
uint8_t characteristic_uuid[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,  // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010,  // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,        // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL,  // test_manufacturer,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static ble_gatt_cb ble_gatt_cb_ = NULL;
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,        // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL,  //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;
const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static const esp_gatts_attr_db_t gatt_db[3] = {
    // Service Declaration
    [0] = {{ESP_GATT_AUTO_RSP},
           {ESP_UUID_LEN_16, (uint8_t *) &primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(service_uuid2), (uint8_t *) &service_uuid2}      },

    /* Characteristic Declaration */
    [1] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *) &character_declaration_uuid, ESP_GATT_PERM_READ, 1, 1, (uint8_t *) &char_prop_read_write_notify}},

    /* Characteristic Value */
    [2] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *) &characteristic_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, 500, 0, NULL}                   }
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void ble_gatt_set_cb(ble_gatt_cb cb) {
  ble_gatt_cb_ = cb;
}
static void ble_gatt_data_call_cb(uint8_t *data, uint32_t len) {
  if (ble_gatt_cb_) {
    ble_gatt_cb_(data, len);
  }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
      adv_config_done &= (~adv_config_flag);
      if (adv_config_done == 0) {
        esp_ble_gap_start_advertising(&adv_params);
      }
      break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
      adv_config_done &= (~scan_rsp_config_flag);
      if (adv_config_done == 0) {
        esp_ble_gap_start_advertising(&adv_params);
      }
      break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
      esp_ble_gap_start_advertising(&adv_params);
      break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
      esp_ble_gap_start_advertising(&adv_params);
      break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
      // advertising start complete event to indicate advertising start successfully or failed
      if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
        break;
      }
      ESP_LOGI(TAG, "Advertising start successfully");
      break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
      if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "Advertising stop failed, status %d", param->adv_stop_cmpl.status);
        break;
      }
      ESP_LOGI(TAG, "Advertising stop successfully");
      break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
      ESP_LOGI(TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d", param->update_conn_params.status,
               param->update_conn_params.conn_int, param->update_conn_params.latency, param->update_conn_params.timeout);
      break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
      ESP_LOGI(TAG, "Packet length update, status %d, rx %d, tx %d", param->pkt_data_length_cmpl.status, param->pkt_data_length_cmpl.params.rx_len,
               param->pkt_data_length_cmpl.params.tx_len);
      break;
    default:
      break;
  }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
  switch (event) {
    case ESP_GATTS_REG_EVT: {
      esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name("OriginFlow");
      if (set_dev_name_ret) {
        ESP_LOGE(TAG, "set device name failed, error code = %x", set_dev_name_ret);
      }
#ifdef CONFIG_SET_RAW_ADV_DATA
      esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
      if (raw_adv_ret) {
        ESP_LOGE(TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
      }
      adv_config_done |= ADV_CONFIG_FLAG;
      esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
      if (raw_scan_ret) {
        ESP_LOGE(TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
      }
      adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#else
      // config adv data
      esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
      if (ret) {
        ESP_LOGE(TAG, "config adv data failed, error code = %x", ret);
      }
      adv_config_done |= ADV_CONFIG_FLAG;
      // config scan response data
      ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
      if (ret) {
        ESP_LOGE(TAG, "config scan response data failed, error code = %x", ret);
      }
      adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#endif
      esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, 3, 0);
      if (create_attr_ret) {
        ESP_LOGE(TAG, "create attr table failed, error code = %x", create_attr_ret);
      }
    } break;
    case ESP_GATTS_READ_EVT:
      ESP_LOGI(TAG, "ESP_GATTS_READ_EVT");
      break;
    case ESP_GATTS_WRITE_EVT:
      ESP_LOGI(TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
      ble_gatt_data_call_cb(param->write.value, param->write.len);
      break;
    case ESP_GATTS_EXEC_WRITE_EVT:
      // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
      ESP_LOGI(TAG, "ESP_GATTS_EXEC_WRITE_EVT");
      break;
    case ESP_GATTS_MTU_EVT:
      ESP_LOGI(TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
      break;
    case ESP_GATTS_CONF_EVT:
      ESP_LOGI(TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
      break;
    case ESP_GATTS_START_EVT:
      ESP_LOGI(TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
      break;
    case ESP_GATTS_CONNECT_EVT:
      ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
      ESP_LOG_BUFFER_HEX(TAG, param->connect.remote_bda, 6);
      esp_ble_conn_update_params_t conn_params = {0};
      memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
      /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
      conn_params.latency = 0;
      conn_params.max_int = 0x20;  // max_int = 0x20*1.25ms = 40ms
      conn_params.min_int = 0x10;  // min_int = 0x10*1.25ms = 20ms
      conn_params.timeout = 400;   // timeout = 400*10ms = 4000ms
      // start sent the update connection parameters to the peer device.
      esp_ble_gap_update_conn_params(&conn_params);
      break;
    case ESP_GATTS_DISCONNECT_EVT:
      ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
      esp_ble_gap_start_advertising(&adv_params);
      break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
      if (param->add_attr_tab.status != ESP_GATT_OK) {
        ESP_LOGE(TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
      } else {
        ESP_LOGI(TAG, "create attribute table successfully, the number handle = %d", param->add_attr_tab.num_handle);
        esp_ble_gatts_start_service(param->add_attr_tab.handles[0]);
      }
      break;
    }
    case ESP_GATTS_STOP_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    case ESP_GATTS_UNREG_EVT:
    case ESP_GATTS_DELETE_EVT:
    default:
      break;
  }
}

void ble_gatt_init(void) {
  esp_err_t ret;

#if CONFIG_EXAMPLE_CI_PIPELINE_ID
  memcpy(test_device_name, esp_bluedroid_get_example_name(), ESP_BLE_ADV_NAME_LEN_MAX);
#endif

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
    ESP_LOGE(TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
    ESP_LOGE(TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
    return;
  }

  ret = esp_bluedroid_init();
  if (ret) {
    ESP_LOGE(TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
    return;
  }
  ret = esp_bluedroid_enable();
  if (ret) {
    ESP_LOGE(TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
    return;
  }
  // Note: Avoid performing time-consuming operations within callback functions.
  ret = esp_ble_gatts_register_callback(gatts_event_handler);
  if (ret) {
    ESP_LOGE(TAG, "gatts register error, error code = %x", ret);
    return;
  }
  ret = esp_ble_gap_register_callback(gap_event_handler);
  if (ret) {
    ESP_LOGE(TAG, "gap register error, error code = %x", ret);
    return;
  }
  ret = esp_ble_gatts_app_register(0);
  if (ret) {
    ESP_LOGE(TAG, "gatts app register error, error code = %x", ret);
    return;
  }
  esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(ESP_GATT_MAX_MTU_SIZE);
  if (local_mtu_ret) {
    ESP_LOGE(TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
  }

  return;
}

#endif
#endif