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
#define EXT_ADV_HANDLE          0
#define NUM_EXT_ADV_SET         1
#define EXT_ADV_DURATION        0
#define EXT_ADV_MAX_EVENTS      0
#define EXT_ADV_NAME_LEN_OFFSET 10
#define EXT_ADV_NAME_OFFSET     12
/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,  // slave connection min interval, Time =
                             // min_interval * 1.25 msec
    .max_interval = 0x0010,  // slave connection max interval, Time =
                             // max_interval * 1.25 msec
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
esp_ble_gap_ext_adv_params_t ext_adv_params_2M = {
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_CONNECTABLE,
    .interval_min = ESP_BLE_GAP_ADV_ITVL_MS(20),
    .interval_max = ESP_BLE_GAP_ADV_ITVL_MS(20),
    .channel_map = ADV_CHNL_ALL,
    .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .primary_phy = ESP_BLE_GAP_PHY_1M,
    .max_skip = 0,
    .secondary_phy = ESP_BLE_GAP_PHY_2M,
    .sid = 0,
    .scan_req_notif = false,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .tx_power = EXT_ADV_TX_PWR_NO_PREFERENCE,
};
static uint8_t ext_adv_raw_data[] = {
    0x02,
    ESP_BLE_AD_TYPE_FLAG,
    0x06,
    0x02,
    ESP_BLE_AD_TYPE_TX_PWR,
    0xeb,
    0x03,
    ESP_BLE_AD_TYPE_16SRV_CMPL,
    0xab,
    0xcd,
    0x11,
    ESP_BLE_AD_TYPE_NAME_CMPL,
    'O',
    'r',
    'i',
    'g',
    'i',
    'n',
    'F',
    'l',
    'o',
    'w',
    '-',
    'S',
    'E',
    'R',
    'V',
    'E',
    'R',
};

static esp_ble_gap_ext_adv_t ext_adv[1] = {
    [0] = {EXT_ADV_HANDLE, EXT_ADV_DURATION, EXT_ADV_MAX_EVENTS},
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

static char *esp_key_type_to_str(esp_ble_key_type_t key_type) {
  char *key_str = NULL;
  switch (key_type) {
    case ESP_LE_KEY_NONE:
      key_str = "ESP_LE_KEY_NONE";
      break;
    case ESP_LE_KEY_PENC:
      key_str = "ESP_LE_KEY_PENC";
      break;
    case ESP_LE_KEY_PID:
      key_str = "ESP_LE_KEY_PID";
      break;
    case ESP_LE_KEY_PCSRK:
      key_str = "ESP_LE_KEY_PCSRK";
      break;
    case ESP_LE_KEY_PLK:
      key_str = "ESP_LE_KEY_PLK";
      break;
    case ESP_LE_KEY_LLK:
      key_str = "ESP_LE_KEY_LLK";
      break;
    case ESP_LE_KEY_LENC:
      key_str = "ESP_LE_KEY_LENC";
      break;
    case ESP_LE_KEY_LID:
      key_str = "ESP_LE_KEY_LID";
      break;
    case ESP_LE_KEY_LCSRK:
      key_str = "ESP_LE_KEY_LCSRK";
      break;
    default:
      key_str = "INVALID BLE KEY TYPE";
      break;
  }

  return key_str;
}

static char *esp_auth_req_to_str(esp_ble_auth_req_t auth_req) {
  char *auth_str = NULL;
  switch (auth_req) {
    case ESP_LE_AUTH_NO_BOND:
      auth_str = "ESP_LE_AUTH_NO_BOND";
      break;
    case ESP_LE_AUTH_BOND:
      auth_str = "ESP_LE_AUTH_BOND";
      break;
    case ESP_LE_AUTH_REQ_MITM:
      auth_str = "ESP_LE_AUTH_REQ_MITM";
      break;
    case ESP_LE_AUTH_REQ_BOND_MITM:
      auth_str = "ESP_LE_AUTH_REQ_BOND_MITM";
      break;
    case ESP_LE_AUTH_REQ_SC_ONLY:
      auth_str = "ESP_LE_AUTH_REQ_SC_ONLY";
      break;
    case ESP_LE_AUTH_REQ_SC_BOND:
      auth_str = "ESP_LE_AUTH_REQ_SC_BOND";
      break;
    case ESP_LE_AUTH_REQ_SC_MITM:
      auth_str = "ESP_LE_AUTH_REQ_SC_MITM";
      break;
    case ESP_LE_AUTH_REQ_SC_MITM_BOND:
      auth_str = "ESP_LE_AUTH_REQ_SC_MITM_BOND";
      break;
    default:
      auth_str = "INVALID BLE AUTH REQ";
      break;
  }

  return auth_str;
}

static void show_bonded_devices(void) {
  int dev_num = esp_ble_get_bond_device_num();
  if (dev_num == 0) {
    ESP_LOGI(TAG, "Bonded devices number zero\n");
    return;
  }

  esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *) malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
  if (!dev_list) {
    ESP_LOGE(TAG, "malloc failed\n");
    return;
  }
  esp_ble_get_bond_device_list(&dev_num, dev_list);
  ESP_LOGI(TAG, "Bonded devices number %d", dev_num);
  for (int i = 0; i < dev_num; i++) {
    ESP_LOGI(TAG, "[%u] addr_type %u, addr " ESP_BD_ADDR_STR "", i, dev_list[i].bd_addr_type, ESP_BD_ADDR_HEX(dev_list[i].bd_addr));
  }

  free(dev_list);
}

static void __attribute__((unused)) remove_all_bonded_devices(void) {
  int dev_num = esp_ble_get_bond_device_num();
  if (dev_num == 0) {
    ESP_LOGI(TAG, "Bonded devices number zero\n");
    return;
  }

  esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *) malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
  if (!dev_list) {
    ESP_LOGE(TAG, "malloc failed\n");
    return;
  }
  esp_ble_get_bond_device_list(&dev_num, dev_list);
  for (int i = 0; i < dev_num; i++) {
    esp_ble_remove_bond_device(dev_list[i].bd_addr);
  }

  free(dev_list);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
      ESP_LOGI(TAG, "Extended advertising params set, status %d", param->ext_adv_set_params.status);
      esp_ble_gap_config_ext_adv_data_raw(EXT_ADV_HANDLE, sizeof(ext_adv_raw_data), &ext_adv_raw_data[0]);
      break;
    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
      ESP_LOGI(TAG, "Extended advertising data set, status %d", param->ext_adv_data_set.status);
      esp_ble_gap_ext_adv_start(NUM_EXT_ADV_SET, &ext_adv[0]);
      break;
    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
      ESP_LOGI(TAG, "Extended advertising start, status %d", param->ext_adv_data_set.status);
      break;
    case ESP_GAP_BLE_ADV_TERMINATED_EVT:
      ESP_LOGI(TAG, "Extended advertising terminated, status %d", param->adv_terminate.status);
      if (param->adv_terminate.status == 0x00) {
        ESP_LOGI(TAG, "Advertising successfully ended with a connection being created");
      }
      break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT: /* passkey request event */
      /* Call the following function to input the passkey which is displayed on
       * the remote device */
      ESP_LOGI(TAG, "Passkey request");
      // esp_ble_passkey_reply(heart_rate_profile_tab[HEART_PROFILE_APP_IDX].remote_bda,
      // true, 0x00);
      break;
    case ESP_GAP_BLE_OOB_REQ_EVT: {
      ESP_LOGI(TAG, "OOB request");
      uint8_t tk[16] = {1};  // If you paired with OOB, both devices need to use the same tk
      esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk, sizeof(tk));
      break;
    }
    case ESP_GAP_BLE_LOCAL_IR_EVT: /* BLE local IR event */
      ESP_LOGI(TAG, "Local identity root");
      break;
    case ESP_GAP_BLE_LOCAL_ER_EVT: /* BLE local ER event */
      ESP_LOGI(TAG, "Local encryption root");
      break;
    case ESP_GAP_BLE_NC_REQ_EVT:
      /* The app will receive this evt when the IO has DisplayYesNO capability
      and the peer device IO also has DisplayYesNo capability. show the passkey
      number to the user to confirm it with the number displayed by peer device.
    */
      esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
      ESP_LOGI(TAG, "Numeric Comparison request, passkey %" PRIu32, param->ble_security.key_notif.passkey);
      break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
      /* send the positive(true) security response to the peer device to accept
      the security request. If not accept the security request, should send the
      security response with negative(false) accept value*/
      esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
      break;
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:  /// the app will receive this evt when
                                         /// the IO  has Output capability and
                                         /// the peer device IO has Input
                                         /// capability.
      /// show the passkey number to the user to input it in the peer device.
      ESP_LOGI(TAG, "Passkey notify, passkey %06" PRIu32, param->ble_security.key_notif.passkey);
      break;
    case ESP_GAP_BLE_KEY_EVT:
      // shows the ble key info share with peer device to the user.
      ESP_LOGI(TAG, "Key exchanged, key_type %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
      if (param->ble_security.ble_key.key_type == ESP_LE_KEY_PID) {
        ESP_LOGI(TAG, "peer addr " ESP_BD_ADDR_STR "", ESP_BD_ADDR_HEX(param->ble_security.ble_key.bd_addr));
      }
      break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
      esp_bd_addr_t bd_addr;
      memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
      ESP_LOGI(TAG, "Authentication complete, addr_type %u, addr " ESP_BD_ADDR_STR "", param->ble_security.auth_cmpl.addr_type, ESP_BD_ADDR_HEX(bd_addr));
      if (!param->ble_security.auth_cmpl.success) {
        ESP_LOGI(TAG, "Pairing failed, reason 0x%x", param->ble_security.auth_cmpl.fail_reason);
      } else {
        ESP_LOGI(TAG, "Pairing successfully, auth_mode %s", esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));
      }
      show_bonded_devices();
      break;
    }
    case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT: {
      ESP_LOGD(TAG, "Bond device remove, status %d, device " ESP_BD_ADDR_STR "", param->remove_bond_dev_cmpl.status,
               ESP_BD_ADDR_HEX(param->remove_bond_dev_cmpl.bd_addr));
      break;
    }
    case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
      ESP_LOGI(TAG, "Local privacy config, status %x", param->local_privacy_cmpl.status);
      esp_ble_gap_ext_adv_set_params(EXT_ADV_HANDLE, &ext_adv_params_2M);
      break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
      ESP_LOGI(TAG,
               "Connection params update, status %d, conn_int %d, latency %d, "
               "timeout %d",
               param->update_conn_params.status, param->update_conn_params.conn_int, param->update_conn_params.latency, param->update_conn_params.timeout);
      break;
    default:
      break;
  }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
  switch (event) {
    case ESP_GATTS_REG_EVT: {
      esp_ble_gap_config_local_privacy(true);
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
      // the length of gattc prepare write data must be less than
      // GATTS_DEMO_CHAR_VAL_LEN_MAX.
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
      /* For the iOS system, please refer to Apple official documents about the
       * BLE connection parameters restrictions. */
      conn_params.latency = 0;
      conn_params.max_int = 0x20;  // max_int = 0x20*1.25ms = 40ms
      conn_params.min_int = 0x10;  // min_int = 0x10*1.25ms = 20ms
      conn_params.timeout = 400;   // timeout = 400*10ms = 4000ms
      // start sent the update connection parameters to the peer device.
      esp_ble_gap_update_conn_params(&conn_params);
      break;
    case ESP_GATTS_DISCONNECT_EVT:
      ESP_LOGI(TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
      ESP_LOGI(TAG, "Disconnected, remote " ESP_BD_ADDR_STR ", reason 0x%x", ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
      /* start advertising again when missing the connect */
      esp_ble_gap_ext_adv_start(NUM_EXT_ADV_SET, &ext_adv[0]);
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