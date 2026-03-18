#include "sdkconfig.h"
#if CONFIG_BT_ENABLED
#if CONFIG_BT_NIMBLE_ENABLED
#include "common.h"
#include "gatt_svc.h"
#include "gatts.h"

/* Private function declarations */
static int ble_tx_rx_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

/* Private variables */
/* Heart rate service */
static const ble_uuid128_t ble_tx_rx_svc_uuid =
    BLE_UUID128_INIT(0xFB, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x01);

static uint8_t ble_tx_rx_chr_val[2] = {0};
static uint16_t ble_tx_rx_chr_val_handle;

static uint16_t ble_tx_rx_chr_conn_handle = 0;
static bool ble_tx_rx_chr_conn_handle_inited = false;
static bool ble_tx_rx_ind_status = false;

static const ble_uuid128_t ble_tx_rx_chr_uuid =
    BLE_UUID128_INIT(0xFB, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x02);
static ble_gatt_cb ble_gatt_cb_ = NULL;
void ble_gatt_set_cb(ble_gatt_cb cb) {
  ble_gatt_cb_ = cb;
}
static void ble_gatt_data_call_cb(uint8_t *data, uint32_t len) {
  if (ble_gatt_cb_) {
    ble_gatt_cb_(data, len);
  }
}

/* GATT services table */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    /* Heart rate service */
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = &ble_tx_rx_svc_uuid.u,
     .characteristics = (struct ble_gatt_chr_def[]){{/* Heart rate characteristic */
                                                     .uuid = &ble_tx_rx_chr_uuid.u,
                                                     .access_cb = ble_tx_rx_chr_access,
                                                     .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_WRITE,
                                                     .val_handle = &ble_tx_rx_chr_val_handle},
                                                    {
                                                        0, /* No more characteristics in this service. */
                                                    }}},

    {
     0, /* No more services. */
    },
};

/* Private functions */
static int ble_tx_rx_chr_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
  /* Local variables */
  int rc = 0;

  /* Handle access events */
  /* Note: Heart rate characteristic is read only */
  switch (ctxt->op) {
    /* Read characteristic event */
    case BLE_GATT_ACCESS_OP_READ_CHR:
      /* Verify connection handle */
      if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ESP_LOGI(TAG, "characteristic read; conn_handle=%d attr_handle=%d", conn_handle, attr_handle);
      } else {
        ESP_LOGI(TAG, "characteristic read by nimble stack; attr_handle=%d", attr_handle);
      }

      /* Verify attribute handle */
      if (attr_handle == ble_tx_rx_chr_val_handle) {
        /* Update access buffer value */
        ble_tx_rx_chr_val[1] = rand() % 100;
        rc = os_mbuf_append(ctxt->om, &ble_tx_rx_chr_val, sizeof(ble_tx_rx_chr_val));
        return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
      }
      goto error;
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
      if (ctxt->om->om_len > 0) {
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, ctxt->om->om_data, ctxt->om->om_len, ESP_LOG_INFO);
        ble_gatt_data_call_cb(ctxt->om->om_data, ctxt->om->om_len);
      }

      return 0;
    /* Unknown event */
    default:
      goto error;
  }

error:
  ESP_LOGE(TAG, "unexpected access operation to heart rate characteristic, opcode: %d", ctxt->op);
  return BLE_ATT_ERR_UNLIKELY;
}

/* Public functions */
void send_ble_tx_rx_indication(void) {
  if (ble_tx_rx_ind_status && ble_tx_rx_chr_conn_handle_inited) {
    ble_gatts_indicate(ble_tx_rx_chr_conn_handle, ble_tx_rx_chr_val_handle);
  }
}

void ble_tx_rx_send_notify(uint8_t *data, uint32_t len) {
  if (ble_tx_rx_ind_status && ble_tx_rx_chr_conn_handle_inited) {
    struct os_mbuf *om;
    om = ble_hs_mbuf_from_flat(data, len);
    if (om == NULL || ble_gatts_notify_custom(ble_tx_rx_chr_conn_handle, ble_tx_rx_chr_val_handle, om) != 0) {
      ESP_LOGE(TAG, "%s failed", __func__);
    } else {
      ESP_LOGD(TAG, "%s success", __func__);
    }
  }
}

/*
 *  Handle GATT attribute register events
 *      - Service register event
 *      - Characteristic register event
 *      - Descriptor register event
 */
void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
  /* Local variables */
  char buf[BLE_UUID_STR_LEN];

  /* Handle GATT attributes register events */
  switch (ctxt->op) {
    /* Service register event */
    case BLE_GATT_REGISTER_OP_SVC:
      ESP_LOGI(TAG, "registered service %s with handle=%d", ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf), ctxt->svc.handle);
      break;

    /* Characteristic register event */
    case BLE_GATT_REGISTER_OP_CHR:
      ESP_LOGI(TAG,
               "registering characteristic %s with "
               "def_handle=%d val_handle=%d",
               ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf), ctxt->chr.def_handle, ctxt->chr.val_handle);
      break;

    /* Descriptor register event */
    case BLE_GATT_REGISTER_OP_DSC:
      ESP_LOGI(TAG, "registering descriptor %s with handle=%d", ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf), ctxt->dsc.handle);
      break;

    /* Unknown event */
    default:
      assert(0);
      break;
  }
}

/*
 *  GATT server subscribe event callback
 *      1. Update heart rate subscription status
 */

void gatt_svr_subscribe_cb(struct ble_gap_event *event) {
  /* Check connection handle */
  if (event->subscribe.conn_handle != BLE_HS_CONN_HANDLE_NONE) {
    ESP_LOGI(TAG, "subscribe event; conn_handle=%d attr_handle=%d", event->subscribe.conn_handle, event->subscribe.attr_handle);
  } else {
    ESP_LOGI(TAG, "subscribe by nimble stack; attr_handle=%d", event->subscribe.attr_handle);
  }
  ESP_LOGI(TAG, "subscribe cur_notify=%d, event->subscribe.attr_handle:%d,ble_tx_rx_chr_val_handle:%d", event->subscribe.cur_notify,
           event->subscribe.attr_handle, ble_tx_rx_chr_val_handle);
  /* Check attribute handle */
  if (event->subscribe.attr_handle == ble_tx_rx_chr_val_handle) {
    /* Update heart rate subscription status */
    ble_tx_rx_chr_conn_handle = event->subscribe.conn_handle;
    ble_tx_rx_chr_conn_handle_inited = true;
    ble_tx_rx_ind_status = event->subscribe.cur_notify;
  }
}

/*
 *  GATT server initialization
 *      1. Initialize GATT service
 *      2. Update NimBLE host GATT services counter
 *      3. Add GATT services to server
 */
int gatt_svc_init(void) {
  /* Local variables */
  int rc = 0;

  /* 1. GATT service initialization */
  ble_svc_gatt_init();

  /* 2. Update GATT services counter */
  rc = ble_gatts_count_cfg(gatt_svr_svcs);
  if (rc != 0) {
    return rc;
  }

  /* 3. Add GATT services */
  rc = ble_gatts_add_svcs(gatt_svr_svcs);
  if (rc != 0) {
    return rc;
  }

  return 0;
}

#endif
#endif