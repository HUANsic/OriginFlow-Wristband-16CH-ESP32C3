#ifndef H_CDT_PROTOCOL_H
#define H_CDT_PROTOCOL_H

#ifdef __cplusplus
#include <cstdint>
#include <string>
#else
#include <stdbool.h>
#include <stdint.h>
#endif

#define PROTOCOL_LITTLE_ENDIAN     0
#define PROTOCOL_BIG_ENDIAN        1

#define PROTOCODEC_USE_VERSION     1

#define PROTOCODEC_USE_MSGID       1

#define PROTOCODEC_USE_TIMESTAMP   1

#define PROTOCODEC_USE_DEVICE_TYPE 0

#define PROTOCOL_CRC_ENDIANNESS    PROTOCOL_BIG_ENDIAN
#define PROTOCOL_DATA_ENDIANNESS   PROTOCOL_LITTLE_ENDIAN

// 500ms
#define PROTOCOL_ACK_TIMEOUT  500

#define DT_FRAMEHEADER        0xAA
#define DT_FRAMEHEADER_LENGTH 1

#ifdef PROTOCODEC_USE_MSGID
#define DT_MSGID_LENGTH 4
#else
#define DT_MSGID_LENGTH 0
#endif

#ifdef PROTOCODEC_USE_TIMESTAMP
#define DT_TIMESTAMP_LENGTH 8
#else
#define DT_TIMESTAMP_LENGTH 0
#endif

#ifdef PROTOCODEC_USE_WORD_LEN
#define DT_LEN_LENGTH 4
#else
#define DT_LEN_LENGTH 2
#endif

#if PROTOCODEC_USE_VERSION == 1
#define DT_PROTOCOL_VERSION        "V001"
#define DT_PROTOCOL_VERSION_LENGTH 4
#else
#define DT_PROTOCOL_VERSION_LENGTH 0
#endif
#if PROTOCODEC_USE_DEVICE_TYPE == 1
#define DT_FIXED_LEN             (6 + DT_LEN_LENGTH + DT_FRAMEHEADER_LENGTH + DT_MSGID_LENGTH + DT_PROTOCOL_VERSION_LENGTH)
#define DT_FIXED_WITHOUT_CRC_LEN (4 + DT_LEN_LENGTH + DT_FRAMEHEADER_LENGTH + DT_MSGID_LENGTH + DT_PROTOCOL_VERSION_LENGTH)
#else
#define DT_FIXED_LEN             (4 + DT_LEN_LENGTH + DT_FRAMEHEADER_LENGTH + DT_MSGID_LENGTH + DT_PROTOCOL_VERSION_LENGTH + DT_TIMESTAMP_LENGTH)
#define DT_FIXED_WITHOUT_CRC_LEN (2 + DT_LEN_LENGTH + DT_FRAMEHEADER_LENGTH + DT_MSGID_LENGTH + DT_PROTOCOL_VERSION_LENGTH + DT_TIMESTAMP_LENGTH)
#endif

#if !defined(USE_ESP_LOG) && !defined(USE_STD_PRINTF) && !defined(USE_SEGGER_RTT_LOG)
#define USE_STD_PRINTF
#endif

#ifdef USE_SEGGER_RTT_LOG
#include "SEGGER_RTT.h"
#define protocol_log(fmt, ...) SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__)
#elif defined(USE_STD_PRINTF)
#include <stdio.h>
#define protocol_log(fmt, ...) printf(fmt, ##__VA_ARGS__)
#elif defined(USE_ESP_LOG)
#include "esp_log.h"
#define protocol_log(...) ESP_LOGI("PACKET", __VA_ARGS__)
#endif

#if PROTOCODEC_USE_DEVICE_TYPE == 1
/* device type */
typedef enum {
  dt_device_type_broadcast = 0,
} dt_device_type_t;
#endif

typedef enum {
  dt_file_type_firmware = 0,
  dt_file_type_resource = 1
} dt_file_type_t;

typedef enum {
  dt_checksum_crc16 = 0,
  dt_checksum_md5,
  dt_checksum_sha256
} dt_checksum_type_t;

typedef enum {
  dt_control_frame_none = 0,
  dt_control_frame_start = 1,
  dt_control_frame_next = 2,
  dt_control_frame_end = 0xFF,
} dt_control_frame_t;

typedef enum {
  dt_cmd_response = 0x00,                  /* device response command. up */
  dt_cmd_semg = 0x01,                      /* device semg command. up/downlink */
  dt_cmd_imu = 0x02,                       /* device imu command. up/downlink */
  dt_cmd_CAMERA_UP = 0x03,                 /* device camera data command. up/downlink */
  dt_cmd_CAMERA_DOWN = 0x04,               /* device camera data command. up/downlink */
  dt_cmd_VIBRATE_DATA = 0x05,              /* device vibration feedback data command. up/downlink */
  dt_cmd_BATTERY = 0x06,                   /* device battery status command. up/downlink */
  dt_cmd_data_start = 0x07E,               /* device start send data command. up/downlink */
  dt_cmd_data_end = 0x07F,                 /* device end send data command. up/downlink */
  dt_cmd_test = 0xFB,                      /* test result. uplink/downlink */
  dt_cmd_reset = 0xFD,                     /* device fatory reset command. downlink */
  dt_cmd_shutdown = 0xFE,                  /* device shutdown command. downlink */
  dt_cmd_reboot = 0xFF,                    /* device reboot command. downlink */
  dt_cmd_update_file = 0x170,              /* device update file command. downlink */
  dt_cmd_send_file = 0x172,                /* device send file command. downlink */
  dt_cmd_software_version = 0xA001,        /* device software version command. up/downlink */
  dt_cmd_device_protocol_version = 0xA003, /* device protocol version command. up/downlink */
  dt_cmd_ip = 0xA004,                      /* device ip command. up/downlink */
  dt_cmd_ble = 0xA006,                     /* device ble command. uplink/downlink */
  dt_cmd_connect = 0xA008,                 /* connect device. uplink/downlink */
  dt_cmd_disconnect = 0xA009,              /* disconnect device. uplink/downlink */
  dt_cmd_wifi = 0xA00A,                    /* device ble command. uplink/downlink */
  dt_cmd_device_connect_status = 0xA00B,   /* device ble、wifi、app连接状态. uplink */
  dt_cmd_log = 0xFFFD,                     /* error. uplink */
  dt_cmd_info = 0xFFFE,                    /* error. uplink */
  dt_cmd_error = 0xFFFF,                   /* error. uplink */
} data_transmission_command_t;

typedef enum {
  dt_imu_data_9_aix_float = 0,
  dt_imu_data_9_aix_int16,
} dt_imu_data_command_t;
typedef enum {
  dt_imu_ctrl_data_type = 0x00,  // 发送9轴数据类型，0x00:float(转换后的) 0x01:int16（RAW）
} dt_imu_ctrl_command_t;

typedef enum {
  dt_error_none = 0x0,
  dt_error_crc = 0x1,
  dt_error_package_seq = 0x2,
  dt_error_checksum = 0x3,
  dt_error_update = 0x3,
  dt_error_stm32_hardware = 0x100,
  dt_error_log = 0xFF,
} dt_error_code_t;

typedef enum {
  dt_info_charge_time = 0x01,
  dt_info_face_touch = 0x02,
  dt_info_face_no_touch = 0x03,
  dt_info_log = 0xff,
} dt_info_code_t;

typedef enum {
  dt_response_ok = 0x0,
  dt_response_ready_receive,
  dt_response_receive_next,
  dt_response_receive_retry,
  dt_response_receive_complete,
  dt_response_installing,
  dt_response_install_complete,
  dt_response_update_complete,
  dt_response_error = 0xFF,
} dt_response_t;

typedef enum {
  dt_ota_code_ok = 0x0,
  dt_ota_code_err_md5 = 0x1,
  dt_ota_code_err_download = 0x2,
  dt_ota_code_err_timeout = 0x3,
  dt_ota_code_err_version = 0x4,
  dt_ota_code_err_device_broke = 0x05,
  dt_ota_code_err_image_invalid = 0x06,
  dt_ota_code_err_flash = 0x07,
  dt_ota_code_download_finish = 0x08,
  dt_ota_code_download_progress = 0x09,
} dt_ota_code_t;

typedef enum {
  dt_ota_status_ok = 0x0,
  dt_ota_status_none,
  dt_ota_status_download_file,
  dt_ota_status_next_file,
  dt_ota_status_retry_download,
  dt_ota_status_wait_response,
  dt_ota_status_download_complete,
  dt_ota_status_installing,
  dt_ota_status_install_complete,
  dt_ota_status_update_complete,
  dt_ota_status_err = 0xff,
} dt_ota_status_t;

typedef enum {
  dt_ctrl_frame_none = 0x0,
  dt_ctrl_frame_start = 0x1,
  dt_ctrl_frame_next = 0x2,
  dt_ctrl_frame_end = 0xFF,
} dt_ctrl_frame_t;

typedef enum {
  dt_err = -1,
  dt_ok,
  dt_malloc_err,
  dt_err_len_too_little,
  dt_err_data_null,
  dt_err_frame_header,
  dt_err_crc,
  dt_err_invalid_protocol,
} dt_err_t;

typedef enum {
  dt_ble_status_closed = 0x0,
  dt_ble_status_open = 0x1,
  dt_ble_status_connected = 0x2,
  dt_ble_status_disconnect = 0x3,
} dt_ble_status_t;

typedef enum {
  dt_ble_cmd_query = 0x0,
  dt_ble_cmd_open = 0x1,
  dt_ble_cmd_close = 0x2,
} dt_ble_cmd_t;

typedef enum {
  dt_wifi_status_closed = 0x0,
  dt_wifi_status_open = 0x1,
  dt_wifi_status_connected = 0x2,
  dt_wifi_status_disconnect = 0x3,
  dt_wifi_status_not_auth = 0x4,
} dt_wifi_status_t;

typedef enum {
  dt_device_connect_status_none = 0x0,
  dt_device_wifi_connected = 0x2,
  dt_device_app_wifi_connected = 0x3,
  dt_device_ble_connected = 0x4,
  dt_device_wifi_ble_connected = 0x6,
  dt_device_app_wifi_ble_connected = 0x7,
} dt_device_connect_status_t;

typedef enum {
  dt_battery_not_charge = 0x0,
  dt_battery_charging = 0x1,
  dt_battery_charge_full = 0x2,
} dt_battery_charge_t;

/* 是否放在充电底座上 */
typedef enum {
  dt_charge_base_untouch = 0x0,
  dt_charge_base_touch = 0x1,
} dt_charge_base_touch_t;

#pragma pack(1)
typedef struct {
  uint8_t FrameHeader[DT_FRAMEHEADER_LENGTH]; /* frame header is fixed */
  uint16_t Length;                            /* 消息帧不算data为14 */
#if PROTOCODEC_USE_VERSION == 1
  uint8_t Protocol[DT_PROTOCOL_VERSION_LENGTH]; /* protocol version */
#endif
#if PROTOCODEC_USE_MSGID == 1
  uint32_t MsgID;
#endif
#if PROTOCODEC_USE_TIMESTAMP == 1
  uint8_t Timestamp[DT_TIMESTAMP_LENGTH]; /* protocol version */
#endif
#if PROTOCODEC_USE_DEVICE_TYPE == 1
  uint8_t SrcDeviceType;
  uint8_t DestDeviceType;
#endif
  uint16_t Cmd;
  uint16_t Crc16; /* crc16 modbus */
  uint8_t *Buffer;
} data_transmission_t;
#pragma pack()

/* little-endian data transfer */

#ifdef __cplusplus

namespace PROTOCODEC {
// std::string dt_ota_type_esp = "OTA_ESP";
// std::string dt_ota_type_stm = "OTA_STM";
/* class for data transmission decode and encode */
class Codec {
 public:
  Codec();
#if PROTOCODEC_USE_DEVICE_TYPE == 1
  explicit Codec(dt_device_type_t Srcdevice_type, dt_device_type_t Destdevice_type);
#endif
  ~Codec();

  /**
   * @description:  generate a packet without data
   * @param command
   * @return dt_ok is generate success
   */
  dt_err_t Generate_without_data(data_transmission_command_t command, uint8_t *out_data);
  /**
   * @description: generate a packet with data
   * @param command
   * @param *data
   * @param len
   * @return dt_ok is generate success
   */
  dt_err_t Generate_packet(data_transmission_command_t command, uint8_t *data, uint32_t len, uint8_t *out_data);

  /**
   * @description: 解析数据，并进行深拷贝到out_data
   * @param *data 数据指针
   * @param len 数据长度
   * @param *out_data 解析后的数据指针
   * @return
   */
  dt_err_t Parse_from_data(void *data, uint32_t len, uint8_t *out_data);

  /**
   * @description: 生成带有子包头的数据
   * @param command 命令
   * @param msgid 消息ID
   * @param *data 数据
   * @param len 数据长度
   * @param *subData 子包头数据
   * @param subLen 子包头长度
   * @param *out_data 缓存指针
   * @return
   */
  dt_err_t Generate_packet_with_custom_subHeader(data_transmission_command_t command, uint8_t *data, uint32_t len, uint8_t *subData, uint32_t subLen,
                                                 uint8_t *out_data);
  uint8_t *AllData();
  uint8_t *Data();
  uint32_t Len();

  /**
   * @description: 按16进制打印原始数据
   * @return
   */
  void Printf_data();

  /**
   * @description: 按字符串打印格式化后的包数据
   * @return
   */
  void Printf_format();

  /**
   * @description: 判断数据的目标是否为当前设备或者是广播包
   * @param *data
   * @param len
   * @return
   */
  bool CheckoutCurrentDeviceType(uint8_t *data, uint32_t len);

  std::string get_format_data();
  data_transmission_t dt;
  uint8_t send_fixed_buffer[DT_FIXED_LEN];
  uint8_t send_litter_buffer[200];
};
std::string hexstring_from_data(const void *data, size_t len);
}  // namespace PROTOCODEC
#endif

#ifdef __cplusplus
extern "C" {
#endif
#if PROTOCODEC_USE_DEVICE_TYPE == 1
void DT_init(data_transmission_t *dt, dt_device_type_t src_device_type, dt_device_type_t dest_device_type);
#else
void DT_init(data_transmission_t *dt);
#endif
void DT_free(data_transmission_t *dt);
/**
 * @description:  generate a packet without data
 * @param command
 * @return dt_ok is generate success
 */
dt_err_t Generate_without_data(data_transmission_t *dt, data_transmission_command_t command, uint8_t *out_data);

/**
 * @description: generate a packet with data
 * @param command
 * @param *data
 * @param len
 * @return dt_ok is generate success
 */
dt_err_t Generate_packet(data_transmission_t *dt, data_transmission_command_t command, uint8_t *data, uint32_t len, uint8_t *out_data);

dt_err_t Generate_packet_with_custom_subHeader(data_transmission_t *dt, data_transmission_command_t command, uint8_t *data, uint32_t len, uint8_t *subData,
                                               uint32_t subLen, uint8_t *out_data);

dt_err_t Parse_from_data(data_transmission_t *dt, void *data, uint32_t len, uint8_t *out_data);
uint8_t *AllData(data_transmission_t *dt);
uint8_t *Data(data_transmission_t *dt);
void Printf_data(data_transmission_t *dt);
void Printf_format(data_transmission_t *dt);
typedef int64_t (*protocol_get_timestamp)();

void set_protocol_get_timestamp(protocol_get_timestamp get_timestamp);
#ifdef __cplusplus
}
#endif

#endif