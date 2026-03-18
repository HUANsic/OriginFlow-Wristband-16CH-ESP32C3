#include <format>
#ifndef __cplusplus
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#else
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#endif

#include <map>
#include <string>

#include "crc.h"
#include "protocol.h"
#define TAG            "Codec"
#define I2S_func(args) args = #args

static uint32_t msg_id = 0;
static void intToByte(int64_t v, uint8_t *out, int byteLen, bool isBigEndian) {
  int i = 0;
  if (isBigEndian) {
    for (i = byteLen - 1; i >= 0; i--) {
      out[i] = (uint8_t) (v >> (byteLen - 1 - i) * 8);
    }
  } else {
    for (i = 0; i < byteLen; i++) {
      out[i] = (uint8_t) (v >> i * 8);
    }
  }
}
std::map<dt_err_t, std::string> dt_err_map = {
    {dt_err,                  "dt_err"                 },
    {dt_ok,                   "dt_ok"                  },
    {dt_malloc_err,           "dt_malloc_err"          },
    {dt_err_len_too_little,   "dt_err_len_too_little"  },
    {dt_err_data_null,        "dt_err_data_null"       },
    {dt_err_frame_header,     "dt_err_frame_header"    },
    {dt_err_crc,              "dt_err_crc"             },
    {dt_err_invalid_protocol, "dt_err_invalid_protocol"},
};

std::map<uint16_t, std::string> dt_cmd_map = {
    {dt_cmd_response,                "dt_cmd_response"               },
    {dt_cmd_semg,                    "dt_cmd_semg"                   },
    {dt_cmd_imu,                     "dt_cmd_imu"                    },
    {dt_cmd_CAMERA_UP,               "dt_cmd_CAMERA_UP"              },
    {dt_cmd_CAMERA_DOWN,             "dt_cmd_CAMERA_DOWN"            },
    {dt_cmd_VIBRATE_DATA,            "dt_cmd_VIBRATE_DATA"           },
    {dt_cmd_BATTERY,                 "dt_cmd_BATTERY"                },
    {dt_cmd_end,                     "dt_cmd_end"                    },
    {dt_cmd_test,                    "dt_cmd_test"                   },
    {dt_cmd_reset,                   "dt_cmd_reset"                  },
    {dt_cmd_shutdown,                "dt_cmd_shutdown"               },
    {dt_cmd_reboot,                  "dt_cmd_reboot"                 },
    {dt_cmd_update_file,             "dt_cmd_update_file"            },
    {dt_cmd_send_file,               "dt_cmd_send_file"              },
    {dt_cmd_software_version,        "dt_cmd_software_version"       },
    {dt_cmd_device_protocol_version, "dt_cmd_device_protocol_version"},
    {dt_cmd_ip,                      "dt_cmd_ip"                     },
    {dt_cmd_ble,                     "dt_cmd_ble"                    },
    {dt_cmd_connect,                 "dt_cmd_connect"                },
    {dt_cmd_disconnect,              "dt_cmd_disconnect"             },
    {dt_cmd_wifi,                    "dt_cmd_wifi"                   },
    {dt_cmd_device_connect_status,   "dt_cmd_device_connect_status"  },
    {dt_cmd_info,                    "dt_cmd_info"                   },
    {dt_cmd_error,                   "dt_cmd_error"                  },
};
#if PROTOCODEC_USE_DEVICE_TYPE == 1
std::map<uint16_t, std::string> dt_device_type_map = {
    {dt_device_type_broadcast, "dt_device_type_broadcast"},
};
#endif

void byte_copy(uint8_t *out, uint8_t *in, int len, bool BigEndian) {
  if (BigEndian) {
    while (len--) {
      *out++ = *(in + len);
    }
  } else {
    while (len--) {
      *out++ = *in++;
    }
  }
}

int64_t byteToInt(uint8_t *b, int byteLen, bool isBigEndian) {
  int64_t result = 0;
  int i = 0;
  if (isBigEndian) {
    for (i = byteLen - 1; i >= 0; i--) {
      result += (int64_t) b[i] << (byteLen - 1 - i) * 8;
    }
  } else {
    for (i = 0; i < byteLen; i++) {
      result += (int64_t) b[i] << i * 8;
    }
  }
  return result;
}

void hexstring_from_data(const void *data, size_t len, char *output) {
  const auto *buf = (const unsigned char *) data;
  size_t i, j;
  for (i = j = 0; i < len; ++i) {
    char c;
    c = (char) ((buf[i] >> 4) & 0xf);
    c = (c > 9) ? c + 'a' - 10 : c + '0';
    output[j++] = c;
    c = (char) ((buf[i] & 0xf));
    c = (c > 9) ? c + 'a' - 10 : c + '0';
    output[j++] = c;
  }
}
char const hex[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

std::string byte_2_str(uint8_t *bytes, int size) {
  std::string str;
  for (int i = 0; i < size; ++i) {
    const uint8_t ch = bytes[i];
    str.append(&hex[(ch & 0xF0) >> 4], 1);
    str.append(&hex[ch & 0xF], 1);
  }
  return str;
}
std::string hexstring_from_data(const void *data, size_t len) {
  if (len == 0) {
    return std::string();
  }
  std::string result;
  result.resize(len * 2);
  hexstring_from_data(data, len, (char *) result.data());
  return result;
}

namespace PROTOCODEC {
Codec::Codec() {
  memset(&dt, 0, sizeof(data_transmission_t));
  dt.FrameHeader[0] = DT_FRAMEHEADER;
  dt.Buffer = nullptr;
  dt.Length = 0;
#if PROTOCODEC_USE_DEVICE_TYPE == 1
  dt.DestDeviceType = dt_device_type_broadcast;
#endif
}

#if PROTOCODEC_USE_DEVICE_TYPE == 1
Codec::Codec(dt_device_type_t Srcdevice_type, dt_device_type_t Destdevice_type) {
  memset(&dt, 0, sizeof(data_transmission_t));
  dt.Buffer = nullptr;
  dt.Length = 0;
  dt.SrcDeviceType = Srcdevice_type;
  dt.DestDeviceType = Destdevice_type;
}
#endif

Codec::~Codec() {
  dt.Buffer = nullptr;
  dt.Length = 0;
}
uint8_t *Codec::Data() {
  if (dt.Buffer != nullptr) {
    return dt.Buffer + DT_FIXED_WITHOUT_CRC_LEN;
  }
  return nullptr;
}
uint8_t *Codec::AllData() {
  return dt.Buffer;
}
uint32_t Codec::Len() {
  return dt.Length;
}

void Codec::Printf_data() {
  protocol_log("[%s] data is :\t", TAG);
  for (uint32_t i = 0; i < dt.Length; i++) {
    protocol_log("%02x ", dt.Buffer[i]);
  }
  protocol_log("\r\n");
}
// TODO:这个方法添加判断#if PROTOCODEC_USE_DEVICE_TYPE == 1
void Codec::Printf_format() {
  protocol_log("%s\r\n", get_format_data().c_str());
}

std::string Codec::get_format_data() {
  std::string str;
  str += "frame start is {";
  // str += std::string((char *) dt.FrameHeader, sizeof(dt.FrameHeader));
  for (int i = 0; i < sizeof(dt.FrameHeader); i++) {
    str += std::format("{:02X}", dt.FrameHeader[i]);
    str += " ";
  }
#if PROTOCOL_USE_VERSION == 1
  str += "}.\r\nprotocol version is {";
  str += std::string((char *) dt.Protocol, sizeof(dt.Protocol));
#endif
  str += "}.\r\npacket size is {";
  str += std::to_string(dt.Length);
#if PROTOCODEC_USE_MSGID == 1
  str += "}.\r\nmsg id is {";
  str += std::to_string(dt.MsgID);
#endif
#if PROTOCODEC_USE_DEVICE_TYPE == 1
  str += "}.\r\nsource device type is {";
  str += std::string(dt_device_type_map[dt.SrcDeviceType]);
  str += "}.\r\ndestination device type is {";
  str += std::string(dt_device_type_map[dt.DestDeviceType]);
#endif
  str += "}.\r\ncommand is {";
  str += std::to_string(dt.Cmd);
  str += ":";
  str += std::string(dt_cmd_map[dt.Cmd]);
  str += "}.\r\ncrc is {";
  str += std::format("{:X}", dt.Crc16);
  str += "}\r\n";
  if (dt.Length - 2 > DT_FIXED_WITHOUT_CRC_LEN) {
    str += "packet data(option) is ";
    std::string option = byte_2_str(dt.Buffer, dt.Length);
    str += option + "\r\n";
  }
  if (dt.Length > 6) {
    str += "end is " + std::format("{:02X}", dt.Buffer[dt.Length - 6]) + " " + std::format("{:02X}", dt.Buffer[dt.Length - 5]) + " " +
           std::format("{:02X}", dt.Buffer[dt.Length - 4]) + " " + std::format("{:02X}", dt.Buffer[dt.Length - 3]) + " " +
           std::format("{:02X}", dt.Buffer[dt.Length - 2]) + " " + std::format("{:02X}", dt.Buffer[dt.Length - 1]) + "\r\n";
  }
  return str;
}

dt_err_t Codec::Generate_without_data(data_transmission_command_t command, uint8_t *out_data) {
  // byte_copy(dt.FrameHeader, (uint8_t *) DT_FRAMEHEADER, sizeof(dt.FrameHeader), false);
  if (out_data == nullptr) {
    return dt_malloc_err;
  }
#if PROTOCODEC_USE_VERSION == 1
  byte_copy(dt.Protocol, (uint8_t *) DT_PROTOCOL_VERSION, sizeof(dt.Protocol), false);
#endif
  dt.Buffer = out_data;

  dt.Length = DT_FIXED_LEN;
#ifdef PROTOCODEC_USE_MSGID
  dt.MsgID = msg_id++;
#endif
  dt.Cmd = command;
  memcpy(dt.Buffer, (uint8_t *) &dt, dt.Length);
#ifdef PROTOCODEC_USE_MSGID
#if PROTOCOL_DATA_ENDIANNESS == PROTOCOL_BIG_ENDIAN
  intToByte(dt.MsgID, dt.Buffer + DT_FRAMEHEADER_LENGTH + DT_LEN_LENGTH + DT_PROTOCOL_VERSION_LENGTH, 4, true);
#else
  intToByte(dt.MsgID, dt.Buffer + DT_FRAMEHEADER_LENGTH + DT_LEN_LENGTH + DT_PROTOCOL_VERSION_LENGTH, 4, false);
#endif
#endif
  /* crc16 is 2 byte */
  dt.Crc16 = CRC16(dt.Buffer, dt.Length - 2);
#if PROTOCOL_CRC_ENDIANNESS == PROTOCOL_BIG_ENDIAN
  dt.Buffer[dt.Length - 1] = dt.Crc16 & 0xff;
  dt.Buffer[dt.Length - 2] = dt.Crc16 >> 8;
#else
  dt.Buffer[dt.Length - 2] = dt.Crc16 & 0xff;
  dt.Buffer[dt.Length - 1] = dt.Crc16 >> 8;
#endif
  return dt_ok;
}

dt_err_t Codec::Generate_packet(data_transmission_command_t command, uint8_t *data, uint32_t len, uint8_t *out_data) {
  if (data == nullptr || len == 0) {
    /* data is null or len is 0*/
    protocol_log("[%s]%s generate packet failure. data or len is not match\r\n", TAG, __func__);
    return dt_err;
  }
  // byte_copy(dt.FrameHeader, (uint8_t *) DT_FRAMEHEADER, sizeof(dt.FrameHeader), false);
#if PROTOCODEC_USE_VERSION == 1
  byte_copy(dt.Protocol, (uint8_t *) DT_PROTOCOL_VERSION, sizeof(dt.Protocol), false);
#endif
  dt.Length = DT_FIXED_LEN + len;

  dt.Cmd = command;
  dt.Buffer = out_data;
  if (dt.Buffer == nullptr) {
    protocol_log("[%s]%s malloc is failure\r\n", TAG, __func__);
    return dt_malloc_err;
  }
#ifdef PROTOCODEC_USE_MSGID
  dt.MsgID = msg_id++;
#endif
  memcpy(dt.Buffer, (uint8_t *) &dt, DT_FIXED_WITHOUT_CRC_LEN);
  memcpy(dt.Buffer + DT_FIXED_WITHOUT_CRC_LEN, data, len);
#ifdef PROTOCODEC_USE_MSGID
#if PROTOCOL_DATA_ENDIANNESS == PROTOCOL_BIG_ENDIAN
  intToByte(dt.MsgID, dt.Buffer + DT_FRAMEHEADER_LENGTH + DT_LEN_LENGTH + DT_PROTOCOL_VERSION_LENGTH, 4, true);
#else
  intToByte(dt.MsgID, dt.Buffer + DT_FRAMEHEADER_LENGTH + DT_LEN_LENGTH + DT_PROTOCOL_VERSION_LENGTH, 4, false);
#endif
#endif
  /* crc16 is 2 byte */
  dt.Crc16 = CRC16(dt.Buffer, dt.Length - 2);
#if PROTOCOL_CRC_ENDIANNESS == PROTOCOL_BIG_ENDIAN
  dt.Buffer[dt.Length - 1] = dt.Crc16 & 0xff;
  dt.Buffer[dt.Length - 2] = dt.Crc16 >> 8;
#else
  dt.Buffer[dt.Length - 2] = dt.Crc16 & 0xff;
  dt.Buffer[dt.Length - 1] = dt.Crc16 >> 8;
#endif
  return dt_ok;
}

dt_err_t Codec::Generate_packet_with_custom_subHeader(data_transmission_command_t command, uint8_t *data, uint32_t len, uint8_t *subData, uint32_t subLen,
                                                      uint8_t *out_data) {
  if (data == nullptr || len == 0) {
    /* data is null or len is 0*/
    protocol_log("[%s]%s generate packet failure. data or len is not match\r\n", TAG, __func__);
    return dt_err;
  }
  // byte_copy(dt.FrameHeader, (uint8_t *) DT_FRAMEHEADER, sizeof(dt.FrameHeader), false);
#if PROTOCODEC_USE_VERSION == 1
  byte_copy(dt.Protocol, (uint8_t *) DT_PROTOCOL_VERSION, sizeof(dt.Protocol), false);
#endif
  dt.Length = DT_FIXED_LEN + len + subLen;
#ifdef PROTOCODEC_USE_MSGID
  dt.MsgID = msg_id++;
#endif
  dt.Cmd = command;
  dt.Buffer = out_data;
  if (dt.Buffer == nullptr) {
    protocol_log("[%s]%s malloc is failure\r\n", TAG, __func__);
    return dt_malloc_err;
  }
  memcpy(dt.Buffer, (uint8_t *) &dt, DT_FIXED_WITHOUT_CRC_LEN);
#ifdef PROTOCODEC_USE_MSGID
#if PROTOCOL_DATA_ENDIANNESS == PROTOCOL_BIG_ENDIAN
  intToByte(dt.MsgID, dt.Buffer + DT_FRAMEHEADER_LENGTH + DT_LEN_LENGTH + DT_PROTOCOL_VERSION_LENGTH, 4, true);
#else
  intToByte(dt.MsgID, dt.Buffer + DT_FRAMEHEADER_LENGTH + DT_LEN_LENGTH + DT_PROTOCOL_VERSION_LENGTH, 4, false);
#endif
#endif

  memcpy(dt.Buffer + DT_FIXED_WITHOUT_CRC_LEN, subData, subLen);
  memcpy(dt.Buffer + DT_FIXED_WITHOUT_CRC_LEN + subLen, data, len);
  /* crc16 is 2 byte */
  dt.Crc16 = CRC16(dt.Buffer, dt.Length - 2);
#if PROTOCOL_CRC_ENDIANNESS == PROTOCOL_BIG_ENDIAN
  dt.Buffer[dt.Length - 2] = dt.Crc16 & 0xff;
  dt.Buffer[dt.Length - 1] = dt.Crc16 >> 8;
#else
  dt.Buffer[dt.Length - 1] = dt.Crc16 & 0xff;
  dt.Buffer[dt.Length - 2] = dt.Crc16 >> 8;
#endif
  return dt_ok;
}

dt_err_t Codec::Parse_from_data(void *data, uint32_t len, uint8_t *out_data) {
  auto *msg = (uint8_t *) data;
  if (len < DT_FIXED_LEN) {
    protocol_log("[%s]%s malloc is failure\r\n", TAG, __func__);
    return dt_err_len_too_little;
  }
  if (data == nullptr) {
    protocol_log("[%s]%s malloc is failure\r\n", TAG, __func__);
    return dt_err_data_null;
  }
  uint16_t crc_src = *(msg + len - 2) + ((uint16_t) * (msg + len - 1) << 8);
  uint16_t crc_dest = CRC16(msg, len - 2);
#if PROTOCOL_CRC_ENDIANNESS == PROTOCOL_BIG_ENDIAN
  crc_dest = (crc_dest & 0xff) << 8 | (crc_dest >> 8);
#endif
  if (crc_src != crc_dest) {
    protocol_log("[%s]%s crc is error %04X,%04X\r\n", TAG, __func__, crc_src, crc_dest);
    return dt_err_crc;
  }
#if PROTOCODEC_USE_VERSION == 1
  /* 匹配协议版本 */
  if (strncmp(DT_PROTOCOL_VERSION, (char *) msg + 2, 4) == 0) {
  } else {
    return dt_err_invalid_protocol;
  }
#endif
  byte_copy(dt.FrameHeader, msg, sizeof(dt.FrameHeader), false);
  msg += sizeof(dt.FrameHeader);
#ifdef PROTOCOL_USE_VERSION
  byte_copy(dt.Protocol, msg, 4, false);
  msg += sizeof(dt.Protocol);
#endif
  dt.Length = byteToInt(msg, 2, false);
  msg += sizeof(dt.Length);
#ifdef PROTOCODEC_USE_MSGID
  dt.MsgID = byteToInt(msg, 4, false);
  msg += sizeof(dt.MsgID);
#endif
#if PROTOCODEC_USE_DEVICE_TYPE == 1
  dt.SrcDeviceType = byteToInt(msg, 2, false);
  msg += sizeof(dt.SrcDeviceType);
  dt.DestDeviceType = byteToInt(msg, 2, false);
  msg += sizeof(dt.DestDeviceType);
#endif
  dt.Cmd = byteToInt(msg, 2, false);
  dt.Buffer = out_data;
  if (dt.Buffer == nullptr) {
    protocol_log("[%s]%s malloc is failure\r\n", TAG, __func__);
    return dt_malloc_err;
  }
  memcpy(dt.Buffer, (uint8_t *) &dt, DT_FIXED_WITHOUT_CRC_LEN);
  memcpy(dt.Buffer, data, dt.Length);
  /* crc16 is 2 byte */
  dt.Crc16 = CRC16(dt.Buffer, dt.Length - 2);
#if PROTOCOL_CRC_ENDIANNESS == PROTOCOL_BIG_ENDIAN
  dt.Buffer[dt.Length - 2] = dt.Crc16 & 0xff;
  dt.Buffer[dt.Length - 1] = dt.Crc16 >> 8;
#else
  dt.Buffer[dt.Length - 1] = dt.Crc16 & 0xff;
  dt.Buffer[dt.Length - 2] = dt.Crc16 >> 8;
#endif
  return dt_ok;
}
}  // namespace PROTOCODEC

#if PROTOCODEC_USE_DEVICE_TYPE == 1
void DT_init(data_transmission_t *dt, dt_device_type_t src_device_type, dt_device_type_t dest_device_type) {
  memset(dt, 0, sizeof(data_transmission_t));
  dt->Buffer = nullptr;
  dt->SrcDeviceType = src_device_type;
  dt->DestDeviceType = dest_device_type;
}
#else
void DT_init(data_transmission_t *dt) {
  memset(dt, 0, sizeof(data_transmission_t));
  dt->Buffer = nullptr;
}
#endif

void DT_free(data_transmission_t *dt) {
  dt->Buffer = nullptr;
  dt->Length = 0;
}

uint8_t *Data(data_transmission_t *dt) {
  if (dt->Buffer != nullptr) {
    return dt->Buffer + DT_FIXED_WITHOUT_CRC_LEN;
  }
  return nullptr;
}

uint8_t *AllData(data_transmission_t *dt) {
  return dt->Buffer;
}

void protocol_log_data(data_transmission_t *dt) {
  protocol_log("[%s] data is :\t", TAG);
  for (uint32_t i = 0; i < dt->Length; i++) {
    protocol_log("%02x ", dt->Buffer[i]);
  }
  protocol_log("\r\n");
}

dt_err_t Generate_without_data(data_transmission_t *dt, data_transmission_command_t command, uint8_t *out_data) {
  // byte_copy(dt->FrameHeader, (uint8_t *) DT_FRAMEHEADER, sizeof(dt->FrameHeader), false);
#if PROTOCODEC_USE_VERSION == 1
  byte_copy(dt->Protocol, (uint8_t *) DT_PROTOCOL_VERSION, sizeof(dt->Protocol), false);
#endif
  dt->Length = DT_FIXED_LEN;
#ifdef PROTOCODEC_USE_MSGID
  dt->MsgID = msg_id++;
#endif
  dt->Cmd = command;
  /* crc16 is 2 byte */
  dt->Crc16 = CRC16((uint8_t *) dt, DT_FIXED_WITHOUT_CRC_LEN);
#if PROTOCOL_CRC_ENDIANNESS == PROTOCOL_BIG_ENDIAN
  dt->Crc16 = (dt->Crc16 & 0xff) << 8 | (dt->Crc16 >> 8);
#endif
  dt->Buffer = out_data;
  if (dt->Buffer == nullptr) {
    protocol_log("[%s]%s malloc is failure\r\n", TAG, __func__);
    return dt_malloc_err;
  }
  memcpy(dt->Buffer, dt, dt->Length);
  return dt_ok;
}

dt_err_t Generate_packet(data_transmission_t *dt, data_transmission_command_t command, uint8_t *data, uint32_t len, uint8_t *out_data) {
  if (data == nullptr || len == 0) {
    /* data is null or len is 0*/
    protocol_log("[%s]%s generate packet failure. data or len is not match\r\n", TAG, __func__);
    return dt_err;
  }
  // byte_copy(dt->FrameHeader, (uint8_t *) DT_FRAMEHEADER, sizeof(dt->FrameHeader), false);
#if PROTOCODEC_USE_VERSION == 1
  byte_copy(dt->Protocol, (uint8_t *) DT_PROTOCOL_VERSION, sizeof(dt->Protocol), false);
#endif
  dt->Length = DT_FIXED_LEN + len;
#ifdef PROTOCODEC_USE_MSGID
  dt->MsgID = msg_id++;
#endif
  dt->Cmd = command;
  dt->Buffer = out_data;
  if (dt->Buffer == nullptr) {
    protocol_log("[%s]%s malloc is failure\r\n", TAG, __func__);
    return dt_malloc_err;
  }
  memcpy(dt->Buffer, dt, DT_FIXED_WITHOUT_CRC_LEN);
  memcpy(dt->Buffer + DT_FIXED_WITHOUT_CRC_LEN, data, len);
  /* crc16 is 2 byte */
  dt->Crc16 = CRC16(dt->Buffer, dt->Length - 2);
#if PROTOCOL_CRC_ENDIANNESS == PROTOCOL_BIG_ENDIAN
  dt->Buffer[dt->Length - 2] = dt->Crc16 & 0xff;
  dt->Buffer[dt->Length - 1] = dt->Crc16 >> 8;
#else
  dt->Buffer[dt->Length - 1] = dt->Crc16 & 0xff;
  dt->Buffer[dt->Length - 2] = dt->Crc16 >> 8;
#endif
  return dt_ok;
}

dt_err_t Generate_packet_with_custom_subHeader(data_transmission_t *dt, data_transmission_command_t command, uint8_t *data, uint32_t len, uint8_t *subData,
                                               uint32_t subLen, uint8_t *out_data) {
  if (data == nullptr || len == 0) {
    /* data is null or len is 0*/
    protocol_log("[%s]%s generate packet failure. data or len is not match\r\n", TAG, __func__);
    return dt_err;
  }
  // byte_copy(dt->FrameHeader, (uint8_t *) DT_FRAMEHEADER, sizeof(dt->FrameHeader), false);
#if PROTOCODEC_USE_VERSION == 1
  byte_copy(dt->Protocol, (uint8_t *) DT_PROTOCOL_VERSION, sizeof(dt->Protocol), false);
#endif
  dt->Length = DT_FIXED_LEN + len + subLen;
#ifdef PROTOCODEC_USE_MSGID
  dt->MsgID = msg_id++;
#endif
  dt->Cmd = command;
  dt->Buffer = out_data;
  if (dt->Buffer == nullptr) {
    protocol_log("[%s]%s malloc is failure\r\n", TAG, __func__);
    return dt_malloc_err;
  }
  memcpy(dt->Buffer, dt, DT_FIXED_WITHOUT_CRC_LEN);
  memcpy(dt->Buffer + DT_FIXED_WITHOUT_CRC_LEN, subData, subLen);
  memcpy(dt->Buffer + DT_FIXED_WITHOUT_CRC_LEN + subLen, data, len);
  /* crc16 is 2 byte */
  dt->Crc16 = CRC16(dt->Buffer, dt->Length - 2);
#if PROTOCOL_CRC_ENDIANNESS == PROTOCOL_BIG_ENDIAN
  dt->Buffer[dt->Length - 2] = dt->Crc16 & 0xff;
  dt->Buffer[dt->Length - 1] = dt->Crc16 >> 8;
#else
  dt->Buffer[dt->Length - 1] = dt->Crc16 & 0xff;
  dt->Buffer[dt->Length - 2] = dt->Crc16 >> 8;
#endif
  return dt_ok;
}

dt_err_t Parse_from_data(data_transmission_t *dt, void *data, uint32_t len, uint8_t *out_data) {
  auto *msg = (uint8_t *) data;
  if (len < DT_FIXED_LEN) {
    protocol_log("[%s]%s malloc is failure\r\n", TAG, __func__);
    return dt_err_len_too_little;
  }
  if (data == nullptr) {
    protocol_log("[%s]%s malloc is failure\r\n", TAG, __func__);
    return dt_err_data_null;
  }
  uint16_t crc_src = *(msg + len - 2) + ((uint16_t) * (msg + len - 1) << 8);
  uint16_t crc_dest = CRC16(msg, len - 2);
#if PROTOCOL_CRC_ENDIANNESS == PROTOCOL_BIG_ENDIAN
  dt->Crc16 = (dt->Crc16 & 0xff) << 8 | (dt->Crc16 >> 8);
#endif
  if (crc_src != crc_dest) {
    protocol_log("[%s]%s crc is error %04X,%04X\r\n", TAG, __func__, crc_src, crc_dest);
    return dt_err_crc;
  }
  byte_copy(dt->FrameHeader, msg, sizeof(dt->FrameHeader), false);
  msg += sizeof(dt->FrameHeader);
#if PROTOCODEC_USE_VERSION == 1
  byte_copy(dt->Protocol, msg, sizeof(dt->Protocol), false);
  msg += sizeof(dt->Protocol);
#endif
  dt->Length = byteToInt(msg, sizeof(dt->Length), false);
  msg += sizeof(dt->Length);
#ifdef PROTOCODEC_USE_MSGID
  dt->MsgID = byteToInt(msg, sizeof(dt->MsgID), false);
  msg += sizeof(dt->MsgID);
#endif
#if PROTOCODEC_USE_DEVICE_TYPE == 1
  dt->SrcDeviceType = byteToInt(msg, sizeof(dt->SrcDeviceType), false);
  msg += sizeof(dt->SrcDeviceType);
  dt->DestDeviceType = byteToInt(msg, sizeof(dt->DestDeviceType), false);
  msg += sizeof(dt->DestDeviceType);
#endif
  dt->Cmd = byteToInt(msg, sizeof(dt->Cmd), false);
  dt->Buffer = out_data;
  if (dt->Buffer == nullptr) {
    protocol_log("[%s]%s malloc is failure\r\n", TAG, __func__);
    return dt_malloc_err;
  }
  memcpy(dt->Buffer, dt, DT_FIXED_WITHOUT_CRC_LEN);
  memcpy(dt->Buffer, data, dt->Length);
  /* crc16 is 2 byte */
  dt->Crc16 = CRC16(dt->Buffer, dt->Length - 2);
#if PROTOCOL_CRC_ENDIANNESS == PROTOCOL_BIG_ENDIAN
  dt->Buffer[dt->Length - 2] = dt->Crc16 & 0xff;
  dt->Buffer[dt->Length - 1] = dt->Crc16 >> 8;
#else
  dt->Buffer[dt->Length - 1] = dt->Crc16 & 0xff;
  dt->Buffer[dt->Length - 2] = dt->Crc16 >> 8;
#endif
  return dt_ok;
}

void protocol_log_format(data_transmission_t *dt) {
#if defined(PROTOCODEC_USE_MSGID) && defined(PROTOCODEC_USE_VERSION)
  protocol_log(
      "[%s] frame start is {%.*s}. protocol version is {%.*s}. packet size is {%d}. msg id is {%u}. "
#if PROTOCODEC_USE_DEVICE_TYPE == 1
      "source device type is {%s}. destination device type is {%s}. "
#endif
      "command "
      "is {%X:%s}. crc is {%04X}\r\n",
      TAG, sizeof(dt->FrameHeader), dt->FrameHeader, sizeof(dt->Protocol), dt->Protocol, dt->Length, dt->MsgID,
#if PROTOCODEC_USE_DEVICE_TYPE == 1
      dt_device_type_map[dt->SrcDeviceType].c_str(), dt_device_type_map[dt->DestDeviceType].c_str(),
#endif
      dt->Cmd, dt_cmd_map[dt->Cmd].c_str(), dt->Crc16);
  protocol_log("[%s] packet data(option) is ", TAG);
  for (uint32_t i = DT_FIXED_WITHOUT_CRC_LEN; i < dt->Length - 2; i++) {
    protocol_log("%02x ", dt->Buffer[i]);
  }
  protocol_log("\r\n");
#endif

#if !defined(PROTOCODEC_USE_MSGID) || !defined(PROTOCODEC_USE_VERSION)
  protocol_log(
      "[%s] frame start is {%.*s}. packet size is {%d}. source device type is {%s}. destination device type is "
      "{%s}. command "
      "is {%X:%s}. crc is {%04X}\r\n",
      TAG, (int) sizeof(dt->FrameHeader), dt->FrameHeader, dt->Length, dt_device_type_map[dt->SrcDeviceType].c_str(),
      dt_device_type_map[dt->DestDeviceType].c_str(), dt->Cmd, dt_cmd_map[dt->Cmd].c_str(), dt->Crc16);
  protocol_log("[%s] packet data(option) is ", TAG);
  for (uint32_t i = DT_FIXED_WITHOUT_CRC_LEN; i < dt->Length - 2; i++) {
    protocol_log("%02x ", dt->Buffer[i]);
  }
  protocol_log("\r\n");
#endif
}
