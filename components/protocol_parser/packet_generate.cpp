#include "packet_generate.hpp"

#include <math.h>
#include <string.h>

#include <cmath>
#include <cstdint>

// #include "nlohmann/json.hpp"
#include "protocol.h"

#define DEBUG_LOG 0

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
static void float2Byte(uint8_t *u8Arry, float *floatdata, bool BigEndian) {
  uint8_t farray[4];
  *(float *) farray = *floatdata;
  if (BigEndian) {
    u8Arry[3] = farray[0];
    u8Arry[2] = farray[1];
    u8Arry[1] = farray[2];
    u8Arry[0] = farray[3];
  } else {
    u8Arry[0] = farray[0];
    u8Arry[1] = farray[1];
    u8Arry[2] = farray[2];
    u8Arry[3] = farray[3];
  }
}

static void IEE754_binary32_encode(float x, uint8_t out[4], bool isBigEndian) {
  bool sign = x < 0;
  uint8_t exponent;
  uint32_t fraction;
  if (isinf(x)) {
    exponent = 0xFF;
    fraction = 0;
  } else if (isnan(x)) {  // nan check
    exponent = 0xFF;
    fraction = 0x7FFFFF;
  } else {
    if (sign)
      x = -x;
    int e = 0;
    fraction = frexp(x, &e) * ((uint32_t) 2 << 23);
    exponent = e + 126;
    if (e + 126 > 0xFF) {
      exponent = 0xFF;
      fraction = 0;
    }
  }
  if (isBigEndian) {
    out[0] = ((sign << 7) & 0x80) | ((exponent >> 1) & 0x7F);
    out[1] = ((exponent << 7) & 0x80) | ((fraction >> 16) & 0x7F);
    out[2] = (fraction >> 8) & 0xFF;
    out[3] = fraction & 0xFF;
  } else {
    out[3] = ((sign << 7) & 0x80) | ((exponent >> 1) & 0x7F);
    out[2] = ((exponent << 7) & 0x80) | ((fraction >> 16) & 0x7F);
    out[1] = (fraction >> 8) & 0xFF;
    out[0] = fraction & 0xFF;
  }
}

int Calculate_decimals(double num, int postion) {
  return (int) std::round((num - (int) num) * std::pow(10, postion + 1)) / 10;
}
#if PROTOCODEC_USE_DEVICE_TYPE == 1
PacketGenerate::PacketGenerate(dt_device_type_t src, dt_device_type_t dest) {
  codec.dt.SrcDeviceType = src;
  codec.dt.DestDeviceType = dest;
  send_buffer = nullptr;
}
#else
PacketGenerate::PacketGenerate() {
  send_buffer = nullptr;
}

#endif

PacketGenerate::~PacketGenerate() {
  if (send_buffer != nullptr) {
    free(send_buffer);
  }
}

uint8_t *PacketGenerate::data() {
  return codec.AllData();
}
uint32_t PacketGenerate::length() {
  return codec.Len();
}

dt_err_t PacketGenerate::packet_generate_1_byte(data_transmission_command_t cmd, uint8_t status) {
#if DEBUG_LOG >= 3
  printf("[%s] cmd is %04X value is %d\r\n", __func__, cmd, status);
#endif
  memset(codec.send_litter_buffer, 0, 100);
  return codec.Generate_packet(cmd, &status, 1, codec.send_litter_buffer);
}

/* shutdown 0xFE*/
dt_err_t PacketGenerate::packet_generate_shutdown() {
  return codec.Generate_without_data(dt_cmd_shutdown, codec.send_fixed_buffer);
}

/* reboot 0xFF*/
dt_err_t PacketGenerate::packet_generate_reboot() {
  return codec.Generate_without_data(dt_cmd_reboot, codec.send_fixed_buffer);
}

/* 上报固件版本 */
dt_err_t PacketGenerate::packet_generate_version(char *version) {
  return codec.Generate_packet(dt_cmd_software_version, (uint8_t *) version, strlen(version), codec.send_litter_buffer);
}
#if PROTOCODEC_USE_DEVICE_TYPE == 1
/* 下发文件头（stream） 0x170*/
dt_err_t PacketGenerate::packet_generate_down_stream_file(dt_device_type_t device_type, dt_file_type_t file_type, int file_size, bool need_checksum,
                                                          uint8_t checksum_type, uint8_t sha256[32]) {
  // uint8_t *data = (uint8_t *) malloc(50);
  uint8_t data[50] = {0};
  int len = 0;
  data[0] = device_type;
  data[1] = file_type;
  intToByte(file_size, data + 2, 4, false);
  data[6] = need_checksum;
  data[7] = checksum_type;
  len = 8;
  if (need_checksum) {
    memcpy(data + len, sha256, 32);
    len += 32;
  }
  dt_err_t err = codec.Generate_packet(dt_cmd_update_file, data, len, codec.send_litter_buffer);
  return err;
}

/* 发送文件数据（stream） 0x172*/
dt_err_t PacketGenerate::packet_generate_down_file_packet(dt_device_type_t device_type, int packet_count, int packet_seq, uint8_t *data, int data_len) {
  if (data == nullptr) {
    return dt_err_data_null;
  }
  uint32_t packet_len = 5 + data_len;
  uint8_t *packet_data = (uint8_t *) malloc(packet_len);
  if (packet_data == nullptr) {
    return dt_malloc_err;
  }
  packet_data[0] = device_type;
  intToByte(packet_count, packet_data + 1, 2, false);
  intToByte(packet_seq, packet_data + 3, 2, false);
  memcpy(packet_data + 5, data, data_len);
  if (send_buffer != nullptr) {
    free(send_buffer);
  }
  send_buffer = (uint8_t *) malloc(DT_FIXED_LEN + packet_len);
  dt_err_t err = codec.Generate_packet(dt_cmd_send_file, packet_data, packet_len, send_buffer);
  if (packet_data != nullptr) {
    free(packet_data);
  }
  return err;
}
dt_err_t PacketGenerate::packet_generate_response(dt_device_type_t device_type, dt_response_t response, dt_error_code_t error_code, int packet_seq) {
  uint8_t data[5] = {0};
  data[0] = device_type;
  data[1] = response;
  data[2] = error_code;
  intToByte(packet_seq, data + 3, 2, false);
  return codec.Generate_packet(dt_cmd_response, data, 5, codec.send_litter_buffer);
}
/* 0xFFFE 上报信息 */
dt_err_t PacketGenerate::packet_generate_info(dt_info_code_t info_code) {
  uint16_t code = info_code;
  return codec.Generate_packet(dt_cmd_info, (uint8_t *) &code, 2, codec.send_litter_buffer);
}
#else
/* 下发文件头（stream） 0x170*/
dt_err_t PacketGenerate::packet_generate_down_stream_file(dt_file_type_t file_type, int file_size, bool need_checksum, uint8_t checksum_type,
                                                          uint8_t sha256[32]) {
  // uint8_t *data = (uint8_t *) malloc(50);
  uint8_t data[50] = {0};
  int len = 0;
  data[len++] = file_type;
  intToByte(file_size, data + len, 4, false);
  len += 4;
  data[len++] = need_checksum;
  data[len++] = checksum_type;
  if (need_checksum) {
    memcpy(data + len, sha256, 32);
    len += 32;
  }
  dt_err_t err = codec.Generate_packet(dt_cmd_update_file, data, len, codec.send_litter_buffer);
  return err;
}

/* 发送文件数据（stream） 0x172*/
dt_err_t PacketGenerate::packet_generate_down_file_packet(int packet_count, int packet_seq, uint8_t *data, int data_len) {
  if (data == nullptr) {
    return dt_err_data_null;
  }
  uint32_t packet_len = 4 + data_len;
  uint8_t *packet_data = (uint8_t *) malloc(packet_len);
  if (packet_data == nullptr) {
    return dt_malloc_err;
  }
  intToByte(packet_count, packet_data, 2, false);
  intToByte(packet_seq, packet_data + 2, 2, false);
  memcpy(packet_data + 4, data, data_len);
  if (send_buffer != nullptr) {
    free(send_buffer);
  }
  send_buffer = (uint8_t *) malloc(DT_FIXED_LEN + packet_len);
  dt_err_t err = codec.Generate_packet(dt_cmd_send_file, packet_data, packet_len, send_buffer);
  if (packet_data != nullptr) {
    free(packet_data);
  }
  return err;
}
dt_err_t PacketGenerate::packet_generate_response(dt_response_t response, dt_error_code_t error_code, int packet_seq) {
  uint8_t data[5] = {0};
  data[0] = response;
  data[1] = error_code;
  intToByte(packet_seq, data + 2, 2, false);
  return codec.Generate_packet(dt_cmd_response, data, 4, codec.send_litter_buffer);
}
/* 0xFFFE 上报信息 */
dt_err_t PacketGenerate::packet_generate_info(dt_info_code_t info_code) {
  uint16_t code = info_code;
  return codec.Generate_packet(dt_cmd_info, (uint8_t *) &code, 2, codec.send_litter_buffer);
}
#endif

dt_err_t PacketGenerate::packet_generate_info_log(uint8_t *data, uint32_t len) {
  uint8_t sub_cmd[2];
  sub_cmd[0] = dt_info_log & 0xff;
  sub_cmd[1] = (dt_info_log >> 8) & 0xff;
  dt_err_t err = dt_ok;
  if (len > 180) {
    uint8_t *p = (uint8_t *) malloc(len);
    if (p == nullptr) {
      return dt_malloc_err;
    }
    memcpy(p, data, len);
    err = codec.Generate_packet_with_custom_subHeader(dt_cmd_info, data, len, sub_cmd, 2, p);
    if (p != nullptr) {
      free(p);
    }
  } else {
    err = codec.Generate_packet_with_custom_subHeader(dt_cmd_info, data, len, sub_cmd, 2, codec.send_litter_buffer);
  }
  return err;
}

/* 0xFFFF 上报错误 */
dt_err_t PacketGenerate::packet_generate_error(dt_error_code_t error_code) {
  uint16_t code = error_code;
  return codec.Generate_packet(dt_cmd_error, (uint8_t *) &code, 2, codec.send_litter_buffer);
}

dt_err_t PacketGenerate::packet_generate_error_log(uint8_t *data, uint32_t len) {
  uint8_t sub_cmd[2];
  sub_cmd[0] = dt_error_log & 0xff;
  sub_cmd[1] = (dt_error_log >> 8) & 0xff;
  dt_err_t err = dt_ok;
  if (len > 180) {
    uint8_t *p = (uint8_t *) malloc(len);
    if (p == nullptr) {
      return dt_malloc_err;
    }
    memcpy(p, data, len);
    err = codec.Generate_packet_with_custom_subHeader(dt_cmd_error, data, len, sub_cmd, 2, p);
    if (p != nullptr) {
      free(p);
    }
  } else {
    err = codec.Generate_packet_with_custom_subHeader(dt_cmd_error, data, len, sub_cmd, 2, codec.send_litter_buffer);
  }
  return err;
}

dt_err_t PacketGenerate::packet_generate_stm32_hardFault(uint8_t *data, uint32_t len) {
  uint8_t sub_cmd[2];
  sub_cmd[0] = dt_error_stm32_hardware & 0xff;
  sub_cmd[1] = (dt_error_stm32_hardware >> 8) & 0xff;
  dt_err_t err = dt_ok;
  if (len > 180) {
    uint8_t *p = (uint8_t *) malloc(len);
    if (p == nullptr) {
      return dt_malloc_err;
    }
    memcpy(p, data, len);
    err = codec.Generate_packet_with_custom_subHeader(dt_cmd_error, data, len, sub_cmd, 2, p);
    if (p != nullptr) {
      free(p);
    }
  } else {
    err = codec.Generate_packet_with_custom_subHeader(dt_cmd_error, data, len, sub_cmd, 2, codec.send_litter_buffer);
  }
  return err;
}

dt_err_t PacketGenerate::packet_generate_imu(float gyro_x, float gyro_y, float gyro_z, float accel_x, float accel_y, float accel_z, float mag_x, float mag_y,
                                             float mag_z) {
  uint32_t packet_len = 36;
  uint8_t packet_data[36] = {0};
  memcpy(packet_data, &gyro_x, 4);
  memcpy(packet_data + 4, &gyro_y, 4);
  memcpy(packet_data + 8, &gyro_z, 4);
  memcpy(packet_data + 12, &accel_x, 4);
  memcpy(packet_data + 16, &accel_y, 4);
  memcpy(packet_data + 20, &accel_z, 4);
  memcpy(packet_data + 24, &mag_x, 4);
  memcpy(packet_data + 28, &mag_y, 4);
  memcpy(packet_data + 32, &mag_z, 4);
  return codec.Generate_packet(dt_cmd_imu, packet_data, packet_len, codec.send_litter_buffer);
}