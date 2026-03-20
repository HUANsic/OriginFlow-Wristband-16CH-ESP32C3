#ifndef H_PROTOCODEC_PACKET_GENERATE_H
#define H_PROTOCODEC_PACKET_GENERATE_H
// #include "nlohmann/json.hpp"
#include "protocol.h"

class PacketGenerate {
 public:
#if PROTOCODEC_USE_DEVICE_TYPE == 1
  PacketGenerate(dt_device_type_t src, dt_device_type_t dest);
#else
  PacketGenerate();
#endif
  ~PacketGenerate();
  uint8_t *data();
  uint32_t length();
#ifdef PROTOCODEC_USE_MSGID
  uint32_t msg_id() {
    return codec.dt.MsgID;
  };
#endif
  uint8_t *send_buffer = nullptr;
  PROTOCODEC::Codec codec;
  /* 生成option只有一个字节的包 */
  dt_err_t packet_generate_1_byte(data_transmission_command_t cmd, uint8_t status);

  dt_err_t packet_generate_ota_progress(uint8_t progress);

  /* shutdown 0xFE*/
  dt_err_t packet_generate_shutdown();

  /* reboot 0xFF*/
  dt_err_t packet_generate_reboot();

  /*0x100 多寄存器读 */
  dt_err_t packet_generate_read_multiple_registers(uint16_t start_address, uint16_t quantity);

  /* 0x101 */
  dt_err_t packet_generate_report_multiple_registers(uint16_t start_address, uint16_t quantity, uint16_t *value);

  /*0x102 单寄存器读 */
  dt_err_t packet_generate_read_signal_registers(uint16_t address);

  /*0x104 多寄存器写 */
  dt_err_t packet_generate_write_multiple_registers(uint16_t start_address, uint16_t quantity, uint16_t *value);

  /*0x105 单寄存器写 */
  dt_err_t packet_generate_write_signal_registers(uint16_t address, uint16_t value);

  /* 上报固件版本 */
  dt_err_t packet_generate_version(char *version);

#if PROTOCODEC_USE_DEVICE_TYPE == 1
  /* 下发文件头（stream） 0x170*/
  dt_err_t packet_generate_down_stream_file(dt_device_type_t device_type, dt_file_type_t file_type, int file_size, bool need_checksum, uint8_t checksum_type,
                                            uint8_t sha256[32]);

  /* 发送文件数据（stream） 0x172*/
  dt_err_t packet_generate_down_file_packet(dt_device_type_t frame, int packet_count, int packet_seq, uint8_t *data, int data_len);

  /* 响应 0x00*/
  dt_err_t packet_generate_response(dt_device_type_t device_type, dt_response_t response, dt_error_code_t error_code, int packet_seq);
#else
  /* 下发文件头（stream） 0x170*/
  dt_err_t packet_generate_down_stream_file(dt_file_type_t file_type, int file_size, bool need_checksum, uint8_t checksum_type, uint8_t sha256[32]);

  /* 发送文件数据（stream） 0x172*/
  dt_err_t packet_generate_down_file_packet(int packet_count, int packet_seq, uint8_t *data, int data_len);

  /* 响应 0x00*/
  dt_err_t packet_generate_response(dt_response_t response, dt_error_code_t error_code, int packet_seq);

#endif

  /* 上报信息 0xFFFD*/
  dt_err_t packet_generate_info(dt_info_code_t info_code);

  dt_err_t packet_generate_info_log(uint8_t *data, uint32_t len);

  /* 上报错误 0xFFFF*/
  dt_err_t packet_generate_error(dt_error_code_t error_code);

  /* 上报错误日志 */
  dt_err_t packet_generate_error_log(uint8_t *data, uint32_t len);

  /* stm32 hardfault 0xFFFF*/
  dt_err_t packet_generate_stm32_hardFault(uint8_t *data, uint32_t len);

  dt_err_t packet_generate_imu(float gyro_x, float gyro_y, float gyro_z, float accel_x, float accel_y, float accel_z, float mag_x, float mag_y, float mag_z);
  dt_err_t packet_generate_imu(int16_t raw_gyro_x, int16_t raw_gyro_y, int16_t raw_gyro_z, int16_t raw_accel_x, int16_t raw_accel_y, int16_t raw_accel_z,
                               int16_t raw_mag_x, int16_t raw_mag_y, int16_t raw_mag_z);
  dt_err_t packet_generate_semg(uint8_t *data, size_t len);
};

#endif