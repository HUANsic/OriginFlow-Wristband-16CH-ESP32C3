#ifndef H_PROTOCOL_CRC_H
#define H_PROTOCOL_CRC_H
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif
uint16_t CRC16(uint8_t *puchMsgg, uint16_t usDataLen);

// modbus crc
uint16_t CalcCRC(uint8_t *Buffer, uint8_t u8length);
#ifdef __cplusplus
}
#endif
#endif