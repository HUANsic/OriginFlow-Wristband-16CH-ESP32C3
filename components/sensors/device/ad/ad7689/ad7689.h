/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _H_AD7689_H_
#define _H_AD7689_H_

#include <stdint.h>

#include "sensor_ctrl_if.h"
#include "sensor_gpio_if.h"
#include "sensor_if.h"

#ifdef __cplusplus
extern "C" {
#endif

/************************************宏定义************************************/
// 一共8个通道
#define IN_NUM       8  // IN0~IN7
#define AD7689_DEBUG 0
/* 配置字节所在位置 */
#define RB_POS   0
#define SEQ_POS  1
#define REF_POS  3
#define BW_POS   6
#define INX_POS  7
#define INCC_POS 10
#define CFG_POS  13

/* ----------配置寄存器设置---------- */
#define RB_DIS      (1UL << RB_POS)    // 不回读
#define SEQ_OFF     (0UL << SEQ_POS)   // 0b00 = 禁用通道序列
#define SEQ_ON      (3UL << SEQ_POS)   // 0b11 = 使能通道序列
#define REF_INT_2p5 (0UL << REF_POS)   // 0b000 = 参考电压 = 2.50V
#define REF_INT_4p  (1UL << REF_POS)   // 0b001 = 参考电压 = 4.096V
#define REF_INT_OUT (7UL << REF_POS)   // 0b111 = 外部参考电压
#define BW_FULL     (1UL << BW_POS)    // 全带宽
#define INCC_UNI    (7UL << INCC_POS)  // 共地
#define INCC_COM    (6UL << INCC_POS)  // 普通地
#define INX_0       (0UL << INX_POS)   // IN0
#define INX_1       (1UL << INX_POS)   // IN1
#define INX_2       (2UL << INX_POS)   // IN2
#define INX_3       (3UL << INX_POS)   // IN3
#define INX_4       (4UL << INX_POS)   // IN4
#define INX_5       (5UL << INX_POS)   // IN5
#define INX_6       (6UL << INX_POS)   // IN6
#define INX_7       (7UL << INX_POS)   // IN7
#define CFG_OVR     (1UL << CFG_POS)   // 每次更新配置寄存器
#define CFG_NO_OVR  (0UL << CFG_POS)   // 每次不更新配置寄存器

#define CFG         (13)
#define INCC        (10)
#define INx         (7)
#define BW          (6)
#define REF         (3)
#define SEQ         (1)
#define RB          (0)
typedef enum {
  AD_INCC_BIP_DIFF = 0b000,
  AD_INCC_BIP_SING = 0b010,
  AD_INCC_TEMP = 0b011,
  AD_INCC_UNI_DIFF = 0b100,
  AD_INCC_UNI_SING = 0b110,
  AD_INCC_UNI_SING_INTGND = 0b111
} AD_INCC_e;

typedef enum {
  AD_REF_INT_2V5 = 0b000,
  AD_REF_INT_4V096 = 0b001,
  AD_REF_EXT_NOBUF_TEMP = 0b010,
  AD_REF_EXT_BUF_TEMP = 0b011,
  AD_REF_EXT_NOBUF_NOTEMP = 0b110,
  AD_REF_EXT_BUF_NOTEMP = 0b111
} AD_REF_e;

typedef enum {
  AD_SEQ_DISABLE = 0b00,
  AD_SEQ_UPDATE_WHILE_SCAN = 0b01,
  AD_SEQ_SCAN_THEN_TEMP = 0b10,
  AD_SEQ_SCAN_NOTEMP = 0b11
} AD_SEQ_e;

typedef struct {
  union {
    struct {
      uint16_t rb : 1;
      uint16_t seq : 2;
      uint16_t ref : 3;
      uint16_t bw : 1;
      uint16_t inx : 3;
      uint16_t innc : 3;
      uint16_t cfg : 1;
      uint16_t reserved : 2;
    };
    uint16_t u16;
  };

} ad7689_reg_cfg_t;

#define AD7689_CODEC_DEFAULT_ADDR (0x48)

/**
 * @brief ES8311 codec configuration
 */
typedef struct {
  const sensor_ctrl_if_t *ctrl_if; /*!< Codec Control interface */
  const sensor_gpio_if_t *gpio_if; /*!< Codec GPIO interface */
  ad7689_reg_cfg_t reg_cfg;
} ad7689_cfg_t;

/**
 * @brief         New ES8311 codec interface
 * @param         codec_cfg: ES8311 codec configuration
 * @return        NULL: Fail to new ES8311 codec interface
 *                -Others: ES8311 codec interface
 */
const sensor_if_t *ad7689_new(ad7689_cfg_t *codec_cfg);
uint8_t ad7689_read_channel(sensor_if_t *dev, uint8_t channel, uint16_t *data);
#ifdef __cplusplus
}
#endif

#endif
