#include "ad7689.h"

#include <stdbool.h>
#include <string.h>

#include "prv_include/common.h"
#include "sensor_if.h"

typedef struct {
  sensor_if_t base;
  ad7689_cfg_t cfg;
  bool is_open;
  bool enabled;
} sensor_ad7689_t;

#if AD7689_DEBUG == 1
void rb_printf_cfg2(ad7689_reg_cfg_t cfg) {
  printf("AD7689 rb_printf_cfg2 rb:%d, seq:%d,ref:%d,bw:%d,inx:%d,innc:%d, cfg:%d\r\n", cfg.rb, cfg.seq, cfg.ref, cfg.bw, cfg.inx, cfg.innc, cfg.cfg);
}
void rb_printf_cfg(uint16_t v) {
  ad7689_reg_cfg_t cfg;
  cfg.u16 = 0;
  uint16_t t = v >> 2;
  cfg.cfg = t >> CFG;
  cfg.innc = (t >> INCC) & 0b111;
  cfg.inx = (t >> INx) & 0b111;
  cfg.bw = (t >> BW) & 0x01;
  cfg.ref = (t >> REF) & 0b111;
  cfg.seq = (t >> SEQ) & 0b11;
  cfg.rb = (t >> RB) & 0x01;

  printf("AD7689 rb:%d, seq:%d,ref:%d,bw:%d,inx:%d,innc:%d, cfg:%d\r\n", cfg.rb, cfg.seq, cfg.ref, cfg.bw, cfg.inx, cfg.innc, cfg.cfg);
}
#endif
static int ad7689_write_reg(sensor_ad7689_t *codec, int reg, int value) {
  return codec->cfg.ctrl_if->write_reg(codec->cfg.ctrl_if, reg, 1, &value, 1);
}

static int ad7689_read_reg(sensor_ad7689_t *codec, int reg, int *value) {
  *value = 0;
  return codec->cfg.ctrl_if->read_reg(codec->cfg.ctrl_if, reg, 2, value, 2);
}

static int ad7689_set_reg(const sensor_if_t *h, int reg, int value) {
  sensor_ad7689_t *codec = (sensor_ad7689_t *) h;
  if (codec == NULL) {
    return SENSOR_INVALID_ARG;
  }
  if (codec->is_open == false) {
    return SENSOR_WRONG_STATE;
  }
  return ad7689_write_reg(codec, reg, value);
}

static int ad7689_get_reg(const sensor_if_t *h, int reg, int *value) {
  sensor_ad7689_t *codec = (sensor_ad7689_t *) h;
  if (codec == NULL) {
    return SENSOR_INVALID_ARG;
  }
  if (codec->is_open == false) {
    return SENSOR_WRONG_STATE;
  }
  return ad7689_read_reg(codec, reg, value);
}
static int ad7689_write_read_reg(const sensor_if_t *h, int reg, int value, void *out_data) {
  sensor_ad7689_t *codec = (sensor_ad7689_t *) h;
  if (codec == NULL) {
    return SENSOR_INVALID_ARG;
  }
  uint8_t rx_data[4] = {0};
  uint16_t len = 4;
  uint8_t tx_buf[4] = {0};
  tx_buf[0] = value >> 8;
  tx_buf[1] = value & 0xff;
  codec->cfg.ctrl_if->write_read_reg(codec->cfg.ctrl_if, tx_buf, len, rx_data, len);
  *(int16_t *) out_data = (rx_data[0] << 8) | rx_data[1];
  uint16_t cfg2 = 0;
  cfg2 = ((uint16_t) rx_data[2] << 8) | rx_data[3];
  uint16_t t = cfg2 >> 2;
  int inx = (t >> INx) & 0b111;
#if AD7689_DEBUG == 1
  rb_printf_cfg2(codec->cfg.reg_cfg);
  printf("AD7689 read actual channel:%d cfg: %04X, cfg2:%04x\r\n", inx, value, cfg2);
  rb_printf_cfg(cfg2);
#endif
  return len;
}

static int ad7689_read_channel_data(const sensor_if_t *h, int channel, void *data) {
  sensor_ad7689_t *codec = (sensor_ad7689_t *) h;
  if (codec == NULL) {
    printf("AD7689 [Error] Wrong codec config\r\n");
    return -1;
  }

  codec->cfg.reg_cfg.inx = channel;
  // 回读，需要知道通道
  codec->cfg.reg_cfg.rb = 0;
  uint16_t ad7689_config2 = codec->cfg.reg_cfg.u16 << 2;

  uint8_t rx_data[4] = {0};
  uint16_t len = 4;
  uint8_t tx_buf[4] = {0};
  tx_buf[0] = ad7689_config2 >> 8;
  tx_buf[1] = ad7689_config2 & 0xff;
  codec->cfg.ctrl_if->write_read_reg(codec->cfg.ctrl_if, tx_buf, len, rx_data, len);
  *(int16_t *) data = (rx_data[0] << 8) | rx_data[1];
  uint16_t cfg2 = 0;
  cfg2 = ((uint16_t) rx_data[2] << 8) | rx_data[3];
  uint16_t t = cfg2 >> 2;
  int inx = (t >> INx) & 0b111;
#if AD7689_DEBUG == 1
  rb_printf_cfg2(codec->cfg.reg_cfg);
  printf("AD7689 read channel:%d,actual channel:%d cfg: %04X, cfg2:%04x\r\n", channel, inx, ad7689_config2, cfg2);
  rb_printf_cfg(cfg2);
#endif
  return inx;
}

static int ad7689_read_all_channel_data(const sensor_if_t *h, void *data) {
  int16_t d = 0;
  int channel = 0;
  for (int i = 0; i < 8; i++) {
    channel = ad7689_read_channel_data(h, i, &d);
    if (channel >= 0 && channel < 8) {
      *(int16_t *) (data + i) = d;
    }
  }
  return SENSOR_OK;
}
const sensor_if_t *ad7689_new(ad7689_cfg_t *codec_cfg) {
  if (codec_cfg == NULL || codec_cfg->ctrl_if == NULL) {
    printf("AD7689 [Error] Wrong codec config\r\n");
    return NULL;
  }
  if (codec_cfg->ctrl_if->is_open(codec_cfg->ctrl_if) == false) {
    printf("AD7689 [Error] Control interface not open yet\r\n");
    return NULL;
  }
  sensor_ad7689_t *codec = (sensor_ad7689_t *) calloc(1, sizeof(sensor_ad7689_t));
  if (codec == NULL) {
    CODEC_MEM_CHECK(codec);
    return NULL;
  }
  codec->base.set_reg = ad7689_set_reg;
  codec->base.get_reg = ad7689_get_reg;
  codec->base.read_all_channel_data = ad7689_read_all_channel_data;
  codec->base.read_channel_data = ad7689_read_channel_data;
  codec->base.write_read_reg = ad7689_write_read_reg;
  memcpy(&codec->cfg, codec_cfg, sizeof(ad7689_cfg_t));
  return &codec->base;
}
