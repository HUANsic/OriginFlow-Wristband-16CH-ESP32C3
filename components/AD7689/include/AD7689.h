#include <driver/spi_master.h>

typedef enum
{
    AD_INCC_BIP_DIFF = 0b000,
    AD_INCC_BIP_SING = 0b010,
    AD_INCC_TEMP = 0b011,
    AD_INCC_UNI_DIFF = 0b100,
    AD_INCC_UNI_SING = 0b110,
    AD_INCC_UNI_SING_INTGND = 0b111
} AD_INCC_e;

typedef enum
{
    AD_REF_INT_2V5 = 0b000,
    AD_REF_INT_4V096 = 0b001,
    AD_REF_EXT_NOBUF_TEMP = 0b010,
    AD_REF_EXT_BUF_TEMP = 0b011,
    AD_REF_EXT_NOBUF_NOTEMP = 0b110,
    AD_REF_EXT_BUF_NOTEMP = 0b111
} AD_REF_e;

typedef enum
{
    AD_SEQ_DISABLE = 0b00,
    AD_SEQ_UPDATE_WHILE_SCAN = 0b01,
    AD_SEQ_SCAN_THEN_TEMP = 0b10,
    AD_SEQ_SCAN_NOTEMP = 0b11
} AD_SEQ_e;

typedef struct
{
    uint8_t dummy : 2;
    uint8_t doNotReadBack : 1;
    uint8_t seq : 2;
    uint8_t ref : 3;
    uint8_t doFullBW : 1;
    uint8_t channel : 3;
    uint8_t chCFG : 3;
    uint8_t doUpdate : 1;
} _AD_CFG_t;

typedef union
{
    _AD_CFG_t bits;
    uint16_t u16;
} AD_CFG_u;

inline AD_CFG_u AD_BuildConfig(uint8_t doUpdate, AD_INCC_e chCFG, uint8_t channel, uint8_t doFullBW, AD_REF_e ref, AD_SEQ_e seq, uint8_t doReadBack)
{
    AD_CFG_u config;
    config.u16 = 0;
    config.bits.doUpdate = doUpdate;
    config.bits.chCFG = chCFG;
    config.bits.channel = channel;
    config.bits.doFullBW = doFullBW;
    config.bits.ref = ref;
    config.bits.seq = seq;
    config.bits.doNotReadBack = !doReadBack;
    config.bits.dummy = 0;
    return config;
}

uint8_t AD_Transaction(spi_device_handle_t hdev, AD_CFG_u *cfg, uint16_t *data);
