#ifndef _H_DEV_COMMON_H_
#define _H_DEV_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#define CODEC_MEM_CHECK(ptr)                                         \
  if (ptr == NULL) {                                                 \
    printf("Fail to alloc memory at %s:%d", __FUNCTION__, __LINE__); \
  }

#define BITS(n) (1 << n)

#ifdef __cplusplus
}
#endif

#endif
