#ifndef _GLOBAL_H
#define _GLOBAL_H

enum ColorSpace {
  CS_GRAY = 0,
  CS_RGB,
  CS_LUV,
  CS_HSV,
  CS_ORIG = 1
};

enum PadWith {
  padvalue = 0,
  replicate,
  symmetric,
  circular
};

enum {
  UINT8_CLASS = 0,
  INT8_CLASS = 1,
  UINT16_CLASS = 2,
  INT16_CLASS = 3,
  UINT32_CLASS = 4,
  INT32_CLASS = 5,
  SINGLE_CLASS = 6,
  DOUBLE_CLASS = 7
};

typedef signed char int8_t;
typedef signed short int16_t;
typedef signed int int32_t;

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;

#endif
