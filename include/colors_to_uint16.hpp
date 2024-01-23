#ifndef __COLORS_TO_UINT16_HPP_
#define __COLORS_TO_UINT16_HPP_
#include <stdint.h>

uint16_t color444(uint8_t r, uint8_t g, uint8_t b);
uint16_t color555(uint8_t r, uint8_t g, uint8_t b);
uint16_t colorHSV(long hue, uint8_t sat, uint8_t val);

#endif /* __COLORS_TO_UINT16_HPP_ */