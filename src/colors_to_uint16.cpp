#include "colors_to_uint16.hpp"

uint16_t colorHSV(long hue, uint8_t sat, uint8_t val)
{
  uint8_t  r, g, b, lo;
  uint16_t s1, v1;

  // Hue ( 0 - 1535 )
  hue %= 1536;
  if (hue < 0) hue += 1536;
  lo = hue & 255;          // Low byte  = primary/secondary color mix
  switch (hue >> 8) {      // High byte = sextant of colorwheel
    case 0 : r = 255     ; g =  lo     ; b =   0     ; break; // R to Y
    case 1 : r = 255 - lo; g = 255     ; b =   0     ; break; // Y to G
    case 2 : r =   0     ; g = 255     ; b =  lo     ; break; // G to C
    case 3 : r =   0     ; g = 255 - lo; b = 255     ; break; // C to B
    case 4 : r =  lo     ; g =   0     ; b = 255     ; break; // B to M
    default: r = 255     ; g =   0     ; b = 255 - lo; break; // M to R
  }

  s1 = sat + 1;
  r  = 255 - (((255 - r) * s1) >> 8);
  g  = 255 - (((255 - g) * s1) >> 8);
  b  = 255 - (((255 - b) * s1) >> 8);

  v1 = val + 1;
  r = (r * v1) >> 11;
  g = (g * v1) >> 11;
  b = (b * v1) >> 11;

  return color555(r, g, b);
}

uint16_t color444(uint8_t r, uint8_t g, uint8_t b)
{ 
  return ((r & 0xf) << 1) | ((uint16_t)(g & 0xf) << 6) | ((uint16_t)(b & 0xf) << 11);
}

uint16_t color555(uint8_t r, uint8_t g, uint8_t b)
{
  return (r&0x1f) | ((uint16_t)(g & 0x1f) << 5) | ((uint16_t)(b & 0x1f) << 10);
}