#ifndef __RGB_LED_MATRIX_PIN_DEF_HPP__
#define __RGB_LED_MATRIX_PIN_DEF_HPP__
#include <stdint.h>

class RgbMatrixPins {
public:
    uint8_t pinR1 = 25;
    uint8_t pinG1 = 26;
    uint8_t pinB1 = 27;
    uint8_t pinR2 = 21;
    uint8_t pinG2 = 22;
    uint8_t pinB2 = 23;

    uint8_t pinCLK = 18;
    uint8_t pinLAT = 32;
    uint8_t pinOE = 33;

    uint8_t pinA = 19;
    uint8_t pinB = 14;
    uint8_t pinC = 13;
    uint8_t pinD = 5;
};

#endif /* __RGB_LED_MATRIX_PIN_DEF_HPP__ */