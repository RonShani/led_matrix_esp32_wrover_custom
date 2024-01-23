#include <Arduino.h>

#define MATRIX_WIDTH 64
#define pnls 1
extern uint16_t _matrixbuff[(MATRIX_WIDTH)*32];
extern uint16_t* matrixbuff;

#include <Adafruit_GFX.h>   // Core graphics library
#include "ESP32_LedMatrix_MultiPanel_DMA.hpp"

ESP32_LedMatrix_MultiPanel_DMA matrix{false,64,32,1};

void setup(){
    matrix.begin();
}

volatile int16_t Y=0;
volatile int16_t X=0;

void loop()
{  
  int r=12,g=6,b=0;
  uint32_t t0=micros();
  
  for(int16_t Y=0;Y<32;Y++){
    for(int16_t X=0;X<MATRIX_WIDTH;X++){
      matrix.drawPixel(X, Y, matrix.color444(15, 15, 15));
      delay(10);
      matrix.drawPixel(X, Y,matrix.color444(0, 0, 0));
    }
  }
    
  do{
    do{
      matrix.drawPixel(X, Y, matrix.color444(r++, g++, b++));
      delay(10);
      matrix.drawPixel(X, Y,matrix.color444(0, 0, 0));
      if (r>=16)r=random(0,15);
      else if (g>=16)g=random(0,15);
      else if (b>=16)b=random(0,15);
      }while(X++<MATRIX_WIDTH);
      X=0;
    }while(Y++<31);
    Y=0;
}
