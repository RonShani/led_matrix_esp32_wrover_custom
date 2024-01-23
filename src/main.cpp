#include <Arduino.h>

#include <Adafruit_GFX.h>   
#include "ESP32_LedMatrix_MultiPanel_DMA.hpp"
#include "colors_to_uint16.hpp"

ESP32_LedMatrix_MultiPanel_DMA matrix{};

void setup(){
    matrix.begin();
}

int16_t Y=0;
int16_t X=0;

void loop()
{  
  int r=12,g=6,b=0;
  
  for(int16_t Y=0;Y<32;Y++){
    for(int16_t X=0;X<matrix.matrix_width();X++){
      matrix.drawPixel(X, Y, color444(15, 15, 15));
      delay(10);
      matrix.drawPixel(X, Y,color444(0, 0, 0));
    }
  }
    
  do{
    do{
      matrix.drawPixel(X, Y, color444(r++, g++, b++));
      delay(10);
      matrix.drawPixel(X, Y,color444(0, 0, 0));
      if (r>=16)r=random(0,15);
      else if (g>=16)g=random(0,15);
      else if (b>=16)b=random(0,15);
      }while(X++ < matrix.matrix_width());
      X=0;
    }while(Y++<31);
    Y=0;
}
