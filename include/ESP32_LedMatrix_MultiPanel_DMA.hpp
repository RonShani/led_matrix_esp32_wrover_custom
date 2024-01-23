#ifndef _ESP32_LedMatrix_MultiPanel_DMA
#define _ESP32_LedMatrix_MultiPanel_DMA

#define ESP32_I2S_CLOCK_SPEED (40000000UL)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/soc.h"

#include "Adafruit_GFX.h"
#include "point.hpp"
#include "rgb_led_matrix_pin_configuration.hpp"

class ESP32_LedMatrix_MultiPanel_DMA : public Adafruit_GFX {
  public:
    ESP32_LedMatrix_MultiPanel_DMA(uint16_t a_matrix_width = 64, uint16_t a_matrix_height = 32, uint16_t a_panels = 1);
    void begin();
    void stop();
    virtual void drawPixel(int16_t x, int16_t y, uint16_t color);
    virtual void drawPixel(Point p, uint16_t color);

  public:
    uint16_t &matrix_width();
    uint16_t &matrix_height();
  
  private:
    static void IRAM_ATTR onTimer();
    void IRAM_ATTR draw();
    void swapBuffer();
    void printScr();
    void initMatrixBuff();
    uint16_t* drawBuffer();

  private:
    uint16_t m_matrix_width;
    uint16_t m_matrix_height;
    uint16_t *m_matrixbuff;
    uint16_t *matrixbuff;
    RgbMatrixPins m_pins;
    hw_timer_t* timer;

  private:
    static volatile SemaphoreHandle_t timerSemaphore;
    static ESP32_LedMatrix_MultiPanel_DMA *singleton;
    
};

#endif
