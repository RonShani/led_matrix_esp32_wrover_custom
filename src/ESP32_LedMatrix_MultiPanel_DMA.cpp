#include "ESP32_LedMatrix_MultiPanel_DMA.hpp"

volatile SemaphoreHandle_t ESP32_LedMatrix_MultiPanel_DMA::timerSemaphore;
ESP32_LedMatrix_MultiPanel_DMA *ESP32_LedMatrix_MultiPanel_DMA::singleton;

ESP32_LedMatrix_MultiPanel_DMA::ESP32_LedMatrix_MultiPanel_DMA(uint16_t a_matrix_width, uint16_t a_matrix_height, uint16_t a_panels)
: Adafruit_GFX(a_matrix_width, a_matrix_height)
, m_matrix_width(a_matrix_width*a_panels)
, m_matrix_height(a_matrix_height)
, m_matrixbuff(new uint16_t[a_matrix_width * a_matrix_height])
, matrixbuff(nullptr)
, m_pins{}
{
  initMatrixBuff();
}

void ESP32_LedMatrix_MultiPanel_DMA::initMatrixBuff()
{
  memset(m_matrixbuff,0,sizeof(uint16_t)*m_matrix_height*m_matrix_width);
  matrixbuff = &m_matrixbuff[0];
}

void ESP32_LedMatrix_MultiPanel_DMA::swapBuffer()
{
  matrixbuff = drawBuffer();
}

void ESP32_LedMatrix_MultiPanel_DMA::printScr()
{
  if (timer){
    timerDetachInterrupt(timer);
    timerEnd(timer);
  }
  onTimer();
}

uint16_t* ESP32_LedMatrix_MultiPanel_DMA::drawBuffer()
{
  return &m_matrixbuff[0];
}

uint16_t &ESP32_LedMatrix_MultiPanel_DMA::matrix_width()
{
    return m_matrix_width;
}

uint16_t &ESP32_LedMatrix_MultiPanel_DMA::matrix_height()
{
    return m_matrix_height;
}

void IRAM_ATTR ESP32_LedMatrix_MultiPanel_DMA::onTimer()
{
  portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL_ISR(&timerMux);
  singleton->draw();

  portEXIT_CRITICAL_ISR(&timerMux);
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

void ESP32_LedMatrix_MultiPanel_DMA::begin()
{
  singleton = this;
  pinMode(m_pins.pinR1, OUTPUT);
  pinMode(m_pins.pinG1, OUTPUT);
  pinMode(m_pins.pinB1, OUTPUT);
  pinMode(m_pins.pinR2, OUTPUT);
  pinMode(m_pins.pinG2, OUTPUT);
  pinMode(m_pins.pinB2, OUTPUT);

  pinMode(m_pins.pinLAT, OUTPUT);
  pinMode(m_pins.pinCLK, OUTPUT);
  pinMode(m_pins.pinOE,  OUTPUT);

  pinMode(m_pins.pinA, OUTPUT);
  pinMode(m_pins.pinB, OUTPUT);
  pinMode(m_pins.pinC, OUTPUT);
  pinMode(m_pins.pinD, OUTPUT);

  digitalWrite(m_pins.pinLAT, LOW);
  digitalWrite(m_pins.pinCLK, LOW);
  digitalWrite(m_pins.pinOE, HIGH);

  timerSemaphore = xSemaphoreCreateBinary();
  timer = timerBegin(0, 80,true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 240,true);
    
  timerAlarmEnable(timer);
}

void ESP32_LedMatrix_MultiPanel_DMA::stop()
{
  if (timer) {
    timerDetachInterrupt(timer);
    timerEnd(timer);
  }
}

void ESP32_LedMatrix_MultiPanel_DMA::drawPixel(int16_t x, int16_t y, uint16_t color)
{
  if (x < 0 || x >= m_matrix_width || y < 0 || y >= 32) return;
  int16_t idx = x + y * m_matrix_width;
    m_matrixbuff[idx] = color;
}

void ESP32_LedMatrix_MultiPanel_DMA::drawPixel(Point p, uint16_t color)
{
  if (p.x < 0 || p.x >= m_matrix_width || p.y < 0 || p.y >= 32) return;
  int16_t idx = p.x + p.y * m_matrix_width;
    m_matrixbuff[idx] = color;
}

void IRAM_ATTR ESP32_LedMatrix_MultiPanel_DMA::draw()
{
  static byte cnt = 30;
  static byte y = 15;
  static uint32_t out = 0;
  y = (y + 1) % 16;

  if (y == 0){
      cnt = (cnt + 1) % 31;
  }

  byte cmp = (cnt >> 4) | ((cnt >> 2) & 0x2) | (cnt & 0x4) | ((cnt << 2) & 0x8) | ((cnt << 4) & 0x10);
  for (int x = 0; x < m_matrix_width; x++) {
    bool r1, b1, g1, r2, g2, b2;
    uint16_t c = m_matrixbuff[x + y * m_matrix_width];
   
    r1 = (c & 0x1f) > cmp;
    g1 = ((c >>  5) & 0x1f) > cmp;
    b1 = ((c >> 10) & 0x1f) > cmp;
    
    c = m_matrixbuff[x + (y + 16) * m_matrix_width];

    r2 = (c & 0x1f) > cmp;
    g2 = ((c >>  5) & 0x1f) > cmp;
    b2 = ((c >> 10) & 0x1f) > cmp;

    REG_WRITE(GPIO_OUT_REG, out |
       ((uint32_t)r1 << m_pins.pinR1) |
       ((uint32_t)g1 << m_pins.pinG1) |
       ((uint32_t)b1 << m_pins.pinB1) |
       ((uint32_t)r2 << m_pins.pinR2) |
       ((uint32_t)g2 << m_pins.pinG2) |
       ((uint32_t)b2 << m_pins.pinB2));

    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << m_pins.pinCLK));
  }

  REG_WRITE(GPIO_OUT1_W1TS_REG, (1 << (m_pins.pinOE - 32)) | (1 << (m_pins.pinLAT - 32)));

  out = ((y & 1) << m_pins.pinA) | ((y & 2) << (m_pins.pinB - 1)) |
        ((y & 4) << (m_pins.pinC - 2)) | ((y & 8) << (m_pins.pinD - 3));
  REG_WRITE(GPIO_OUT_REG, out);
  
  for (int x = 0; x < 8; x++) NOP();

  REG_WRITE(GPIO_OUT1_W1TC_REG, (1 << (m_pins.pinOE - 32)) | (1 << (m_pins.pinLAT - 32)));
}