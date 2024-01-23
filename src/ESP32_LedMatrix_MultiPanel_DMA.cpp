#include "ESP32_LedMatrix_MultiPanel_DMA.hpp"

volatile SemaphoreHandle_t ESP32_LedMatrix_MultiPanel_DMA::timerSemaphore;
ESP32_LedMatrix_MultiPanel_DMA* ESP32_LedMatrix_MultiPanel_DMA ::singleton;

ESP32_LedMatrix_MultiPanel_DMA::ESP32_LedMatrix_MultiPanel_DMA(bool _doubleBuffer = false, uint16_t a_matrix_width = 64, uint16_t a_matrix_height = 32, uint16_t a_panels = 1)
: Adafruit_GFX(a_matrix_width, a_matrix_height)
, m_matrix_width(a_matrix_width)
, m_matrix_height(a_matrix_height)
, m_panels(a_panels)
, m_matrixbuff(new uint16_t[a_matrix_width * a_matrix_height])
, matrixbuff(nullptr)
, doubleBuffer(_doubleBuffer)
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

void ESP32_LedMatrix_MultiPanel_DMA::copyBuffer(uint8_t  * payload, int len, int globalIndex) {
  typedef union{
    uint16_t * u16p;
    uint8_t  * u8p;
  }MB_DATA;
  MB_DATA unitMB;
  unitMB.u8p=payload;
  memcpy(&m_matrixbuff[(1920*globalIndex)],unitMB.u16p,len);
}
void IRAM_ATTR ESP32_LedMatrix_MultiPanel_DMA::onTimer() {
  portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
  portENTER_CRITICAL_ISR(&timerMux);
  singleton->draw();

  portEXIT_CRITICAL_ISR(&timerMux);
  xSemaphoreGiveFromISR(timerSemaphore, NULL);
}

void ESP32_LedMatrix_MultiPanel_DMA::begin() {
  singleton = this;
  pinMode(pinR1, OUTPUT);
  pinMode(pinG1, OUTPUT);
  pinMode(pinB1, OUTPUT);
  pinMode(pinR2, OUTPUT);
  pinMode(pinG2, OUTPUT);
  pinMode(pinB2, OUTPUT);

  pinMode(pinLAT, OUTPUT);
  pinMode(pinCLK, OUTPUT);
  pinMode(pinOE,  OUTPUT);

  pinMode(pinA, OUTPUT);
  pinMode(pinB, OUTPUT);
  pinMode(pinC, OUTPUT);
  pinMode(pinD, OUTPUT);

  digitalWrite(pinLAT, LOW);
  digitalWrite(pinCLK, LOW);
  digitalWrite(pinOE, HIGH);

  timerSemaphore = xSemaphoreCreateBinary();
  timer = timerBegin(0, 80,true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 240,true);
    
  timerAlarmEnable(timer);
}

void ESP32_LedMatrix_MultiPanel_DMA::stop() {
  if (timer) {
    timerDetachInterrupt(timer);
    timerEnd(timer);
  }
}

uint16_t ESP32_LedMatrix_MultiPanel_DMA::colorHSV(long hue, uint8_t sat, uint8_t val) {
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

void ESP32_LedMatrix_MultiPanel_DMA::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if (x < 0 || x >= _MATRIX_WIDTH || y < 0 || y >= 32) return;
  int16_t idx = x + y * _MATRIX_WIDTH;
    m_matrixbuff[idx] = color;
}
void ESP32_LedMatrix_MultiPanel_DMA::drawPixel(Point p, uint16_t color){
  if (p.x < 0 || p.x >= _MATRIX_WIDTH || p.y < 0 || p.y >= 32) return;
  int16_t idx = p.x + p.y * _MATRIX_WIDTH;
    m_matrixbuff[idx] = color;
}

void IRAM_ATTR ESP32_LedMatrix_MultiPanel_DMA::draw(){
  static byte cnt = 30;
  static byte y = 15;
  static uint32_t out = 0;
  y = (y + 1) % 16;

  if (y == 0){
      cnt = (cnt + 1) % 31;
  }

  byte cmp = (cnt >> 4) | ((cnt >> 2) & 0x2) | (cnt & 0x4) | ((cnt << 2) & 0x8) | ((cnt << 4) & 0x10);
  for (int x = 0; x < _MATRIX_WIDTH; x++) {
    bool r1, b1, g1, r2, g2, b2;
    uint16_t c = m_matrixbuff[x + y * _MATRIX_WIDTH];
   
    r1 = (c & 0x1f) > cmp;
    g1 = ((c >>  5) & 0x1f) > cmp;
    b1 = ((c >> 10) & 0x1f) > cmp;
    
    c = m_matrixbuff[x + (y + 16) * _MATRIX_WIDTH];

    r2 = (c & 0x1f) > cmp;
    g2 = ((c >>  5) & 0x1f) > cmp;
    b2 = ((c >> 10) & 0x1f) > cmp;

    REG_WRITE(GPIO_OUT_REG, out |
       ((uint32_t)r1 << pinR1) |
       ((uint32_t)g1 << pinG1) |
       ((uint32_t)b1 << pinB1) |
       ((uint32_t)r2 << pinR2) |
       ((uint32_t)g2 << pinG2) |
       ((uint32_t)b2 << pinB2));

    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << pinCLK));
  }

  REG_WRITE(GPIO_OUT1_W1TS_REG, (1 << (pinOE - 32)) | (1 << (pinLAT - 32)));

  out = ((y & 1) << pinA) | ((y & 2) << (pinB - 1)) |
        ((y & 4) << (pinC - 2)) | ((y & 8) << (pinD - 3));
  REG_WRITE(GPIO_OUT_REG, out);
  
  for (int x = 0; x < 8; x++) NOP();

  REG_WRITE(GPIO_OUT1_W1TC_REG, (1 << (pinOE - 32)) | (1 << (pinLAT - 32)));
}


void ESP32_LedMatrix_MultiPanel_DMA::drawManual() {
  static byte cnt = 30;
  static uint32_t out = 0;
  
  for(byte y=0;y<16;y++){
  byte cmp = (cnt >> 4) | ((cnt >> 2) & 0x2) | (cnt & 0x4) | ((cnt << 2) & 0x8) | ((cnt << 4) & 0x10);
  for (int x = 0; x < _MATRIX_WIDTH; x++) {
    bool r1, b1, g1, r2, g2, b2;
    int16_t idb=x + y * _MATRIX_WIDTH;
    uint16_t c = m_matrixbuff[idb];
   
    r1 = (c & 0x1f) > cmp;
    g1 = ((c >>  5) & 0x1f) > cmp;
    b1 = ((c >> 10) & 0x1f) > cmp;
    idb=x + (y + 16) * _MATRIX_WIDTH;
    c = m_matrixbuff[idb];
    r2 = (c & 0x1f) > cmp;
    g2 = ((c >>  5) & 0x1f) > cmp;
    b2 = ((c >> 10) & 0x1f) > cmp;

    REG_WRITE(GPIO_OUT_REG, out |
       ((uint32_t)r1 << pinR1) |
       ((uint32_t)g1 << pinG1) |
       ((uint32_t)b1 << pinB1) |
       ((uint32_t)r2 << pinR2) |
       ((uint32_t)g2 << pinG2) |
       ((uint32_t)b2 << pinB2));

    REG_WRITE(GPIO_OUT_W1TS_REG, (1 << pinCLK));
  }

  REG_WRITE(GPIO_OUT1_W1TS_REG, (1 << (pinOE - 32)) | (1 << (pinLAT - 32)));

  out = ((y & 1) << pinA) | ((y & 2) << (pinB - 1)) |
        ((y & 4) << (pinC - 2)) | ((y & 8) << (pinD - 3));
  REG_WRITE(GPIO_OUT_REG, out);

  for (int x = 0; x < 8; x++) NOP();

  REG_WRITE(GPIO_OUT1_W1TC_REG, (1 << (pinOE - 32)) | (1 << (pinLAT - 32)));
}
}