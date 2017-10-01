/***************************************************
  This is a library for the Arduino 1.8" SPI display.

This library works with the Arduino 1.8" TFT Breakout w/SD card
  ----> https://store.arduino.cc/arduino-lcd-screen

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#ifndef _ARDUINO_ILI9163H_
#define _ARDUINO_ILI9163H_

#include <Arduino.h>
#include <Print.h>
#include <Adafruit_GFX.h>

#if defined(__AVR__) || defined(CORE_TEENSY)
  #include <avr/pgmspace.h>
  #define USE_FAST_IO
  typedef volatile uint8_t RwReg;
#elif defined(ARDUINO_STM32_FEATHER)
  typedef volatile uint32 RwReg;
  #define USE_FAST_IO
#elif defined(ARDUINO_FEATHER52)
  typedef volatile uint32_t RwReg;
  #define USE_FAST_IO
#elif defined(ESP8266)
  #include <pgmspace.h>
#elif defined(__SAM3X8E__)
  #undef __FlashStringHelper::F(string_literal)
  #define F(string_literal) string_literal
  #include <include/pio.h>
  #define PROGMEM
  #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
  #define pgm_read_word(addr) (*(const unsigned short *)(addr))
  typedef unsigned char prog_uchar;
#endif


  // for 1.44 and mini
#define ILI9163_TFTWIDTH_128  128
  // for mini
#define ILI9163_TFTWIDTH_80   80
  // for 1.44" display
#define ILI9163_TFTHEIGHT_128 128
  // for 1.8" and mini display
#define ILI9163_TFTHEIGHT_160 160

#define ILI9163_NOP     0x00
#define ILI9163_SWRESET 0x01
#define ILI9163_RDDID   0x04
#define ILI9163_RDDST   0x09

#define ILI9163_SLPIN   0x10
#define ILI9163_SLPOUT  0x11
#define ILI9163_PTLON   0x12
#define ILI9163_NORON   0x13

#define ILI9163_INVOFF  0x20
#define ILI9163_INVON   0x21
#define ILI9163_DISPOFF 0x28
#define ILI9163_DISPON  0x29
#define ILI9163_CASET   0x2A
#define ILI9163_RASET   0x2B
#define ILI9163_RAMWR   0x2C
#define ILI9163_RAMRD   0x2E

#define ILI9163_PTLAR   0x30
#define ILI9163_COLMOD  0x3A
#define ILI9163_MADCTL  0x36

#define ILI9163_FRMCTR1 0xB1
#define ILI9163_FRMCTR2 0xB2
#define ILI9163_FRMCTR3 0xB3
#define ILI9163_INVCTR  0xB4
#define ILI9163_DISSET5 0xB6

#define ILI9163_PWCTR1  0xC0
#define ILI9163_PWCTR2  0xC1
#define ILI9163_PWCTR3  0xC2
#define ILI9163_PWCTR4  0xC3
#define ILI9163_PWCTR5  0xC4
#define ILI9163_VMCTR1  0xC5

#define ILI9163_RDID1   0xDA
#define ILI9163_RDID2   0xDB
#define ILI9163_RDID3   0xDC
#define ILI9163_RDID4   0xDD

#define ILI9163_PWCTR6  0xFC

#define ILI9163_GMCTRP1 0xE0
#define ILI9163_GMCTRN1 0xE1

// Color definitions
#define	ILI9163_BLACK   0x0000
#define	ILI9163_BLUE    0x001F
#define	ILI9163_RED     0xF800
#define	ILI9163_GREEN   0x07E0
#define ILI9163_CYAN    0x07FF
#define ILI9163_MAGENTA 0xF81F
#define ILI9163_YELLOW  0xFFE0
#define ILI9163_WHITE   0xFFFF


class Arduino_ILI9163 : public Adafruit_GFX {

 public:

  Arduino_ILI9163(int8_t CS, int8_t RS, int8_t SID, int8_t SCLK, int8_t RST = -1);
  Arduino_ILI9163(int8_t CS, int8_t RS, int8_t RST = -1);

  void     initG(void),                             // for ILI9163C displays
           setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1),
           pushColor(uint16_t color),
           fillScreen(uint16_t color),
           drawPixel(int16_t x, int16_t y, uint16_t color),
           drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color),
           drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color),
           fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
             uint16_t color),
           setRotation(uint8_t r),
           invertDisplay(boolean i);
  uint16_t Color565(uint8_t r, uint8_t g, uint8_t b);

  /* These are not for current use, 8-bit protocol only!
  uint8_t  readdata(void),
           readcommand8(uint8_t);
  uint16_t readcommand16(uint8_t);
  uint32_t readcommand32(uint8_t);
  void     dummyclock(void);
  */

 private:
  void     spiwrite(uint8_t),
           writecommand(uint8_t c),
           writedata(uint8_t d),
           commandList(const uint8_t *addr),
           commonInit(const uint8_t *cmdList);
//uint8_t  spiread(void);


  inline void CS_HIGH(void);
  inline void CS_LOW(void);
  inline void DC_HIGH(void);
  inline void DC_LOW(void);

  boolean  hwSPI;

  int8_t  _cs, _dc, _rst, _sid, _sclk;
  uint8_t colstart, rowstart, xstart, ystart; // some displays need this changed

#if defined(USE_FAST_IO)
  volatile RwReg  *dataport, *clkport, *csport, *dcport;

  #if defined(__AVR__) || defined(CORE_TEENSY)  // 8 bit!
    uint8_t  datapinmask, clkpinmask, cspinmask, dcpinmask;
  #else    // 32 bit!
    uint32_t  datapinmask, clkpinmask, cspinmask, dcpinmask;
  #endif
#endif

};


#endif // _ARDUINO_ILI9163H_
