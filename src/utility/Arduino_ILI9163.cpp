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

#include "Arduino_ILI9163.h"
#include <limits.h>
#include <pins_arduino.h>
#include <wiring_private.h>
#include <SPI.h>

inline uint16_t swapcolor(uint16_t x) { 
  return (x << 11) | (x & 0x07E0) | (x >> 11);
}

#if defined (SPI_HAS_TRANSACTION)
  static SPISettings mySPISettings;
#elif defined (__AVR__) || defined(CORE_TEENSY)
  static uint8_t SPCRbackup;
  static uint8_t mySPCR;
#endif

// Constructor when using software SPI.  All output pins are configurable.
Arduino_ILI9163::Arduino_ILI9163(int8_t cs, int8_t dc, int8_t sid, int8_t sclk, int8_t rst) 
  : Adafruit_GFX(ILI9163_TFTWIDTH_128, ILI9163_TFTHEIGHT_160)
{
  _cs   = cs;
  _dc   = dc;
  _sid  = sid;
  _sclk = sclk;
  _rst  = rst;
  hwSPI = false;
}

// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
Arduino_ILI9163::Arduino_ILI9163(int8_t cs, int8_t dc, int8_t rst) 
  : Adafruit_GFX(ILI9163_TFTWIDTH_128, ILI9163_TFTHEIGHT_160) {
  _cs   = cs;
  _dc   = dc;
  _rst  = rst;
  hwSPI = true;
  _sid  = _sclk = -1;
}

inline void Arduino_ILI9163::spiwrite(uint8_t c) {

  //Serial.println(c, HEX);

  if (hwSPI) {
#if defined (SPI_HAS_TRANSACTION)
      SPI.transfer(c);
#elif defined (__AVR__) || defined(CORE_TEENSY)
      SPCRbackup = SPCR;
      SPCR = mySPCR;
      SPI.transfer(c);
      SPCR = SPCRbackup;
#elif defined (__arm__)
      SPI.setClockDivider(21); //4MHz
      SPI.setDataMode(SPI_MODE0);
      SPI.transfer(c);
#endif
  } else {

    // Fast SPI bitbang swiped from LPD8806 library
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
#if defined(USE_FAST_IO)
      if(c & bit) *dataport |=  datapinmask;
      else        *dataport &= ~datapinmask;
      *clkport |=  clkpinmask;
      *clkport &= ~clkpinmask;
#else
      if(c & bit) digitalWrite(_sid, HIGH);
      else        digitalWrite(_sid, LOW);
      digitalWrite(_sclk, HIGH);
      digitalWrite(_sclk, LOW);
#endif
    }
  }
}


void Arduino_ILI9163::writecommand(uint8_t c) {
#if defined (SPI_HAS_TRANSACTION)
  if (hwSPI)    SPI.beginTransaction(mySPISettings);
#endif
  DC_LOW();
  CS_LOW();

  spiwrite(c);

  CS_HIGH();
#if defined (SPI_HAS_TRANSACTION)
  if (hwSPI)    SPI.endTransaction();
#endif
}


void Arduino_ILI9163::writedata(uint8_t c) {
#if defined (SPI_HAS_TRANSACTION)
  if (hwSPI)    SPI.beginTransaction(mySPISettings);
#endif
  DC_HIGH();
  CS_LOW();
    
  spiwrite(c);

  CS_HIGH();
#if defined (SPI_HAS_TRANSACTION)
  if (hwSPI)    SPI.endTransaction();
#endif
}

// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80
static const uint8_t PROGMEM
  Gcmd[] = {                  // Initialization commands for ILI9163C screens
      19,                     // 18 commands in list:
      ILI9163_SWRESET,   DELAY,//  1: Software reset, no args, w/delay
      50,                     //     50 ms delay
      ILI9163_SLPOUT ,   DELAY,//  2: Out of sleep mode, no args, w/delay
      100,                    //     255 = 500 ms delay
      0x26 , 1,  			  // 3: Set default gamma
      0x04,                   //     16-bit color
      0xb1, 2,                // 4: Frame Rate
      0x0b,
      0x14,
      0xc0, 2,                // 5: VRH1[4:0] & VC[2:0]
      0x08,
      0x00,
      0xc1, 1,                // 6: BT[2:0]
      0x05,
      0xc5, 2,                // 7: VMH[6:0] & VML[6:0]
      0x41,
      0x30,
      0xc7, 1,                // 8: LCD Driving control
      0xc1,
      0xEC, 1,                // 9: Set pumping color freq
      0x1b,
      0x3a , 1 + DELAY,       // 10: Set color format
      0x55,                   //     16-bit color
      100,
      0x2a, 4,                // 11: Set Column Address
      0x00,
      0x00,
      0x00,
      0x7f,
      0x2b, 4,                // 12: Set Page Address
      0x00,
      0x00,
      0x00,
      0x9f,
      0x36, 1,                // 12+1: Set Scanning Direction
      0xc8,
      0xb7, 1,                // 14: Set Source Output Direciton
      0x00,
      0xf2, 1,                // 15: Enable Gamma bit
      0x00,
      0xe0, 15 + DELAY,       // 16: magic
      0x28, 0x24, 0x22, 0x31,
      0x2b, 0x0e, 0x53, 0xa5,
      0x42, 0x16, 0x18, 0x12,
      0x1a, 0x14, 0x03,
      50,
      0xe1, 15 + DELAY,       // 17: more magic
      0x17, 0x1b, 0x1d, 0x0e,
      0x14, 0x11, 0x2c, 0xa5,
      0x3d, 0x09, 0x27, 0x2d,
      0x25, 0x2b, 0x3c,
      50,
      ILI9163_NORON  ,   DELAY,// 17: Normal display on, no args, w/delay
      10,                     //     10 ms delay
      ILI9163_DISPON ,   DELAY,// 18: Main screen turn on, no args, w/delay
      255 };                  //     255 = 500 ms delay


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void Arduino_ILI9163::commandList(const uint8_t *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    writecommand(pgm_read_byte(addr++)); //   Read, issue command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while(numArgs--) {                   //   For each argument...
      writedata(pgm_read_byte(addr++));  //     Read, issue argument
    }

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delay(ms);
    }
  }
}


// Initialization code common to both 'B' and 'R' type displays
void Arduino_ILI9163::commonInit(const uint8_t *cmdList) {
  ystart = xstart = colstart  = rowstart = 0; // May be overridden in init func

  pinMode(_dc, OUTPUT);
  pinMode(_cs, OUTPUT);

#if defined(USE_FAST_IO)
  csport    = portOutputRegister(digitalPinToPort(_cs));
  dcport    = portOutputRegister(digitalPinToPort(_dc));
  cspinmask = digitalPinToBitMask(_cs);
  dcpinmask = digitalPinToBitMask(_dc);
#endif

  if(hwSPI) { // Using hardware SPI
#if defined (SPI_HAS_TRANSACTION)
    SPI.begin();
    mySPISettings = SPISettings(8000000, MSBFIRST, SPI_MODE0);
#elif defined (__AVR__) || defined(CORE_TEENSY)
    SPCRbackup = SPCR;
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV4);
    SPI.setDataMode(SPI_MODE0);
    mySPCR = SPCR; // save our preferred state
    //Serial.print("mySPCR = 0x"); Serial.println(SPCR, HEX);
    SPCR = SPCRbackup;  // then restore
#elif defined (__SAM3X8E__)
    SPI.begin();
    SPI.setClockDivider(21); //4MHz
    SPI.setDataMode(SPI_MODE0);
#endif
  } else {
    pinMode(_sclk, OUTPUT);
    pinMode(_sid , OUTPUT);
    digitalWrite(_sclk, LOW);
    digitalWrite(_sid, LOW);

#if defined(USE_FAST_IO)
    clkport     = portOutputRegister(digitalPinToPort(_sclk));
    dataport    = portOutputRegister(digitalPinToPort(_sid));
    clkpinmask  = digitalPinToBitMask(_sclk);
    datapinmask = digitalPinToBitMask(_sid);
#endif
  }

  // toggle RST low to reset; CS low so it'll listen to us
  CS_LOW();
  if (_rst != -1) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, HIGH);
    delay(500);
    digitalWrite(_rst, LOW);
    delay(500);
    digitalWrite(_rst, HIGH);
    delay(500);
  }

  if(cmdList) commandList(cmdList);
}


// Initialization for ILI9163C screens
void Arduino_ILI9163::initG(void) {
  commonInit(Gcmd);
}


void Arduino_ILI9163::setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1,
 uint8_t y1) {

  writecommand(ILI9163_CASET); // Column addr set
  writedata(0x00);
  writedata(x0+xstart);     // XSTART 
  writedata(0x00);
  writedata(x1+xstart);     // XEND

  writecommand(ILI9163_RASET); // Row addr set
  writedata(0x00);
  writedata(y0+ystart);     // YSTART
  writedata(0x00);
  writedata(y1+ystart);     // YEND

  writecommand(ILI9163_RAMWR); // write to RAM
}


void Arduino_ILI9163::pushColor(uint16_t color) {
#if defined (SPI_HAS_TRANSACTION)
  if (hwSPI)    SPI.beginTransaction(mySPISettings);
#endif

  DC_HIGH();
  CS_LOW();
  spiwrite(color >> 8);
  spiwrite(color);
  CS_HIGH();

#if defined (SPI_HAS_TRANSACTION)
  if (hwSPI)    SPI.endTransaction();
#endif
}

void Arduino_ILI9163::drawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  setAddrWindow(x,y,x+1,y+1);

#if defined (SPI_HAS_TRANSACTION)
  if (hwSPI)     SPI.beginTransaction(mySPISettings);
#endif

  DC_HIGH();
  CS_LOW();
  spiwrite(color >> 8);
  spiwrite(color);
  CS_HIGH();

#if defined (SPI_HAS_TRANSACTION)
  if (hwSPI)     SPI.endTransaction();
#endif
}


void Arduino_ILI9163::drawFastVLine(int16_t x, int16_t y, int16_t h,
 uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((y+h-1) >= _height) h = _height-y;
  setAddrWindow(x, y, x, y+h-1);

  uint8_t hi = color >> 8, lo = color;
    
#if defined (SPI_HAS_TRANSACTION)
  if (hwSPI)      SPI.beginTransaction(mySPISettings);
#endif

  DC_HIGH();
  CS_LOW();
  while (h--) {
    spiwrite(hi);
    spiwrite(lo);
  }
  CS_HIGH();

#if defined (SPI_HAS_TRANSACTION)
  if (hwSPI)      SPI.endTransaction();
#endif
}


void Arduino_ILI9163::drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  setAddrWindow(x, y, x+w-1, y);

  uint8_t hi = color >> 8, lo = color;

#if defined (SPI_HAS_TRANSACTION)
  if (hwSPI)      SPI.beginTransaction(mySPISettings);
#endif

  DC_HIGH();
  CS_LOW();
  while (w--) {
    spiwrite(hi);
    spiwrite(lo);
  }
  CS_HIGH();

#if defined (SPI_HAS_TRANSACTION)
  if (hwSPI)      SPI.endTransaction();
#endif
}



void Arduino_ILI9163::fillScreen(uint16_t color) {
  fillRect(0, 0,  _width, _height, color);
}



// fill a rectangle
void Arduino_ILI9163::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;
    
#if defined (SPI_HAS_TRANSACTION)
  if (hwSPI)      SPI.beginTransaction(mySPISettings);
#endif

  DC_HIGH();
  CS_LOW();
  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
      spiwrite(hi);
      spiwrite(lo);
    }
  }
  CS_HIGH();

#if defined (SPI_HAS_TRANSACTION)
  if (hwSPI)      SPI.endTransaction();
#endif
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Arduino_ILI9163::Color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void Arduino_ILI9163::setRotation(uint8_t m) {

  writecommand(ILI9163_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     {
       writedata(MADCTL_MX | MADCTL_MY | MADCTL_BGR);
     }

     {
       _height = ILI9163_TFTHEIGHT_160;
       _width  = ILI9163_TFTWIDTH_128;
     }
     xstart = colstart;
     ystart = rowstart;
     break;
   case 1:
     {
       writedata(MADCTL_MY | MADCTL_MV | MADCTL_BGR);
     }

     {
       _width = ILI9163_TFTHEIGHT_160;
       _height = ILI9163_TFTWIDTH_128;
     }
     ystart = colstart;
     xstart = rowstart;
     break;
  case 2:
     {
       writedata(MADCTL_BGR);
     }

     {
       _height = ILI9163_TFTHEIGHT_160;
       _width  = ILI9163_TFTWIDTH_128;
     }
     xstart = colstart;
     ystart = rowstart;
     break;
   case 3:
     {
       writedata(MADCTL_MX | MADCTL_MV | MADCTL_BGR);
     }

     {
       _width = ILI9163_TFTHEIGHT_160;
       _height = ILI9163_TFTWIDTH_128;
     }
     ystart = colstart;
     xstart = rowstart;
     break;
  }
}


void Arduino_ILI9163::invertDisplay(boolean i) {
  writecommand(i ? ILI9163_INVON : ILI9163_INVOFF);
}


/******** low level bit twiddling **********/


inline void Arduino_ILI9163::CS_HIGH(void) {
#if defined(USE_FAST_IO)
  *csport |= cspinmask;
#else
  digitalWrite(_cs, HIGH);
#endif
}

inline void Arduino_ILI9163::CS_LOW(void) {
#if defined(USE_FAST_IO)
  *csport &= ~cspinmask;
#else
  digitalWrite(_cs, LOW);
#endif
}

inline void Arduino_ILI9163::DC_HIGH(void) {
#if defined(USE_FAST_IO)
  *dcport |= dcpinmask;
#else
  digitalWrite(_dc, HIGH);
#endif
}

inline void Arduino_ILI9163::DC_LOW(void) {
#if defined(USE_FAST_IO)
  *dcport &= ~dcpinmask;
#else
  digitalWrite(_dc, LOW);
#endif
}



////////// stuff not actively being used, but kept for posterity
/*

 uint8_t Arduino_ILI9163::spiread(void) {
 uint8_t r = 0;
 if (_sid > 0) {
 r = shiftIn(_sid, _sclk, MSBFIRST);
 } else {
 //SID_DDR &= ~_BV(SID);
 //int8_t i;
 //for (i=7; i>=0; i--) {
 //  SCLK_PORT &= ~_BV(SCLK);
 //  r <<= 1;
 //  r |= (SID_PIN >> SID) & 0x1;
 //  SCLK_PORT |= _BV(SCLK);
 //}
 //SID_DDR |= _BV(SID);
 
 }
 return r;
 }
 
 
 void Arduino_ILI9163::dummyclock(void) {
 
 if (_sid > 0) {
 digitalWrite(_sclk, LOW);
 digitalWrite(_sclk, HIGH);
 } else {
 // SCLK_PORT &= ~_BV(SCLK);
 //SCLK_PORT |= _BV(SCLK);
 }
 }
 uint8_t Arduino_ILI9163::readdata(void) {
 *portOutputRegister(rsport) |= rspin;
 
 *portOutputRegister(csport) &= ~ cspin;
 
 uint8_t r = spiread();
 
 *portOutputRegister(csport) |= cspin;
 
 return r;
 
 } 
 
 uint8_t Arduino_ILI9163::readcommand8(uint8_t c) {
 digitalWrite(_rs, LOW);
 
 *portOutputRegister(csport) &= ~ cspin;
 
 spiwrite(c);
 
 digitalWrite(_rs, HIGH);
 pinMode(_sid, INPUT); // input!
 digitalWrite(_sid, LOW); // low
 spiread();
 uint8_t r = spiread();
 
 
 *portOutputRegister(csport) |= cspin;
 
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 
 uint16_t Arduino_ILI9163::readcommand16(uint8_t c) {
 digitalWrite(_rs, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 uint16_t r = spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 uint32_t Arduino_ILI9163::readcommand32(uint8_t c) {
 digitalWrite(_rs, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 
 dummyclock();
 dummyclock();
 
 uint32_t r = spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 */
