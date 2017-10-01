/* Copyright (c) 2013, Enrico Gueli <enrico.gueli@gmail.com> All
rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
contributors may be used to endorse or promote products derived from
this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _ARDUINO_LCD_SCREEN_H
#define _ARDUINO_LCD_SCREEN_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include "utility/Arduino_ILI9163.h"

/*
 * This library can work with or without the presence of an SD
 * reading library (to load images). At the moment, only the
 * Arduino SD library is supported; it is included in
 * standard Arduino libraries.
 * 
 * The presence of the SD library is detected by looking at the
 * __SD_H__ preprocessor variable, defined into 
 * Arduino SD library to avoid double inclusion. This means
 * that in order to use the image-related API of Adafruit_GFX,
 * SD.h *must* be included before Adafruit_GFX.
 * 
 * The bottom part of this include file contains the actual image
 * loading code; if it was in a separate .cpp file, there were no
 * way to check if the SD library was present or not.
 * 
 * A partial solution was to include SD.h anyway, see if that works
 * (i.e. it is found in the include search path) and act accordingly.
 * But this solution relied on the preprocessor to issue only a
 * warning when an include file is not found. Avr-gcc, used for
 * Arduino 8-bit MCUs, does that, but the standard gcc-4.4, used for
 * Arduino Due, issues a fatal error and stops compilation.
 * 
 * The best solution so far is to put the code here. It works if this
 * include is used only in one .cpp file in the build (this is the
 * case of most Arduino sketches); if used in multiple .cpp files,
 * the linker may complain about duplicate definitions.
 * 
 */

#if defined(__SD_H__)  // Arduino SD library
 #include "utility/PImage.h"
#else
 #warning "The SD library was not found. loadImage() and image() won't be supported."
#endif

typedef uint16_t color;

/// The Arduino LCD is a ILI9163-based device.
/// By default, it is mounted horizontally.
/// LcdScreen class follows the convention of other
/// Arduino library classes by adding a begin() method
/// to be called in the setup() routine.
/// @author Enrico Gueli <enrico.gueli@gmail.com>
class LcdScreen : public Arduino_ILI9163 {
public:
  LcdScreen(uint8_t CS, uint8_t RS, uint8_t RST);

  void begin();  

#if defined(__SD_H__)  // Arduino SD library
  PImage loadImage(const char * fileName) { return PImage::loadImage(fileName); }

  void image(PImage & img, uint16_t x, uint16_t y);
#endif

  // Arduino TFT library compatibility.
  void
      // http://processing.org/reference/background_.html
      background(uint8_t red, uint8_t green, uint8_t blue),
      background(color c),

	  // http://processing.org/reference/fill_.html
	  fill(uint8_t red, uint8_t green, uint8_t blue),
	  fill(color c),

	  // http://processing.org/reference/noFill_.html
	  noFill(),

	  // http://processing.org/reference/stroke_.html
	  stroke(uint8_t red, uint8_t green, uint8_t blue),
	  stroke(color c),

	  // http://processing.org/reference/noStroke_.html
	  noStroke(),

	  // https://processing.org/reference/text_.html
	  text(const char * text, int16_t x, int16_t y),

	  // https://processing.org/reference/textSize_.html
	  textSize(uint8_t size),

	  // similar to ellipse() in Processing, but with
	  // a single radius.
	  // http://processing.org/reference/ellipse_.html
	  circle(int16_t x, int16_t y, int16_t r),
	  point(int16_t x, int16_t y),
	  line(int16_t x1, int16_t y1, int16_t x2, int16_t y2),
	  quad(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x3, int16_t y3, int16_t x4, int16_t y4),
	  rect(int16_t x, int16_t y, int16_t width, int16_t height),
	  rect(int16_t x, int16_t y, int16_t width, int16_t height, int16_t radius),
	  triangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x3, int16_t y3);
	  ;
protected:
  /*
   * Processing-style graphics state
   */
  color strokeColor = 0x0000;
  bool useStroke = false;
  color fillColor = 0x0000;
  bool useFill = false;
};

/// Esplora boards have hard-wired connections with
/// the Arduino LCD if mounted on the onboard connector.
#if ARDUINO_AVR_ESPLORA // are we building for Esplora?
extern LcdScreen EsploraTFT;
#endif

#if defined(__SD_H__)  // Arduino SD library

#define BUFFPIXEL 20

void LcdScreen::image(PImage & img, uint16_t x, uint16_t y) {
  int      w, h, wb, hb, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0;
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer

  // Crop area to be loaded
  w = img._bmpWidth;
  h = img._bmpHeight;
  wb = x + w - 1;
  hb = y + h - 1;
  if(wb >= width())  w = width()  - x;
  if(hb >= height()) h = height() - y;

  //// Set TFT address window to clipped image bounds
  //setAddrWindow(x, y, x+w-1, y+h-1);
  
  for (row=0; row<h; row++) { // For each scanline...
    // Seek to start of scan line.  It might seem labor-
    // intensive to be doing this on every line, but this
    // method covers a lot of gritty details like cropping
    // and scanline padding.  Also, the seek only takes
    // place if the file position actually needs to change
    // (avoids a lot of cluster math in SD library).
    if(img._flip) // Bitmap is stored bottom-to-top order (normal BMP)
      pos = img._bmpImageoffset + (img._bmpHeight - 1 - row) * img._rowSize;
    else     // Bitmap is stored top-to-bottom
      pos = img._bmpImageoffset + row * img._rowSize;
    if(img._bmpFile.position() != pos) { // Need seek?
      img._bmpFile.seek(pos);
      buffidx = sizeof(sdbuffer); // Force buffer reload
    }

    for (col=0; col<w; col++) { // For each pixel...
      // Time to read more pixel data?
      if (buffidx >= sizeof(sdbuffer)) { // Indeed
        img._bmpFile.read(sdbuffer, sizeof(sdbuffer));
        buffidx = 0; // Set index to beginning
      }

      // Convert pixel from BMP to TFT format, push to display
      b = sdbuffer[buffidx++];
      g = sdbuffer[buffidx++];
      r = sdbuffer[buffidx++];
      //pushColor(tft.Color565(r,g,b));
      drawPixel(x + col, y + row, Color565(r, g, b));
      
    } // end pixel
  } // end scanline

}




// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t PImage::read16(File f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t PImage::read32(File f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}


PImage PImage::loadImage(const char * fileName) {
  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  bool     flip    = true;        // BMP is stored bottom-to-top
  uint32_t bmpFilesize;
  uint32_t bmpHeadersize;


  // Open requested file on SD card
  if ((bmpFile = SD.open(fileName)) == false) {
#if defined(LCD_SCREEN_DEBUG)
    Serial.print(F("loadImage: file not found: "));
    Serial.println(fileName);
#endif
    return PImage(); // load error
  }
  
  
  
  // Parse BMP header
  if(read16(bmpFile) != 0x4D42) { // BMP signature
#if defined(LCD_SCREEN_DEBUG)
    Serial.println(F("loadImage: file doesn't look like a BMP"));
#endif
    return PImage();
  }
  
  bmpFilesize = read32(bmpFile);
  (void)read32(bmpFile); // Read & ignore creator bytes
  bmpImageoffset = read32(bmpFile); // Start of image data
  // Read DIB header
  bmpHeadersize = read32(bmpFile); 
  bmpWidth = read32(bmpFile);
  bmpHeight = read32(bmpFile);
#if defined(LCD_SCREEN_DEBUG)
  Serial.print(F("File size: ")); Serial.println(bmpFilesize);
  Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
  Serial.print(F("Header size: ")); Serial.println(bmpHeadersize);
#endif
  if(read16(bmpFile) != 1) { // # planes -- must be '1'
#if defined(LCD_SCREEN_DEBUG)
    Serial.println(F("loadImage: invalid n. of planes"));
#endif
    return PImage();
  }
  
  bmpDepth = read16(bmpFile); // bits per pixel
#if defined(LCD_SCREEN_DEBUG)
  Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
#endif
  if((bmpDepth != 24) || (read32(bmpFile) != 0)) { // 0 = uncompressed {
#if defined(LCD_SCREEN_DEBUG)
    Serial.println(F("loadImage: invalid pixel format"));
#endif
    return PImage();
  }

#if defined(LCD_SCREEN_DEBUG)
  Serial.print(F("Image size: "));
  Serial.print(bmpWidth);
  Serial.print('x');
  Serial.println(bmpHeight);
#endif

  // BMP rows are padded (if needed) to 4-byte boundary
  rowSize = (bmpWidth * 3 + 3) & ~3;

  // If bmpHeight is negative, image is in top-down order.
  // This is not canon but has been observed in the wild.
  if(bmpHeight < 0) {
    bmpHeight = -bmpHeight;
    flip      = false;
  }
  
  return PImage(bmpFile, bmpWidth, bmpHeight, bmpDepth, bmpImageoffset, rowSize, flip);
}

#endif // __SD_H__

#endif // _ARDUINO_LCD_SCREEN_H
