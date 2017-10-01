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

#include "LcdScreen.h"

#if ARDUINO_AVR_ESPLORA
LcdScreen EsploraTFT(7, 0, 1);
#endif

LcdScreen::LcdScreen(uint8_t CS, uint8_t RS, uint8_t RST)
  : Arduino_ILI9163(CS, RS, RST)
{ }

void LcdScreen::begin() {
  initG();
  setRotation(1);
}

// Arduino TFT library compatibility.

void LcdScreen::background(uint8_t red, uint8_t green, uint8_t blue) {
  background(Color565(red, green, blue));
}

void LcdScreen::background(color c) {
  fillScreen(c);
}

void LcdScreen::stroke(uint8_t red, uint8_t green, uint8_t blue) {
  stroke(Color565(red, green, blue));
}

void LcdScreen::stroke(color c) {
  useStroke = true;
  strokeColor = c;
  setTextColor(c);
}

void LcdScreen::noStroke() {
  useStroke = false;
}

void LcdScreen::noFill() {
  useFill = false;
}

void LcdScreen::fill(uint8_t red, uint8_t green, uint8_t blue) {
  fill(Color565(red, green, blue));
}

void LcdScreen::fill(color c) {
  useFill = true;
  fillColor = c;
}

void LcdScreen::point(int16_t x, int16_t y) {
	if (!useStroke)
		return;

	drawPixel(x, y, strokeColor);
}

void LcdScreen::text(const char * text, int16_t x, int16_t y) {
	if (!useStroke)
		return;

	setTextWrap(false);
	setTextColor(strokeColor);
	setCursor(x, y);
	print(text);
}

void LcdScreen::textSize(uint8_t size) {
	setTextSize(size);
}

void LcdScreen::line(int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
	if (!useStroke)
		return;

	if (x1 == x2) {
		if (y1 < y2)
			drawFastVLine(x1, y1, y2 - y1, strokeColor);
		else
			drawFastVLine(x1, y2, y1 - y2, strokeColor);
	}
	else if (y1 == y2) {
		if (x1 < x2)
			drawFastHLine(x1, y1, x2 - x1, strokeColor);
		else
			drawFastHLine(x2, y1, x1 - x2, strokeColor);
	}
	else {
		drawLine(x1, y1, x2, y2, strokeColor);
	}
}

void LcdScreen::rect(int16_t x, int16_t y, int16_t width, int16_t height) {
	if (useFill) {
		fillRect(x, y, width, height, fillColor);
	}
	if (useStroke) {
		drawRect(x, y, width, height, strokeColor);
	}
}

void LcdScreen::rect(int16_t x, int16_t y, int16_t width, int16_t height, int16_t radius) {
	if (radius == 0) {
		rect(x, y, width, height);
	}
	if (useFill) {
		fillRoundRect(x, y, width, height, radius, fillColor);
	}
	if (useStroke) {
		drawRoundRect(x, y, width, height, radius, strokeColor);
	}
}

void LcdScreen::circle(int16_t x, int16_t y, int16_t r) {
	if (r == 0)
		return;

	if (useFill) {
		fillCircle(x, y, r, fillColor);
	}
	if (useStroke) {
		drawCircle(x, y, r, strokeColor);
	}
}

void LcdScreen::triangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t x3, int16_t y3) {
	if (useFill) {
		fillTriangle(x1, y1, x2, y2, x3, y3, fillColor);
	}
	if (useStroke) {
		drawTriangle(x1, y1, x2, y2, x3, y3, strokeColor);
	}
}
