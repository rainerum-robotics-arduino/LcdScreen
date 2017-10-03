# Arduino LCD Screen Library
This updated library enables an Arduino board to communicate with the Arduino
TFT LCD screen. It simplifies the process for drawing shapes, lines, images, and
text to the screen. See also
[the original TFT library](https://github.com/arduino-libraries/TFT).

The Arduino TFT library extends the Adafruit GFX, and ILI9163 libraries that it
is based on. The GFX library is responsible for the drawing routines, while the
ST7735 library is specific to the screen on the Arduino GTFT. The Arduino
specific additions were designed to work as similarly to the Processing API as
possible.

Onboard the screen is a SD card slot, which can be used through the SD library.
The TFT library relies on the SPI library for communication with the screen and
SD card, and needs to be included in all sketches.

* https://github.com/adafruit/Adafruit-GFX-Library
* https://github.com/adafruit/Adafruit-ST7735-Library
* http://www.arduino.cc/en/Reference/SD
* http://www.arduino.cc/en/Reference/SPI

See also:
* [Adafruit GFX Graphics Library](https://learn.adafruit.com/adafruit-gfx-graphics-library)

Processing subset:
* [PImage](https://processing.org/reference/PImage.html)
* [loadImage()](https://processing.org/reference/loadImage_.html)
* [image()](https://processing.org/reference/image_.html)
* [background()](https://processing.org/reference/background_.html)
* [fill()](http://processing.org/reference/fill_.html)
* [noFill()](http://processing.org/reference/noFill_.html)
* [stroke()](http://processing.org/reference/stroke_.html)
* [noStroke()](http://processing.org/reference/noStroke_.html)
* [text()](https://processing.org/reference/text_.html)
* [textSize()](https://processing.org/reference/textSize_.html)
* circle() similar to [ellipse()](https://processing.org/reference/ellipse_.html) in Processing, but with a single radius.
* [point()](https://processing.org/reference/point_.html)
* [line()](https://processing.org/reference/line_.html)
* [quad()](https://processing.org/reference/quad_.html)
* [rect()](https://processing.org/reference/rect_.html)
* [triangle()](https://processing.org/reference/triangle_.html)

This library requires at least
[Arduino IDE](https://www.arduino.cc/en/Main/Software) v1.6.8, where v1.8.3 or
newer is recommended.

## How to install
### Manual Installation
Click on the "Clone or download" button in the upper right corner. Exctract the
ZIP file, and *move and rename* the extracted folder either to the location
"**~/Documents/Arduino/libraries**" or to your custom *sketchbook* folder.
Create the "libraries" folder if it doesn't exist. Open Arduino IDE and the new
library should show up in the libraries menu.

## Derived Open Source Software
Portions of this software have been modified from Arduino source that is
covered by either [L]GPLv2 or [L]GPLv3.  The licensing details are generally
contained within the files themselves.  However, if there are no licensing
specifications, it should be treated as being the same as for the Arduino 1.8.3
environment (and later, as appropriate). It is also available free of charge.

For more information, including trademarks and copyrights and licenses, see

&nbsp;&nbsp;&nbsp;&nbsp;http://arduino.cc/

as well as the copyright and licenses for the relevant source files.