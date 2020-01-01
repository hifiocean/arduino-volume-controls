/*
 * Digital Volume Control with WM8816 - an example
 *
 * This is free and unencumbered software released into the public domain.
 * 
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 * 
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 * 
 * For more information, please refer to <https://unlicense.org>
 *
 * The WM8816 is SPI MODE 0 compatible. To control it, send two bytes,
 * first with the 4-bit register address and a read/write bit, second
 * with the value for the register (0 - 255).
 *
 * WM8816 has one DATA pin for writing and reading. To support both 
 * reading and writing, DATA needs to be connected to MISO directly and 
 * to MOSI via a resistor of 10kohm or so.
 *  
 * The connections:
 *   AGND   (WM8816 pin 16) and DGND (pin 11) - to Arduino's GND
 *   AVDD   (WM8816 pin 1)  and DVDD (pin 7)  - to +5V
 *   CCLK   (WM8816 pin 10) - to digital pin 13 (SCK pin)
 *   DATA   (WM8816 pin  9) - to digital pin 12 (MISO pin) and, 
 *           via a 10k resistor, to digital pin 11 (MOSI pin)
 *   /CS    (WM8816 pin  6) - to digital pin 10 (SS pin)
 *   /MUTEB (WM8816 pin  8) - to digital pin  9
 *   Encoder - to digital pins 2 and 4, common - to pin 3
 *
 * created 21 Jul 2018
 * by Alexey Khudyakov
 */

// include the SPI library:
#include <SPI.h>
// include the Encoder library:
#include <Encoder.h>

// SPI pins are set by the SPI peripheral and cannot be changed
// define muteb and csb pins for the WM8816
const int mutebPin = 9; // mute, active low
const int csbPin = 10;  // chip select, active low
// define encoder pins
const int encoderPinA = 2;
const int encoderCommon = 3; 
const int encoderPinB = 4;

// define the default volume after reset, as well as max and min limits
long defaultVolume = 0xE0L; // = 0 dB; ok for a demo but would be annoying in practice
long minVolume = 0;         // = mute
long maxVolume = 0xFFL;     // = +15.5 dB

// WM8816 register addresses
// Bits 6..3 contain the actual register address
// Bits 7, 1, and 0 are "don't care", set to 0 here
// Bit 2 is 1 for read and 0 for write
const uint8_t wm8816RegRight  = 0x0E<<3; // R1 - right channel gain (read/write)
const uint8_t wm8816RegLeft   = 0x0D<<3; // R2 - left channel gain (read/write)
const uint8_t wm8816RegRef    = 0x0C<<3; // R3 - peak detector reference (read/write)
const uint8_t wm8816RegStatus = 0x0B<<3; // R4 - peak detector status (read/write)
const uint8_t wm8816RegBoth   = 0x09<<3; // R5 - both channels gain (write only)
const uint8_t wm8816RegMask   = 0x0F<<3; // mask for register address
const uint8_t wm8816ReadBit   = 0x01<<2; // set bit 2 in the address to 1 for read, to 0 for write

Encoder myEncoder(encoderPinA, encoderPinB);
long oldVolume = defaultVolume;

void setup() {
  // for convenience, set the encoder's common pin to LOW
  // this allows plugging a rotary encoder directly into the Arduino
  pinMode(encoderCommon, OUTPUT);
  digitalWrite(encoderCommon, LOW);

  // intialize myEncoder with the default volume
  myEncoder.write(oldVolume);

  // set the CSB as an output
  pinMode(csbPin, OUTPUT);
  digitalWrite(csbPin, HIGH);  
  
  // initialize SPI
  SPI.begin();
  // WM8816 SPI interface is rated at 1MHz maximum
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  // on reset, WM8816 is muted
  // to activate it, MUTEB needs to be set HIGH 
  // and a non-zero value needs to be written to the gain registers
  pinMode(mutebPin, OUTPUT);
  digitalWrite(mutebPin, HIGH);  
  wm8816Write(wm8816RegBoth, defaultVolume);

  // for debug and demonstration of wm8816Read()
  // Serial.begin(9600);
  // Serial.print("Right=");  Serial.print(wm8816Read(wm8816RegRight)); Serial.print(" ");
  // Serial.print("Left=");   Serial.print(wm8816Read(wm8816RegLeft));  Serial.print(" ");
  // Serial.print("Ref=");    Serial.print(wm8816Read(wm8816RegRef));   Serial.print(" ");
  // Serial.print("Status="); Serial.println(wm8816Read(wm8816RegStatus));
}

void loop() {
  long newVolume = myEncoder.read();
  if (newVolume != oldVolume) {
    oldVolume = constrain(newVolume, minVolume, maxVolume);
    myEncoder.write(oldVolume);
    wm8816Write(wm8816RegBoth, oldVolume);
    // for debug - requires Serial.begin() uncommented in setup() above
    // Serial.println(oldVolume);
  }
}

uint8_t wm8816Write(uint8_t wm8816Reg, uint8_t value) {
  // WM8816 datasheet recommends setting "don't care" bith to HIGH
  uint8_t address = (wm8816Reg & wm8816RegMask) | ~(wm8816RegMask | wm8816ReadBit);

  // take the CSB pin low to select the chip
  digitalWrite(csbPin, LOW);
  delayMicroseconds(1);
  
  //  send in the address and value via SPI
  SPI.transfer(address);
  uint8_t result = SPI.transfer(value);
  
  // take the CSB pin high to de-select the chip
  delayMicroseconds(1);
  digitalWrite(csbPin, HIGH);

  return result;
}

uint8_t wm8816Read(uint8_t wm8816Reg) {
  // WM8816 datasheet recommends setting "don't care" bith to HIGH
  uint8_t address = (wm8816Reg & wm8816RegMask) | ~wm8816RegMask | wm8816ReadBit;
  
  // take the CSB pin low to select the chip:
  digitalWrite(csbPin, LOW);
  delayMicroseconds(1);
  
  //  send in the address and read value via SPI
  SPI.transfer(address);
  // WM8816 has only one DATA pin, so any value can be written when reading
  uint8_t result = SPI.transfer(0);
  
  // take the CSB pin high to de-select the chip
  delayMicroseconds(1);
  digitalWrite(csbPin, HIGH);

  return result;
}

