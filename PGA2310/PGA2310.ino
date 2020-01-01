/*
 * Digital Volume Control with PGA2310 - an example
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
 * The PGA2310 is SPI MODE 0 compatible. To control it, send two bytes,
 * first with the right cahnnel gain, second with the left channel gain. 
 * 
 * The connections:
 *   VD+    (PGA2310 pin  4) - to +5V
 *   DGND   (PGA2310 pin  5) - to Arduino's GND
 *   VA+    (PGA2310 pin 12) - to +15V
 *   VA-    (PGA2310 pin 13) - to -15V
 *   SCLK   (PGA2310 pin  6) - to digital pin 13 (SCK pin)
 *   SDO    (PGA2310 pin  7) - to digital pin 12 (MISO pin) 
 *   SDI    (PGA2310 pin  3) - to digital pin 11 (MOSI pin)
 *   /CS    (PGA2310 pin  2) - to digital pin 10 (SS pin)
 *   /MUTE  (PGA2310 pin  8) - to digital pin 9
 *   ZCEN   (PGA2310 pin  1) - to digital pin 8
 *   Encoder - to digital pins 2 and 4, common - to pin 3
 *
 * created 22 Jul 2018
 * by alexcp
 *
 */

// include the SPI library:
#include <SPI.h>
// include the Encoder library:
#include <Encoder.h>

// SPI pins are set by the SPI peripheral and cannot be changed
// define muteb, zcen and csb pins for the PGA2310
const int zcenPin = 8;  // zero crossing enable, active HIGH
const int mutebPin = 9; // mute, active LOW
const int csbPin = 10;  // chip select, active LOW

// define encoder pins
const int encoderPinA = 2;
const int encoderCommon = 3; 
const int encoderPinB = 4;

// define the default volume after reset, as well as max and min limits
long defaultVolume = 0xC0L; // = 0 dB; ok for a demo but would be annoying in practice
long minVolume = 0;         // = mute
long maxVolume = 0xFFL;     // = +31.5 dB

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
  
  // set the ZCEN as an output
  pinMode(zcenPin, OUTPUT);
  digitalWrite(zcenPin, HIGH);  
  
  // initialize SPI
  SPI.begin();
  // PGA2310 SPI interface is rated at 6.25MHz maximum
  SPI.beginTransaction(SPISettings(6250000, MSBFIRST, SPI_MODE0));

  // on reset, PGA2310 is muted
  // to activate it, MUTEB needs to be set HIGH 
  // and a non-zero values needs to be written to the gain registers
  pinMode(mutebPin, OUTPUT);
  digitalWrite(mutebPin, HIGH);  
  pga2310Write(defaultVolume, defaultVolume);

  // for debug
  // Serial.begin(9600);
}

void loop() {
  long newVolume = myEncoder.read();
  if (newVolume != oldVolume) {
    oldVolume = constrain(newVolume, minVolume, maxVolume);
    myEncoder.write(oldVolume);
    pga2310Write(oldVolume, oldVolume);
    // for debug - requires Serial.begin() uncommented in setup() above
    // Serial.println(oldVolume);
  }
}

uint8_t pga2310Write(uint8_t rightChannelvalue, uint8_t leftChannelValue) {

  // take the CSB pin low to select the chip
  digitalWrite(csbPin, LOW);
  delayMicroseconds(1);
  
  //  send in the data via SPI
  SPI.transfer(rightChannelvalue);
  uint8_t result = SPI.transfer(leftChannelValue);
  
  // take the CSB pin high to de-select the chip
  delayMicroseconds(1);
  digitalWrite(csbPin, HIGH);

  return result;
}

