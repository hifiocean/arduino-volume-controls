/*
 * Digital Volume Control with DS1882 - an example
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
 * DS1882 board connections:
 *   V+     - to +8..15V (regulated to +7V onboard)
 *   V-     - to -8..15V (regulated to -7V onboard)
 *   I2C C  - to Arduino's SCL (A5 on Duemilanove)
 *   I2C D  - to Arduino's SDA (A4 on Duemilanove)
 *   I2C +  - to Ardunio's 5V
 *   I2C 0  - to Arduino GND
 *   I,O,C  - to analog Input, Output, Common for each channel
 *   COM2GND jumpers on the bottom of the board MUST be shorted
 *   (with e.g. a drop of solder) if the analog Common is not 
 *   connected to Arduino's GND elswhere.
 *   
 * Encoder connections:
 *   Pin A  - to Arduino pin 2 
 *   Pin B  - to Arduino pin 4 
 *   Common - to digital pin 3 (or to GND)
 *
 * created 31 Dec 2019
 * by Alexey Khudyakov
 */

// include the I2C/Wire library:
#include <Wire.h>

// include Adafruit's library for LCD with I2C backpack
#include "Adafruit_LiquidCrystal.h" 

// include the Encoder library:
#include <Encoder.h>

// define encoder pins
#define encoderPinA   (2)
#define encoderPinB   (4)
#define encoderCommon (3) 

// DS1882 I2C address
#define ds1882Addr      (0x28) // default 7-bit address, A0..A2 are LOW

// DS1882 registers addresses and bit masks and values
#define bmReg           (0xC0) // Register address bits 
#define regPot0         (0x00) // Potentiometer 0 register
#define regPot1         (0x40) // Potentiometer 1 register
#define regConfig       (0x80) // Configuration register
#define bmValue         (0x3F) // Register value bits 
#define bvVolatile      (0x04) // Non-volatile memory disable
#define bvZeroCrossing  (0x02) // Zero crossing detection enable
#define bvCompatibility (0x01) // Enable DS1808 compatibility mode (33 positions + mute)
#define ds1882Conf      (bvZeroCrossing) // non-volatile, zero-crossing enabled, 64 positions

// define max and min volume limits
#define minPosition (0)  // 0dB - no attenuation
#define maxPosition (63) // mute

// LCD display with Adafruit's I2C backpack, default I2C address (A0-A2 not jumpered)
Adafruit_LiquidCrystal lcd(0);

// Rotary encoder
Encoder myEncoder(encoderPinB, encoderPinA); // reverse because we control attenuation, not volume
int8_t currentPosition;

void displayVolume() {
  // display volume level
  lcd.setCursor(7, 0);
  lcd.print(-currentPosition);
  lcd.print("dB "); // to clear the 2nd digit if the number is < 10
}

void ds1882write(int8_t ch0, int8_t ch1) {
  // update potentiometers
  Wire.beginTransmission(ds1882Addr);
  Wire.write((ch0 & bmValue) | regPot0); // Pot 0
  Wire.write((ch1 & bmValue) | regPot1); // Pot 1
  Wire.endTransmission();
}

int8_t ds1882init() {
  int8_t dsState[3];
  
  // begin I2C transaction
  Wire.begin();
  
  // request three status bytes from DS1882
  Wire.requestFrom(ds1882Addr, 3);

  for(int i = 0; Wire.available() && i < 3; i++) dsState[i] = Wire.read() & bmValue;
  // dsState[0] = Pot0 value, dsState[1] = Pot1 value, dsState[2] = configuration

  if (dsState[2] != ds1882Conf) {
    // the configuration is not what it should be, so we send the right one
    Wire.beginTransmission(ds1882Addr);
    Wire.write(ds1882Conf | regConfig);
    Wire.endTransmission();
  }
  return dsState[0];
}

void setup() {
  //set the encoder's common pin to LOW
  pinMode(encoderCommon, OUTPUT);
  digitalWrite(encoderCommon, LOW);

  // initialize the DS1882
  currentPosition = ds1882init();

  // intialize myEncoder with the current volume
  myEncoder.write(currentPosition);

  // initialize the LCD and display the initial state
  lcd.begin(16, 2);
  lcd.print("Volume:");
  displayVolume();
}

void loop() {
  int8_t newPosition = myEncoder.read();
  if (newPosition != currentPosition) {
    currentPosition = constrain(newPosition, minPosition, maxPosition);
    if (newPosition != currentPosition) myEncoder.write(currentPosition);
    ds1882write(currentPosition, currentPosition);
    displayVolume();
  }
}
