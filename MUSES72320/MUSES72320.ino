/*
 * Digital Volume Control with NJR MUSES72302 - an example
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
 * The MUSES72302 is SPI MODE 0(?) compatible. To control it, 
 * send two bytes, first with the control data (gain, etc.),
 * second with the pair of register/chip addresses.
 * 
 * The connections:
 *   DREF   (MUSES72302 pin 21) - to Arduino's GND via a 10kOhm resistor
 *   DVDD   (MUSES72302 pin 17) - to +5V via a 10kOhm resistor
 *   CLOCK  (MUSES72302 pin 19) - to digital pin 13 (SCK pin)
 *   DATA   (MUSES72302 pin 18) - to digital pin 11 (MOSI pin)
 *   LATCH  (MUSES72302 pin 20) - to digital pin 10 (SS pin)
 *   /ZCEN  (MUSES72302 pin 32) - to digital pin  8
 *   Encoder - to digital pins 2 and 4, common - to pin 3
 * 
 *  created 03 Aug 2018
 *  by Alexey Khudyakov
 */

// include the SPI library:
#include <SPI.h>
// include the Encoder library:
#include <Encoder.h>

// Chip address must correspond to the state of MUSES72320 pins 29..31
const uint8_t musesChipAddress = 0;

// SPI pins are set by the SPI peripheral and cannot be changed
// define zcenb and csb pins for the MUSES72302
const int zcenPin = 8;  // zero crossing enable, active low
const int csbPin = 10;  // chip select, active low
// define encoder pins
const int encoderPinA = 2;
const int encoderCommon = 3; 
const int encoderPinB = 4;

// define the default volume after reset, as well as max and min limits
// note these are encoder setting, which are translated into control words
// by the logic below
const long defaultVolume = -1L;   // = 0 dB; ok for a demo but would be annoying in practice
const long minVolume = -448;    // = mute
const long maxVolume = 63L;      // = +31.5 dB

Encoder myEncoder(encoderPinA, encoderPinB);
long oldVolume = defaultVolume;

// MUSES72302 register addresses
// Each register is 16 bit
// Bits 15..8 contain the data (volume setting or control bits)
// Bits 7..4 are register address
// Bits 3..0 are chip address, corresponding to the state of MUSES72320 pins 29..31.
const uint8_t musesLeftAtt   = 0;       // Left channel attenuation
const uint8_t musesLeftGain  = 0x01<<4; // Left channel gain
const uint8_t musesRightAtt  = 0x02<<4; // Right channel attenuation
const uint8_t musesRightGain = 0x03<<4; // Right channel gain

// when the lower 4 bits of the gain byte are zeroes, one can set bit 6 to increase gain by 0.25dB
// this is not very useful as a gain setting, but can be considered the LSB of the attenuation setting,
// increasing the attenuation resolution to approx. 9 bits
const uint8_t musesQDbBit    = 0x01<<6; 

const uint8_t musesControl   = 0x04<<4; // Control bits
const uint8_t musesRegMask   = 0x07<<4; // mask for register address
const uint8_t musesZcBit     = 0x01<<5; // set bit 5 LOW in the data word and set zcenPin LOW to enable zero crossing detection
const uint8_t musesLRAtt     = 0x01<<7; // set bit 7 HIGH to link right channel attenuation to left channel control register
const uint8_t musesLRGain    = 0x01<<6; // set bit 6 HIGH to link right channel gain to left channel register

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

  // set the ZCEN pin as an output and activate it
  pinMode(zcenPin, OUTPUT);
  digitalWrite(zcenPin, LOW);  
  
  // initialize SPI
  SPI.begin();
  // MUSES72320 SPI interface is rated at 250kHz maximum
  SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE0));

  // on reset, MUSES72320 is muted, so we set the default volume
  musesWriteVolume(defaultVolume, defaultVolume);
  // Serial.begin(9600);
}

void loop() {
  long newVolume = myEncoder.read();
  if (newVolume != oldVolume) {
    oldVolume = constrain(newVolume, minVolume, maxVolume);
    myEncoder.write(oldVolume);
    musesWriteVolume(oldVolume, oldVolume);
    // for debug - requires Serial.begin() uncommented in setup() above
    // Serial.println(oldVolume);
  }
}

uint8_t volumeToAtt(long volume) {
  uint8_t att;
  volume = constrain(volume, minVolume, maxVolume);
  if (0 < volume) { 
    // no attenuation
    att = 0x10; 
  } else if (minVolume == volume) { 
    // mute 
    att = 0xFF; 
  } else { 
    // attenuation from 0 db (volume = 0) to -115.5 dbB (volume == -447)
    volume--;
    volume >>= 1;
    volume = ~volume;
    att = volume & 0xFF; 
    att += 0x10;
  }
  return att;
}

uint8_t volumeToGain(long volume) {
  uint8_t gain;
  volume = constrain(volume, minVolume, maxVolume);
  if (volume > 0) { 
    // gain is +0.5dB or above
    gain = volume & 0xFF; 
  } else if (!(volume & 0x1) && (volume > minVolume)) { 
    // gain is 0.25dB 
    gain = musesQDbBit; 
  } else { 
    //unity gain
    gain = 0; 
  }
  return gain;
}

uint8_t musesWriteVolume(long rightVolume, long leftVolume) {
  uint8_t address, data;
  
  Serial.print("Vol = "); Serial.print(leftVolume);

  address = musesLeftGain | (musesChipAddress & ~musesRegMask);
  data = volumeToGain(leftVolume);
  musesWriteRaw(address, data);
  Serial.print(" gain = "); Serial.print(data);

  address = musesLeftAtt | (musesChipAddress & ~musesRegMask);
  data = volumeToAtt(leftVolume);
  musesWriteRaw(address, data);
  Serial.print(" att = "); Serial.println(data);

  address = musesRightGain | (musesChipAddress & ~musesRegMask);
  data = volumeToGain(rightVolume);
  musesWriteRaw(address, data);

  address = musesRightAtt | (musesChipAddress & ~musesRegMask);
  data = volumeToAtt(rightVolume);
  musesWriteRaw(address, data);
}

void musesWriteRaw(uint8_t address, uint8_t data) {
  // take the CSB pin low to select the chip
  digitalWrite(csbPin, LOW);
  delayMicroseconds(1);
  //  send in the address and value via SPI
  SPI.transfer(data);
  uint8_t result = SPI.transfer(address);  
  // take the CSB pin high to de-select the chip
  delayMicroseconds(1); 
  digitalWrite(csbPin, HIGH);
}

