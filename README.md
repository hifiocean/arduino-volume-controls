# arduino-volume-controls
Example Arduino sketches interfacing the following digital volume controls over a serial bus:

1. DS1882 (I2C)
2. MUSES72320 (SPI)
3. PGA2310 (SPI)
4. WM8816 (SPI)

The sketches have minimal functionality and, while useful, are not meant to be production code.

The sketches depend on the Encoder library (https://www.pjrc.com/teensy/td_libs_Encoder.html) for interfacing an incremental rotary encoder and on Adafruit_LiquidCrystal for using the I2C backpack for 2x16 character LCD.
