# antenna_analyzer
Arduino 160 to 6m band VNA antenna analyzer based on AD9850 module.

Influenced by https://hamprojects.wordpress.com/2016/02/21/hf-arduino-vna-english-version/

Arudino VNA Antenna Analyzer
============================

Introduction
------------
Arudino based antenna VNA analyzer / plotter can be used to measure antenna
SWR from 160m band up to 6m band.

Next peripherals are in use:

 * Nokia 5110 screen, PCD8544
 * Rotary encoder
 * AD9850 clock generator - www.analog.com/media/en/technical-documentation/data-sheets/AD9850.pdf
 * SGA3386 based amplifier - https://www.rf-microwave.com/datasheets/4152_Sirenza-Microdevices_SGA-3386_01.pdf
 * AD8302 gain and phase detector - http://www.analog.com/media/en/technical-documentation/data-sheets/AD8302.pdf
 * Reflectometer and attenuators as per - https://hamprojects.wordpress.com/2016/02/21/hf-arduino-vna-english-version/
 
Requirements:
-------------
 * Rotary encoder library modified fork - https://github.com/sh123/Rotary/tree/rotary_button
 * Simple Timer library - https://github.com/jfturcot/SimpleTimer
 * Adafruit PCD8544 (from Arduino library)- https://github.com/adafruit/Adafruit-PCD8544-Nokia-5110-LCD-library
 * Adafruit GFX (from Arduino library) - https://github.com/adafruit/Adafruit-GFX-Library
 * AD9850 (from Arduino library)

Supported operations:
---------------------
 * Details screen - shows various numeric parameters, such as forward/reflected signals, current frequency/band. By rotating rotary encoder user can change the frequency. By short press - change the band, by long press go to next screen.
 * Real time graph screen - shows partially updated SWR plot, plot is updated while user is changing the frequency using encoder, plot is shifted left or right depending on frequency change direction.
 * Frequency sweep screen - shows complete SWR plot, which is updated approximately every second.
 * Frequency step change screen - enables user to change frequency step, which affects both rotary encoder changes and sweep screen.
