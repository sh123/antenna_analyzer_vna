# antenna_analyzer_vna

Pocket Arduino Nano 160 to 6m band VNA antenna analyzer based on AD9850 module for field days. Powered from 9V PP3 battery.

Influenced by 
* http://ra4nal.qrz.ru/vna.shtml
* https://hamprojects.wordpress.com/2016/02/21/hf-arduino-vna-english-version/

Arudino VNA Antenna Analyzer
============================

Introduction
------------
Pocket Arduino Nano based antenna VNA analyzer / plotter can be used to measure antenna
SWR from 160m band up to 6m band, real/complex impedance, return loss (s11), draw SWR chart,
draw impedance chart.

Next peripherals are in use:

 * Nokia 5110 screen, PCD8544 - https://www.sparkfun.com/datasheets/LCD/Monochrome/Nokia5110.pdf
 * AD9850 clock generator - http://www.analog.com/media/en/technical-documentation/data-sheets/AD9850.pdf
 * SGA3386 based amplifier - https://www.rf-microwave.com/datasheets/4152_Sirenza-Microdevices_SGA-3386_01.pdf
 * AD8302 gain and phase detector - http://www.analog.com/media/en/technical-documentation/data-sheets/AD8302.pdf
 * Reflectometer and attenuators as per - https://hamprojects.wordpress.com/2016/02/21/hf-arduino-vna-english-version/
 * Rotary encoder
 
Requirements:
-------------
 * AD9850 from F4GOJ - https://github.com/F4GOJ/AD9850
 * Rotary encoder library modified fork - https://github.com/sh123/Rotary/tree/rotary_button
 * Simple Timer library - https://github.com/jfturcot/SimpleTimer
 * Adafruit PCD8544 (from Arduino library)- https://github.com/adafruit/Adafruit-PCD8544-Nokia-5110-LCD-library
 * Adafruit GFX (from Arduino library) - https://github.com/adafruit/Adafruit-GFX-Library

Supported operations:
---------------------
 * Details screen - shows various numeric parameters, such as forward/reflected signals, current frequency/band. By rotating rotary encoder user can change the frequency. By short press - change the band, by long press go to next screen.
 * Real time SWR graph screen - shows partially updated SWR plot, plot is updated while user is changing the frequency using encoder, plot is shifted left or right depending on frequency change direction.
 * Frequency sweep screen - shows complete SWR plot, which is updated approximately every second.
 * Real time impedance graph screen - shows read and imaginary part of impedance on the same screen, partical graph update while user is changing frequency.
 * Impedance sweep screen - impedance graph is updated every second. 
 * Settings screen - enables user to change frequency step, which affects both rotary encoder changes and sweep screen, screen contrast, perform calibration.
 
 ![alt text](https://i62.servimg.com/u/f62/19/62/65/46/img_2036.jpg)
