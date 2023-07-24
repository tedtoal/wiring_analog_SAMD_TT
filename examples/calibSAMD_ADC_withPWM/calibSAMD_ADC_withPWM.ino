/*
  calibSAMD_ADC_withPWM.ino - a program to demonstrate use of ADC calibration
  function calibSAMD_ADC_withPWM().
  Created by Ted Toal, May 21, 2023.
  Released into the public domain.

  Software License Agreement (BSD License)

  Copyright (c) 2023 Ted Toal
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  1. Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
  3. Neither the name of the copyright holders nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


  Usage:

  Before running this program, read the extensive file header comments in
  calibSAMD_ADC_withPWM.h, set CALIB_ADC_DBG = 1, and optionally set
  CALIB_ADC_SCOPE to a non-zero value and pinDEBUG to a debug pin number there.
  Then set the PIN_ constants and CFG_ADC_MULT_SAMP_AVG below for your system.
*/
#include <Arduino.h>
#include <wiring_private.h>
#include <wiring_analog_SAMD_TT.h>  // Modified version of wiring_analog.h, supports changing period of PWM waveform, supports using D4-D7 pins.
#include <calibSAMD_ADC_withPWM.h>
#if CALIB_ADC_DBG
#include <monitor_printf.h>
#endif

// Define the port to be used by the global instance named "the_serial_monitor".
#if CALIB_ADC_DBG
#define MONITOR_PORT &Serial  // Enable printf output to the serial monitor port identified by variable "Serial".
#else
#define MONITOR_PORT NULL     // Disable printf output when using WITHOUT the IDE, USB port, and serial monitor.
#endif

// Set these pin numbers to valid values for your system. See comments in
// calibSAMD_ADC_withPWM.h describing these pins. These pin numbers are each
// indexes into g_APinDescription[] in variant.cpp. The values shown here are
// my values used on my system:
//  PIN_ADC         7 = D7 = PA06 = AIN6
//  PIN_PWM         4 = D4 = PA07 = TCC1 ch 1
//  PIN_AREF_OUT    6 = D6 = PA04
#define PIN_ADC 7
#define PIN_PWM 4
#define PIN_AREF_OUT 6

// Set this to 0 to disable ADC multiple sampling and averaging, or a number X
// between 1 and 10 to average 2^X samples, e.g. 6 means average 2^6 = 64 samples.
// This is internal hardware-based averaging. It produces more stable ADC values.
#define CFG_ADC_MULT_SAMP_AVG 6

//******************************************************************************
// Standard Arduino setup() function.
//
// Initialize monitor port, then call calibSAMD_ADC_withPWM().
//******************************************************************************
void setup() {
  // Initialize for using the Arduino IDE serial monitor.
  monitor.begin(MONITOR_PORT);
  monitor.printf("********************************** RESET **********************************\n");

  // Perform ADC calibration.
  calibSAMD_ADC_withPWM(PIN_ADC, PIN_PWM, PIN_AREF_OUT, CFG_ADC_MULT_SAMP_AVG);
}

//******************************************************************************
// Standard Arduino loop() function.
//******************************************************************************
void loop() {
  delay(1000);
}
