/*
  wiring_analog_SAMD_TT.ino - a program to demonstrate functions in files
    wiring_analog_SAMD_TT.h/.cp
  Created by Ted Toal, July 21, 2023
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

  This program calls function calibSAMD_ADC_withPWM() to calibrate the ADC, and
  this requires a small amount of external circuitry on your microcomputer. Read
  the extensive file header comments in calibSAMD_ADC_withPWM.h. Then wire up
  your system and set the PIN_ constants and CFG_ADC_MULT_SAMP_AVG below to your
  system values. These pins are used for ADC calibration are ALSO used by this
  program to output signals using the PWM (pulse width modulator) and to read
  voltages using the ADC (analog-to-digital converter). One additional pin is
  used, PIN_DAC (also defined below), for outputting voltages using the DAC
  (digital-to-analog converter).  Here is a short description of what this
  program puts out on the pins during the loop() function (main program
  operation):


  pin           Device          Description
  ---           ------          --------------------------------------------
  PIN_DAC       DAC             Write 16 steps, 0 to 3.3V, one step/sec, repeat.
  PIN_PWM       TCC (PWM mode)  Write 10 steps, 0% to 100%, one step/sec, then
                                    repeat, increasing period in steps from 2ms
                                    to 10ms and then starting over.
  PIN_ADC       ADC             Read capacitor voltage from PWM and printf the
                                    value to the serial monitor once/sec.
  PIN_AREF_OUT  (for AREF)      Digital output connected to AREF pin, turned on
                                    during times when ADC is reading a pin.

  Watch the output in the serial monitor window to see if the voltage value read
  by the ADC pretty closely matches the expected value output using the PWM. If
  you have an oscilloscope, monitor PIN_DAC, PIN_PWM, and PIN_ADC to verify that
  you see the expected voltages and signals.
*/
#include <Arduino.h>
#include <wiring_private.h>
#include <wiring_analog_SAMD_TT.h>  // Modified version of wiring_analog.h, supports changing period of PWM waveform, supports using D4-D7 pins.
#include <calibSAMD_ADC_withPWM.h>
#include <monitor_printf.h>
#include <floatToString.h>

// Define the port to be used by the global instance named "the_serial_monitor".
#define MONITOR_PORT &Serial  // Enable printf output to the serial monitor port identified by variable "Serial".

// Set these pin numbers to valid values for your system. See comments in
// calibSAMD_ADC_withPWM.h describing these pins. These pin numbers are each
// indexes into g_APinDescription[] in variant.cpp. The values shown here are
// my values used on my system, which used a Nano 33 IoT microcomputer that had
// an Atmel SAMD21G18 microprocessor:
//  PIN_ADC         7 = D7 = PA06 = AIN6
//  PIN_PWM         4 = D4 = PA07 = TCC1 ch 1
//  PIN_AREF_OUT    6 = D6 = PA04
//  PIN_DAC         14 = DAC0 = PA02 = AIN0
#define PIN_ADC 7
#define PIN_PWM 4
#define PIN_AREF_OUT 6
#define PIN_DAC 14

// Set this to 0 to disable ADC multiple sampling and averaging, or a number X
// between 1 and 10 to average 2^X samples, e.g. 6 means average 2^6 = 64 samples.
// This is internal hardware-based averaging. It produces more stable ADC values.
#define CFG_ADC_MULT_SAMP_AVG 6

// Maximum value to write to DAC (minimum is 0).
#define DAC_MAX 0x3FF

// PWM period value to give a period of 1ms.
#define PWM_PERIOD_1MS 48000

// PWM resolution.
static int PWMres;

// DAC step.
static uint8_t DACstep;

// PWM on-time step and period step.
static uint8_t PWMonTimeStep;
static uint8_t PWMperiodStep;

//******************************************************************************
// Standard Arduino setup() function.
//
// Initialize monitor port, call calibSAMD_ADC_withPWM(), then call functions in
// wiring_analog_SAMD_TT.c.
//******************************************************************************
void setup() {
  // Initialize for using the Arduino IDE serial monitor.
  monitor.begin(MONITOR_PORT);
  monitor.printf("********************************** RESET **********************************\n");

  // Perform ADC calibration.
  calibSAMD_ADC_withPWM(PIN_ADC, PIN_PWM, PIN_AREF_OUT, CFG_ADC_MULT_SAMP_AVG);

  // Initialize ADC read resolution, DAC write resolution, PWM write resolution.
  analogReadResolution_SAMD_TT(12);
  analogWriteResolution_DAC_SAMD_TT(10);
  analogWriteResolution_PWM_SAMD_TT(16);

  // Switch to external voltage reference for ADC.
  analogReference_SAMD_TT(AR_EXTERNAL);

  // Initialize variables.
  DACstep = 0;
  PWMonTimeStep = 0;
  PWMperiodStep = 0;

  // Show PWM resolution.
  PWMres = analogGetResolution_TCC_SAMD_TT(PIN_PWM);
  monitor.printf("PWM TCC timer has %d-bit resolution\n", PWMres);

  // Initialize PWM.
  PWMres = analogStartPWM_TCC_SAMD_TT(PIN_PWM);
}

//******************************************************************************
// Standard Arduino loop() function.
//******************************************************************************
void loop() {

  //  PIN_DAC       DAC             Write 16 steps, 0 to 3.3V, one step/sec, repeat.
  uint32_t DACvalue = (uint32_t) DAC_MAX * DACstep / 10;
  analogWrite_SAMD_TT(PIN_DAC, DACvalue);
  if (++DACstep == 10)
    DACstep = 0;

  //  PIN_PWM       TCC (PWM mode)  Write 10 steps, 0% to 100%, one step/sec, then
  //                                    repeat, increasing period in steps from 2ms
  //                                    to 10ms and then starting over.
  uint32_t PWMperiod = (uint32_t) PWM_PERIOD_1MS*2*(PWMperiodStep+1);
  uint32_t PWMonTime = (uint32_t) PWMperiod * PWMonTimeStep / 10;
  analogSetPWM_TCC_SAMD_TT(PIN_PWM, PWMres, PWMonTime, PWMperiod);

  //  PIN_ADC       ADC             Read capacitor voltage from PWM and printf the
  //                                    value to the serial monitor once/sec.
  int ADCvalue = analogRead_SAMD_TT(PIN_ADC);
  float ADCvoltage = 3.3 * ADCvalue / ADC_MAX;
  char S_ADCvoltage[20];
  floatToString(ADCvoltage, S_ADCvoltage, sizeof(S_ADCvoltage), 3);
  int PWMduty = PWMonTimeStep * 10;
  float PWMvoltage = 3.3 * PWMduty / 100;
  char S_PWMvoltage[20];
  floatToString(PWMvoltage, S_PWMvoltage, sizeof(S_PWMvoltage), 3);
  monitor.printf("PWM duty: %3d%%, corr. to %6s V   ADC value: %5d  corr. to %6s V\n",
    PWMduty, S_PWMvoltage, ADCvalue, S_ADCvoltage);

  // We only update PWM steps AFTER ADC read uses the values.
  if (++PWMonTimeStep == 10) {
    PWMonTimeStep = 0;
    if (++PWMperiodStep == 5)
      PWMperiodStep = 0;
  }

  // Delay one second.
  delay(1000);
}
