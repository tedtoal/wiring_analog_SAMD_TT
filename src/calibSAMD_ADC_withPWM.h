/*
  calibSAMD_ADC_withPWM.h - a function to calibrate the SAMD ADC.
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


  ******************************************************************************
  Details:

  I've observed poor performance from the SAMD21G18A Nano 33 IoT ADC, even after
  using the calibration procedure in open-source program CorrectADCResponse.ino.
  That program assumes that 0 and 3.3V are the two voltage-ends of the ADC, but
  the ADC seems to have problems at the ends of its reference voltage range, at
  least in an earlier Nano I had. Subsequent Nanos have been better, though
  still not great, with a lot of noise visible in the values read from the ADC.
  Turning on the averaging mode to average around 64 samples automatically helps
  a lot. I found that I could further improve the performance by recomputing the
  ADC gain and offset calibration parameters.

  This module defines function calibSAMD_ADC_withPWM() that computes those
  parameters using the ADC and a TCC timer with pulse width modulation (PWM) and
  an external circuit consisting of the PWM output fed into a resistor and
  capacitor in series, used to generate reliable voltages between GND and 3.3V
  that are fed to the ADC analog input. The new gain and offset values are
  loaded into the ADC registers before the function returns.

  NOTE: 12-bit ADC resolution and external voltage reference is set and required.

  This code is customized for use in a SAMD microcomputer, such as in the
  Arduino Nano 33 IoT. It won't work for non-SAMD systems.

  This configures the ADC for 12-bit operation with external voltage reference,
  and assumes the user uses it that way.

  This requires use of the following hardware and ports:
    1. The ADC and one ADC analog input
    2. A TCC timer and one digital output that can be driven by the timer
    3. A digital output that is tied to the AREF input for generating the ADC
      analog reference voltage in a manner that can be turned on and off.

  Circuit wiring:
   1. Wire a 0.1 uF capacitor between GND and pin pinADC.
   2. Wire a 10K ohm resistor between pinADC and pinPWM.
   3. Wire a large (say 100 uF) capacitor between GND and pinAREF_OUT.
   4. Wire pinAREF_OUT to the AREF (external analog reference voltage) pin.
   5. Optionally, during testing you can connect PIN_DEBUG to an oscilloscope
      input to trigger the scope when events happen that affect pinADC and/or
      pinPWM. See CALIB_ADC_SCOPE below.

  Alternatively, wire the AREF pin directly to +3.3V and don't use pinAREF_OUT.
  (Delete the code herein that configures and drives pinAREF_OUT).

  I incorporated this hardware into my thermostat system, and the calibration
  code runs each time the thermostat starts up.

  The pins used are defined by pin* arguments to calibSAMD_ADC_withPWM(). These
  are the standard "Arduino pin numbers", which are not really pin numbers at
  all. A "pin" number is actually an index into table g_APinDescription[], which
  can be found in SAMD core software file variant.cpp.

  NOTES ON FINDING variant.cpp and other system files:
  Right-click on "#include <Arduino.h>" in Arduino IDE and choose "Go to
  Definition" to open the file in the IDE. From there, open WVariant.h,
  variant.h, and samd.h. In WVariant.h, right-click on "extern const
  PinDescription g_APinDescription[];" and choose "Go to Definition" to open
  variant.cpp. From samd.h, open samd21.h, and from there, open samd21g18a.h,
  and from there open both the instance and component .h files for the
  peripherals of interest, say adc.h and tcc.h. To open wiring_analog.c,
  insert a call to analogWrite(), right-click and choose "Go to Definition".
  In that file, find a call to pinPeripheral and right-click it and choose "Go
  to Definition" to open wiring_private.c, and look at that function to see
  what it does. In samd21g18a.h note the definitions of symbols like ADC and
  DAC (the second definition, not the first). To find the path of an open
  file, hover over its name in Arduino IDE open tab for that file. Also open
  wiring_analog_SAMD_TT.cpp and compare to wiring_analog.c.

  NOTE ON MICROPROCESSOR DATASHEET: download the data sheet by going to:
  https://www.microchip.com/en-us/product/ATsamd21g18
  and downloading the data sheet PDF (over 1000 pages, daunting!) To use it,
  study table 7-1 and note how g_APinDescription[] in variant.cpp uses its
  info. Then, for each peripheral of interest, in the Table of Contents at the
  top, find the chapter for that peripheral. You can read the details of the
  peripheral, or go to the "Register Summary" for it, click on registers and
  read their description. The .h files mentioned above define all registers of
  all peripherals in a clean easy-to-use manner. E.g., ADC CTRLB register can
  be accessed with ADC->CTRLB.reg, and the PRESCALER field in that register
  can be accessed with ADC->CTRLB.bit.PRESCALER.

  To use this module:
    1. Choose pin constants for the pins you wish to use for pinADC, pinPWM,
        and pinAREF_OUT (optional).
    2. #include <calibSAMD_ADC_withPWM.h> in your program.
    3. Call calibSAMD_ADC_withPWM() from setup() before using the ADC, passing
        it the appropriate 'pin' values and other arguments for configuring the
        calibration operation.
    4. Wire your system as shown above.
    5. Compile your program and download it into a SAMD-based microcomputer.
    6. When calibSAMD_ADC_withPWM() runs, it will compute and load gain and
        offset corrections into the ADC. Subsequent ADC reads should be more
        accurate.

  This requires use of the file named wiring_analog_SAMD_TT.c, which is part of
  the library containing this file.

  This optionally uses the library module named monitor.printf(). If you enable
  it below by setting CALIB_ADC_DBG to 1 and calling monitor.begin() from your
  setup() function, calibSAMD_ADC_withPWM() will print out debugging information
  on the serial monitor of the Arduino IDE. Look at the values printed for
  gainError and offsetError and ADC values that you can check to make sure the
  function is working properly. You can copy and paste ADC output values into
  the spreadsheet file "calibSAMD_ADC_withPWM.xlsx" into one of the data columns
  to plot the input/output curve.

  I used an oscilloscope to debug this module. If you have a scope, you can
  enable the debugging code I used and monitor pinPWM and pinADC and PIN_DEBUG
  with it to check out the external PWM circuit. See CALIB_ADC_SCOPE below.

  You may find through debugging that you need to adjust certain constants below:
    AREF_STABLE_DELAY
    PWM_STABLE_DELAY
    PERCENT_AT_ENDS
  ******************************************************************************
*/

#ifndef calibSAMD_ADC_withPWM_h
#define calibSAMD_ADC_withPWM_h

#include <Arduino.h>

// Set this to 1 to #include monitor_printf.h and print debug results of the
// calibration procedure to the Arduino IDE monitor when calibSAMD_ADC_withPWM()
// is called. If set to 1, you must call monitor.begin() from setup() BEFORE
// calling calibSAMD_ADC_withPWM().
#define CALIB_ADC_DBG 0

// Maximum ADC output value (minimum is 0).
#define ADC_MAX 0xFFF

// Number of milliseconds to delay after turning on AREF before the reference
// voltage is stable. The larger the capacitor on pinAREF_OUT, the longer this
// should be.
#define AREF_STABLE_DELAY 5

// Number of milliseconds of delay after turning on PWM output before the
// voltage at the PWM capacitor is stable, in the worst case. This value can be
// determined with a scope and a section of code below, see comments below.
#define PWM_STABLE_DELAY 5

// Percentage of ADC range to use, e.g. if this is 10, measure the ADC value at
// 10% and 90% of reference voltage and compute gain and offset corrections.
#define PERCENT_AT_ENDS 10

// Set this to 0 to disable all oscilloscope-related code. Otherwise, set it to
// an integer between 1 and 6 to enable a section of the calibration code that
// does a specific type of test that you can monitor on your scope:
//    1 - test pinPWM as a regular digital output pin. Watch it on a scope.
//    2 - test ADC values at 20% of max. Use a scope to watch the PWM waveform.
//          The PIN_DEBUG output is HIGH during the test.
//    3 - test PWM output rise time. Watch pinPWM and pinADC with scope,
//          triggering on PIN_DEBUG going high. It's a bit less than 3 ms.
//    4 - test PWM output rise time from 0% to PERCENT_AT_ENDS of max. Watch
//          pinPWM with scope, triggering on PIN_DEBUG going high. It's a bit
//          less than 5 ms from 0 to max voltage.
//    5 - test PWM output rise time from 100-PERCENT_AT_ENDS to 100% of max.
//          Watch pinPWM with scope, triggering on PIN_DEBUG going high. It's a
//          bit less than 2 ms.
//    6 - test ADC values at 0, quarter-max, half-max, 3/4-max, and max. The
//          PIN_DEBUG output is high during the test.
//  Work through these values one at a time and check signals to see that
//  everything looks good.
#define CALIB_ADC_SCOPE 0

// Arduino "pin" number of digital output pin to use for debug output (e.g. to
// trigger a scope). Used in conjunction with CALIB_ADC_SCOPE above. This is an
// index into g_APinDescription[] in variant.cpp. If CALIB_ADC_SCOPE is 0, set
// this to -1. On my system, when debugging I used a value of 5 = D5 = PA05.
#if CALIB_ADC_SCOPE > 0
#define PIN_DEBUG 5
#else
#define PIN_DEBUG -1
#endif

/**************************************************************************/
/*!
  @brief    Run a calibration algorithm on the ADC using the PWM output and TCC
            timer, compute ADC gain and offset constants, and load them into the
            ADC.
  @param    pinADC      Arduino "pin" number of an ADC-available analog input
                        pin to use to read the calibration voltage. This is
                        an index into g_APinDescription[] in variant.cpp, and
                        the table must include ADC_Channel# for this pin.
                        This pin has an 0.1 uF capacitor tied between it and
                        ground, and a 10K ohm resistor tied between it and
                        pinPWM. The default value is what I used on my system:
                          7 = D7 = PA06 = AIN6
  @param    pinPWM      Arduino "pin" number of digital output pin to use to
                        generate a PWM waveform using a SAMDG21 TCC timer. This
                        is an index into g_APinDescription[] in variant.cpp, and
                        the table must include TCC#_CH# and PIN_ATTR_PWM. This
                        pin has a 10K ohm resistor tied between it and pinADC.
                        Note: A TCC timer is used rather than a TC timer in
                        order that the period of the waveform can be set. The
                        default value is what I used on my system:
                          4 = D4 = PA07 = TCC1 ch 1
  @param    pinAREF_OUT Arduino "pin" number of digital output pin to use to
                        generate the ADC external reference voltage. This is an
                        index into g_APinDescription[] in variant.cpp, and the
                        table must include PIN_ATTR_DIGITAL. This pin has a
                        100uF capacitor tied between it and ground, and the pin
                        is also tied to the external AREF input. If you don'
                        want to use this pin but instead want to tie the AREF
                        input directly to 3.3V, set this to -1. The default
                        value is what I used on my system:
                          6 = D6 = PA04
  @param  cfgADCmultSampAvg   Set this to 0 to disable ADC automatic multiple
                              sampling and averaging, or a number X between 1
                              and 10 to average 2^X samples, e.g. 6 means
                              average 2^6 = 64 samples (seems to give good
                              results). This is internal hardware-based
                              averaging. It produces more stable ADC values.
*/
/**************************************************************************/
extern void calibSAMD_ADC_withPWM(int pinADC = 7, int pinPWM = 4,
  int pinAREF_OUT = 6, uint8_t cfgADCmultSampAvg = 6);

#endif // calibSAMD_ADC_withPWM_h
