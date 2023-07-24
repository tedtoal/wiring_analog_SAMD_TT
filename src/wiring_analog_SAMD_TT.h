/*
  wiring_analog_SAMD_TT.h - Provides the same functions as SAMD wiring_analog.c
  with several changes to fix problems.

  Created by Ted Toal, July 21, 2023, declaring the functions that are defined
  in wiring_analog_SAMD_TT.c.

  Brief summary of changes to wiring_analog_SAMD_TT.c from wiring_analog.c:
    1. File <wiring_analog_SAMD_TT.h> is provided to declare the .c functions.
    2. Every function in this file has the same name as the original but with
      "_SAMD_TT" appended, e.g. analogRead() becomes analogRead_SAMD_TT().
    3. The original analogWrite(), when writing to a digital port, mapped the
      analog output value to 8-bits and then wrote LOW if < 128 else HIGH. The
      new analogWrite_SAMD_TT() writes LOW if the value is 0 and otherwise HIGH.
    4. The original analogRead() function doesn't accept use of D4/D5/D6/D7
      as analog inputs, even though they can be used that way. Instead, it maps
      those to A4/A5/A6/A7. analogRead_SAMD_TT() removes that mapping and allows
      those pins to be used as analog inputs.
    5. The original analogRead() function would disable the DAC when it has been
      previously written (and thus enabled) for the same pin as is being read,
      which may not be what is wanted if the DAC is being used to drive the ADC
      pin for calibration. Function analogRead_disableDACoption_SAMD_TT() is
      added, that is a copy of analogRead() with one additional bool argument
      named 'disableDAC'. The argument value is used to decide whether or not to
      disable the DAC. The analogRead_SAMD_TT() function simply calls that
      function with the 'disableDAC' argument set to true.
    6. The write resolution for the PWM was shared with that for the DAC, and
      could be changed at any time. This code separates the write resolutions,
      adding the functions analogWriteResolution_DAC_SAMD_TT() and
      analogWriteResolution_PWM_SAMD_TT(). The analogWriteResolution_SAMD_TT()
      function sets BOTH of the new resolutions to maintain compatibility with
      analogWriteResolution().
    7. The original analogWrite() function had some problems with regard to
      using a timer for pulse width modulation:
      - It used a fixed value of 0xFFFF for the waveform period with a TCC timer
        even though some TCC timers are 16-bit and some are 24-bit and often the
        user will want to vary the period. The write resolution was not applied
        to the period value.
      - On the first call to analogWrite() for a given timer, the timer is
        initialized and the CC register is written, but not the CCB register.
        On subsequent calls the timer is not initialized and the CCB register
        is written. Instead, the initialization should be separate from update,
        and the user should be able to re-initialize a timer whenever he wants.
      To solve these issues, these functions were added:
        analogGetResolution_TCC_SAMD_TT(): return the resolution of a TCC.
        analogStartPWM_TCC_SAMD_TT(): initialize a timer for PWM.
        analogSetPWM_TCC_SAMD_TT(): write on-time and period values to the PWM.
      These use uint32_t type for value and period rather than int type.
      The PWM functionality in the analogWrite_SAMD_TT() function remains
      unchanged.

  Copyright (c) 2014 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef wiring_analog_SAMD_TT_h
#define wiring_analog_SAMD_TT_h

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void analogReadResolution_SAMD_TT(int res);
extern void analogWriteResolution_SAMD_TT(int res);
extern void analogWriteResolution_DAC_SAMD_TT(int res);
extern void analogWriteResolution_PWM_SAMD_TT(int res);
extern void analogReference_SAMD_TT(eAnalogReference mode);
extern int analogRead_SAMD_TT(pin_size_t pin);
extern int analogRead_disableDACoption_SAMD_TT(pin_size_t pin, bool disableDAC);
extern void analogWrite_SAMD_TT(pin_size_t pin, int value);
extern int analogGetResolution_TCC_SAMD_TT(pin_size_t pin);
extern int analogStartPWM_TCC_SAMD_TT(pin_size_t pin);
extern bool analogSetPWM_TCC_SAMD_TT(pin_size_t pin, int res, uint32_t on_time,
  uint32_t period);

#ifdef __cplusplus
}
#endif

#endif // wiring_analog_SAMD_TT_h
