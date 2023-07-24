/*
  wiring_analog_SAMD_TT.c - Provides the same functions as SAMD wiring_analog.c
  with several changes to fix problems.

  Created by Ted Toal, July 21, 2023, by copying and editing the Arduino SAMD
  file wiring_analog.c.

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
        - Some TCC timers are 16-bit and some are 24-bit and there is no
          function to tell you how many bits a given pin's timer has.

        - It used a fixed value of 0xFFFF for the waveform period with a TCC
          timer even though some TCC timers are 16-bit and some are 24-bit.

        - Often the user will want to vary the PWM period but this was not
          possible.

        -  The write resolution was not applied to the period value.

        - analogWrite() both initialized the timer for PWM operations the first
          time it was called for each timer AND loaded it with the (fixed) pulse
          period and on-time values. There was no way to reinitialize the timer.

      To solve these issues, these functions were added:
        analogGetResolution_TCC_SAMD_TT(): return the resolution of a TCC.
        analogStartPWM_TCC_SAMD_TT(): initialize a timer for PWM.
        analogSetPWM_TCC_SAMD_TT(): write on-time and period values to the PWM,
          mapping the values for PWM resolution when necessary.

      These use uint32_t type for value and period rather than int type.
      The PWM functionality in the analogWrite_SAMD_TT() function remains
      unchanged.
*/

#include "Arduino.h"
#include <wiring_private.h>
#include <wiring_analog_SAMD_TT.h>

#ifdef __cplusplus
extern "C" {
#endif

static int _readResolution = 10;
static int _ADCResolution = 10;
static int _writeResolution_DAC = 8;
static int _writeResolution_PWM = 8;

// A table of clock control IDs for initializing GCLK.
static const uint16_t GCLK_CLKCTRL_IDs[] = {
  GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC0
  GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC1
  GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TCC2
  GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TC3
  GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC4
  GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC5
  GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC6
  GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC7
};

// Wait for synchronization of registers between the clock domains
static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncDAC() __attribute__((always_inline, unused));
static void syncDAC() {
  while (DAC->STATUS.bit.SYNCBUSY == 1)
    ;
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTC_16(Tc* TCx) __attribute__((always_inline, unused));
static void syncTC_16(Tc* TCx) {
  while (TCx->COUNT16.STATUS.bit.SYNCBUSY);
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}

// Map value from a 'from'-bits number to a 'to'-bits number, scaled linearly.
static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
  if (from == to) {
    return value;
  }
  if (from > to) {
    return value >> (from-to);
  }
  return value << (to-from);
}

/**************************************************************************/
/*!
  @brief    Set resolution of the value returned by analogRead_SAMD_TT() and
            analogRead_disableDACoption_SAMD_TT() for analog reads.
  @param    res     Desired resolution in bits.
  @note     The ADC itself supports resolutions of 8, 10, or 12 bits, and this
            resolution is set to the smallest one that not less than res.
*/
/**************************************************************************/
void analogReadResolution_SAMD_TT(int res)
{
  _readResolution = res;
  if (res > 10) {
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
    _ADCResolution = 12;
  } else if (res > 8) {
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;
    _ADCResolution = 10;
  } else {
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_8BIT_Val;
    _ADCResolution = 8;
  }
  syncADC();
}

/**************************************************************************/
/*!
  @brief    Set resolution of the value argument of analogWrite_SAMD_TT() for
            analog writes to both DAC and PWM.
  @param    res     Desired resolution in bits.
  @note     Use analogWriteResolution_DAC_SAMD_TT() or
            analogWriteResolution_PWM_SAMD_TT() to set DAC or PWM resolution
            separately from one another.
*/
/**************************************************************************/
void analogWriteResolution_SAMD_TT(int res)
{
  _writeResolution_DAC = res;
  _writeResolution_PWM = res;
}

/**************************************************************************/
/*!
  @brief    Set resolution of the value argument of analogWrite_SAMD_TT() for
            analog writes to the DAC.
  @param    res     Desired resolution in bits.
*/
/**************************************************************************/
void analogWriteResolution_DAC_SAMD_TT(int res)
{
  _writeResolution_DAC = res;
}

/**************************************************************************/
/*!
  @brief    Set resolution of the value argument of analogWrite_SAMD_TT() and
            the on_time and period arguments of analogSetPWM_TCC_SAMD_TT() for
            analog writes to the PWM.
  @param    res     Desired resolution in bits.
*/
/**************************************************************************/
void analogWriteResolution_PWM_SAMD_TT(int res)
{
  _writeResolution_PWM = res;
}

/**************************************************************************/
/*!
  @brief    Set the ADC REFERENCE INPUT mode.
  @param    mode    The ADC REFERENCE INPUT mode, an AR_ constant such as
                    AR_INTERNAL or AR_EXTERNAL.
  @note     This changes the ADC register fields INPUTCTRL.GAIN and REFCTRL.REFSEL.
  @note     Internal Reference is at 1.0v
  @note     External Reference should be between 1v and VDDANA-0.6v=2.7v
  @note     Warning: On Arduino Zero board the input/output voltage for
            SAMD21G18 is 3.3 volts maximum
/**************************************************************************/
void analogReference_SAMD_TT(eAnalogReference mode)
{
  syncADC();
  switch (mode)
  {
    case AR_INTERNAL:
    case AR_INTERNAL2V23:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val; // 1/1.48 VDDANA = 1/1.48* 3V3 = 2.2297
      break;

    case AR_EXTERNAL:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val;
      break;

    case AR_INTERNAL1V0:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INT1V_Val;   // 1.0V voltage reference
      break;

    case AR_INTERNAL1V65:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain Factor Selection
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // 1/2 VDDANA = 0.5* 3V3 = 1.65V
      break;

    case AR_DEFAULT:
    default:
      ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;
      ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; // 1/2 VDDANA = 0.5* 3V3 = 1.65V
      break;
  }
}

/**************************************************************************/
/*!
  @brief    Use the ADC to read an analog value from a pin.
  @param    pin     Arduino pin number to read, an index into table
                    g_APinDescription[] in SAMD core file variant.cpp.
  @returns  Value read from the ADC on the specified pin, mapped linearly to the
            resolution requested by calling analogReadResolution_SAMD_TT().
  @note     This simply calls analogRead_disableDACoption_SAMD_TT() with the
            argument disableDAC = true. See description of that function.
*/
/**************************************************************************/
int analogRead_SAMD_TT(pin_size_t pin)
{
  return(analogRead_disableDACoption_SAMD_TT(pin, true));
}

/**************************************************************************/
/*!
  @brief    Use the ADC to read an analog value from a pin.
  @param    pin         Arduino pin number to read, an index into table
                        g_APinDescription[] in SAMD core file variant.cpp.
  @param    disableDAC  true to disable the DAC if 'pin' is both an ADC and DAC
                        pin and it has previously been enabled for the DAC.
                        Use false to NOT disable it, which you would do if you
                        want to simultaneously write an analog value to the pin
                        using the DAC and read the same value back using the
                        ADC, perhaps to calibrate the ADC.
  @returns  Value read from the ADC on the specified pin, mapped linearly to the
            resolution requested by calling analogReadResolution_SAMD_TT().
  @note     In the SAMD architecture, some digital pins (e.g. D4) can also be
            used as analog input pins. See the g_APinDescription[] table in file
            variant.cpp.
*/
/**************************************************************************/
int analogRead_disableDACoption_SAMD_TT(pin_size_t pin, bool disableDAC)
{
  uint32_t valueRead = 0;

  // Comment out pin mapping.
  //if (pin < A0) {
  //  pin += A0;
  //}

  pinPeripheral(pin, PIO_ANALOG);

  // Disable DAC, if analogWrite() was used previously to enable the DAC
  if (disableDAC)
    if ((g_APinDescription[pin].ulADCChannelNumber == ADC_Channel0) || (g_APinDescription[pin].ulADCChannelNumber == DAC_Channel0)) {
      syncDAC();
      DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC
      //DAC->CTRLB.bit.EOEN = 0x00; // The DAC output is turned off.
      syncDAC();
    }

  syncADC();
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber; // Selection for the positive ADC input

  // Control A
  /*
   * Bit 1 ENABLE: Enable
   *   0: The ADC is disabled.
   *   1: The ADC is enabled.
   * Due to synchronization, there is a delay from writing CTRLA.ENABLE until the peripheral is enabled/disabled. The
   * value written to CTRL.ENABLE will read back immediately and the Synchronization Busy bit in the Status register
   * (STATUS.SYNCBUSY) will be set. STATUS.SYNCBUSY will be cleared when the operation is complete.
   *
   * Before enabling the ADC, the asynchronous clock source must be selected and enabled, and the ADC reference must be
   * configured. The first conversion after the reference is changed must not be used.
   */
  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC

  // Start conversion
  syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Waiting for the 1st conversion to complete
  while (ADC->INTFLAG.bit.RESRDY == 0);

  // Clear the Data Ready flag
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  // Start conversion again, since The first conversion after the reference is changed must not be used.
  syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Store the value
  while (ADC->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
  valueRead = ADC->RESULT.reg;

  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  syncADC();

  return mapResolution(valueRead, _ADCResolution, _readResolution);
}

/**************************************************************************/
/*!
  @brief    Write a value to the DAC or PWM device or to a digital port.
  @param    pin     Arduino pin number to write, an index into table
                    g_APinDescription[] in SAMD core file variant.cpp.
  @param    value   The value to be written. This is mapped linearly FROM the
                    resolution requested by calling one of the functions
                    analogWriteResolution_SAMD_TT(),
                    analogWriteResolution_DAC_SAMD_TT(), or
                    analogWriteResolution_PWM_SAMD_TT() (using the appropriate
                    resolution, DAC or PWM, depending on whether 'pin' is a DAC
                    or PWM pin) TO the 10-bit resolution required by the DAC or
                    the 16-bit resolution required by the PWM.
  @note     'pin' could be a DAC, PWM, or regular digital port. Which one it is
            is determined by examining the ulPinAttribute column of the table
            g_APinDescription[] (DAC is PIN_ATTR_ANALOG, PWM is PIN_ATTR_PWM,
            else it is assumed to be a digital port).
  @note     If 'pin' is a digital port, it is set LOW if value=0, else HIGH.
  @note     The PWM period is fixed at 0xFFFF in this function.
  @note     This supports PWM on both TC and TCC timers.
  @note     PWM output only works on the pins with hardware support. These are
            defined in the appropriate pins_*.c file. For the rest of the pins,
            the default is digital output.
  @note     This does not allow separate control of PWM initialization and
            subsequent updating of the PWM value. To do that, see functions
            analogStartPWM_TCC_SAMD_TT() and analogSetPWM_TCC_SAMD_TT().
*/
/**************************************************************************/
void analogWrite_SAMD_TT(pin_size_t pin, int value)
{
  PinDescription pinDesc = g_APinDescription[pin];
  uint32_t attr = pinDesc.ulPinAttribute;

  if ((attr & PIN_ATTR_ANALOG) == PIN_ATTR_ANALOG)
  {
    // DAC handling code

    if ((pinDesc.ulADCChannelNumber != ADC_Channel0) && (pinDesc.ulADCChannelNumber != DAC_Channel0)) { // Only 1 DAC on AIN0 / PA02
      return;
    }

    value = mapResolution(value, _writeResolution_DAC, 10);

    syncDAC();
    DAC->DATA.reg = value & 0x3FF;  // DAC on 10 bits.
    syncDAC();
    DAC->CTRLA.bit.ENABLE = 0x01;     // Enable DAC
    syncDAC();
    return;
  }

  if ((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM)
  {
    value = mapResolution(value, _writeResolution_PWM, 16);

    uint32_t tcNum = GetTCNumber(pinDesc.ulPWMChannel);
    uint8_t tcChannel = GetTCChannelNumber(pinDesc.ulPWMChannel);
    static bool tcEnabled[TCC_INST_NUM+TC_INST_NUM];

    if (attr & PIN_ATTR_TIMER) {
      #if !(ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10603)
      // Compatibility for cores based on SAMD core <=1.6.2
      if (pinDesc.ulPinType == PIO_TIMER_ALT) {
        pinPeripheral(pin, PIO_TIMER_ALT);
      } else
      #endif
      {
        pinPeripheral(pin, PIO_TIMER);
      }
    } else {
      // We suppose that attr has PIN_ATTR_TIMER_ALT bit set...
      pinPeripheral(pin, PIO_TIMER_ALT);
    }

    if (!tcEnabled[tcNum]) {
      tcEnabled[tcNum] = true;

      GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_IDs[tcNum]);
      while (GCLK->STATUS.bit.SYNCBUSY == 1);

      // Set PORT
      if (tcNum >= TCC_INST_NUM) {
        // -- Configure TC
        Tc* TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);
        // Disable TCx
        TCx->COUNT16.CTRLA.bit.ENABLE = 0;
        syncTC_16(TCx);
        // Set Timer counter Mode to 16 bits, normal PWM
        TCx->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_NPWM;
        syncTC_16(TCx);
        // Set the initial value
        TCx->COUNT16.CC[tcChannel].reg = (uint32_t) value;
        syncTC_16(TCx);
        // Enable TCx
        TCx->COUNT16.CTRLA.bit.ENABLE = 1;
        syncTC_16(TCx);
      } else {
        // -- Configure TCC
        Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
        // Disable TCCx
        TCCx->CTRLA.bit.ENABLE = 0;
        syncTCC(TCCx);
        // Set TCCx as normal PWM
        TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
        syncTCC(TCCx);
        // Set the initial value
        TCCx->CC[tcChannel].reg = (uint32_t) value;
        syncTCC(TCCx);
        // Set PER to maximum counter value (resolution : 0xFFFF)
        TCCx->PER.reg = 0xFFFF;
        syncTCC(TCCx);
        // Enable TCCx
        TCCx->CTRLA.bit.ENABLE = 1;
        syncTCC(TCCx);
      }
    } else {
      if (tcNum >= TCC_INST_NUM) {
        Tc* TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);
        TCx->COUNT16.CC[tcChannel].reg = (uint32_t) value;
        syncTC_16(TCx);
      } else {
        Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
        TCCx->CTRLBSET.bit.LUPD = 1;
        syncTCC(TCCx);
        TCCx->CCB[tcChannel].reg = (uint32_t) value;
        syncTCC(TCCx);
        TCCx->CTRLBCLR.bit.LUPD = 1;
        syncTCC(TCCx);
      }
    }
    return;
  }

  // -- Defaults to digital write
  pinMode(pin, OUTPUT);
  digitalWrite(pin, value == 0 ? LOW : HIGH);
}

/**************************************************************************/
/*!
  @brief    Return the resolution of a TCC.
  @param    pin         Arduino pin number assigned to TCC, an index into table
                        g_APinDescription[] in SAMD core file variant.cpp.
  @returns  16 or 24 if TCC is a 16-bit or 24-bit timer, 0 if 'pin' is non-PWM
            or is a TC-type timer.
  @note     This resets the TCC timer, it must be reinitialized to use.
  @note     A portion of TCC timer initialization is done here, which is relied
            upon by analogStartPWM_TCC_SAMD_TT().
*/
/**************************************************************************/
int analogGetResolution_TCC_SAMD_TT(pin_size_t pin) {
  PinDescription pinDesc = g_APinDescription[pin];

  // Return 0 if non-PWM pin specified.
  uint32_t attr = pinDesc.ulPinAttribute;
  if ((attr & PIN_ATTR_PWM) != PIN_ATTR_PWM)
    return(0);

  // Return 0 if TC-type timer rather than TCC timer.
  uint32_t tcNum = GetTCNumber(pinDesc.ulPWMChannel);
  if (tcNum >= TCC_INST_NUM)
    return(0);

  uint8_t tcChannel = GetTCChannelNumber(pinDesc.ulPWMChannel);

  // Configure the pin.
  if (attr & PIN_ATTR_TIMER) {
    #if !(ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10603)
    // Compatibility for cores based on SAMD core <=1.6.2
    if (pinDesc.ulPinType == PIO_TIMER_ALT) {
      pinPeripheral(pin, PIO_TIMER_ALT);
    } else
    #endif
    {
      pinPeripheral(pin, PIO_TIMER);
    }
  } else {
    // We suppose that attr has PIN_ATTR_TIMER_ALT bit set...
    pinPeripheral(pin, PIO_TIMER_ALT);
  }

  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_IDs[tcNum]);
  while (GCLK->STATUS.bit.SYNCBUSY == 1);

  // Get the TCC pointer.
  Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);

  // Disable TCC.
  TCCx->CTRLA.bit.ENABLE = 0;
  syncTCC(TCCx);

  // Set TCC as normal PWM
  TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
  syncTCC(TCCx);

  // Test if TCC is 24-bit. If not, it is 16-bit.
  TCCx->PER.reg = 0xFFFFFFUL;
  if (TCCx->PER.reg == 0xFFFFFFUL)
    return(24);
  return(16);
}

/**************************************************************************/
/*!
  @brief    Initialize the PWM and set its initial period and duty cycle.
  @param    pin         Arduino pin number to write, an index into table
                        g_APinDescription[] in SAMD core file variant.cpp.
  @returns  TCC timer resolution in bits, 16 or 24 if TCC is a 16-bit or 24-bit
            timer, or 0 if 'pin' is non-PWM or is a TC-type timer.
  @note     This supports PWM only on TCC timers.
  @note     Single-slope (NPWM) mode is used.
  @note     Dithering is not enabled.
  @note     The generic clock used depends on the TCC which depends on the pin,
            see g_APinDescription[]. It will be either GCM_TCC0_TCC1 or
            GCM_TCC2_TC3 for the SAMD21G.
  @note     This initializes the pulse on-time to 0 and pulse width to an
            arbitrary setting, so the PWM output remains LOW until you call
            analogSetPWM_TCC_SAMD_TT() to set a pulse on-time and period.
*/
/**************************************************************************/
int analogStartPWM_TCC_SAMD_TT(pin_size_t pin)
{
  int res = analogGetResolution_TCC_SAMD_TT(pin);
  if (res == 0)
    return(0);
  PinDescription pinDesc = g_APinDescription[pin];
  uint8_t tcChannel = GetTCChannelNumber(pinDesc.ulPWMChannel);
  Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);

  // Set the initial CC and CCB register values to 0.
  TCCx->CC[tcChannel].reg = 0;
  syncTCC(TCCx);
  TCCx->CCB[tcChannel].reg = 0;
  syncTCC(TCCx);
  // Set both PER and PERB to an arbitrary non-zero value. Since the minimum TCC
  // size is 16 bits, 0xFFFF will work fine.
  TCCx->PER.reg = 0xFFFF;
  syncTCC(TCCx);
  TCCx->PERB.reg = 0xFFFF;
  syncTCC(TCCx);
  // Enable TCCx
  TCCx->CTRLA.bit.ENABLE = 1;
  syncTCC(TCCx);
  return(res);
}

/**************************************************************************/
/*!
  @brief    Update the PWM on-time and period. It must already be initialized.
  @param    pin         Arduino pin number to write, an index into table
                        g_APinDescription[] in SAMD core file variant.cpp.
  @param    res         The TCC timer resolution in bits, as returned by
                        analogStartPWM_TCC_SAMD_TT().
  @param    on_time     The pulse on-time value, 0 <= on_time <= period. This is
                        mapped linearly FROM the resolution requested by calling
                        one of the functions analogWriteResolution_PWM_SAMD_TT()
                        or analogWriteResolution_SAMD_TT() TO the ACTUAL
                        resolution of the TCC timer implied by the 'pin'
                        argument ONLY IF THAT ACTUAL RESOLUTION IS LOWER THAN
                        THE RESOLUTION THAT WAS REQUESTED. For example, if
                        requested resolution is 16 bits and the TCC is 24 bits,
                        no mapping is done, but if the requested resolution is
                        24 bits and the TCC is 16 bits, mapping is done.
  @param    period      The initial PWM period, 1 <= period <= 2^resolution-1 of
                        resolution set with analogWriteResolution_PWM_SAMD_TT().
                        This is also mapped linearly in the same way as'on_time.
                            100% * (on_time/period).
  @returns  true if successful, false if 'pin' is non-PWM or is a TC-type timer
            or on_time > period or period > 2^res-1.
*/
/**************************************************************************/
bool analogSetPWM_TCC_SAMD_TT(pin_size_t pin, int res, uint32_t on_time,
    uint32_t period)
{
  if (res == 0 || on_time > period || period > ((1UL << res)-1))
    return(false);
  PinDescription pinDesc = g_APinDescription[pin];
  uint8_t tcChannel = GetTCChannelNumber(pinDesc.ulPWMChannel);

  // Get the TCC pointer.
  Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);

  // Update the TCC.
  TCCx->CTRLBSET.bit.LUPD = 1;
  syncTCC(TCCx);
  if (res < _writeResolution_PWM)
    on_time = mapResolution(on_time, _writeResolution_PWM, res);
  TCCx->CCB[tcChannel].reg = on_time;
  syncTCC(TCCx);
  if (res < _writeResolution_PWM)
    period = mapResolution(period, _writeResolution_PWM, res);
  TCCx->PERB.reg = period;
  syncTCC(TCCx);
  TCCx->CTRLBCLR.bit.LUPD = 1;
  syncTCC(TCCx);
  return(true);
}

#ifdef __cplusplus
}
#endif

// -------------------------------------------------------------------------
