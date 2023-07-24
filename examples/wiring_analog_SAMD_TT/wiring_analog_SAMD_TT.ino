#include <Arduino.h>
#include <wiring_analog_SAMD_TT.h>
#include <monitor_printf.h>

/*
    These pins are used, chosen on SAMD21G18 using Nano 33 IoT:
        pin     Nano    SAMD21G         Used for    Description
        ---     ----    -------         --------    -----------
        14      DAC0    PA02 (AIN0)     DAC         Write 16 steps, 0 to max, one step/sec
        4       D4      PA07 (TCC1 1)   PWM         Write 10 steps, 0% to 100%, one step/sec
        7       D7      PA06 (AIN6)     ADC         Read PWM result voltage, printf to monitor once/sec
        6       D6      PA04            AREF        Digital output connected to AREF
*/
#define PIN_DAC 14
#define PIN_PWM 4
#define PIN_ADC 7
#define PIN_AREF 6

void setup() {
  // Change this to pass pins.  Should I combine this with THIS LIBRARY?
  calibSAMD_ADC_withPWM();

  analogReadResolution_SAMD_TT(12);
  analogWriteResolution_DAC_SAMD_TT(10);
  analogWriteResolution_PWM_SAMD_TT(16);
  analogReference_SAMD_TT(AR_EXTERNAL);

  @param    pin         Arduino pin number to write, an index into table
  @returns  true if successful, false if 'pin' is non-PWM or is a TC-type timer.
bool analogStartPWM_TCC_SAMD_TT(pin_size_t pin)
}

void loop() {
  @param    pin     Arduino pin number to read, an index into table
  @returns  Value read from the ADC on the specified pin, mapped linearly to the
int analogRead_SAMD_TT(pin_size_t pin)

  @param    pin     Arduino pin number to write, an index into table
  @param    value   The value to be written. This is mapped linearly FROM the
void analogWrite_SAMD_TT(pin_size_t pin, int value)

  @param    pin         Arduino pin number to write, an index into table
  @param    res         The TCC timer resolution in bits, as returned by
  @param    on_time     The new value to be written, mapped linearly the same as
  @param    period   The new PWM period. The low-order 24 bits are used. The
  @returns  true if successful, false if 'pin' is non-PWM or is a TC-type timer.
extern bool analogSetPWM_TCC_SAMD_TT(pin_size_t pin, int res, uint32_t on_time,
  uint32_t period);
}
