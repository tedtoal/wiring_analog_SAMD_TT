# wiring_analog_SAMD_TT

## Library with changes to SAMD wiring_analog.c and ADC calibration code added

This library has two pairs of files:

**wiring_analog_SAMD_TT.h and .c:** These provide functions that directly replace those in the Arduino SAMD core file named *wiring_analog.c*, to fix some problems and provide some new features I needed.

**calibSAMD_ADC_withPWM.h and .c:** These provide a function to calibrate the ADC using least squares regression on multiple samples of a reliable set of voltages created using a simple resistor-capacitor external circuit together with a pulse-width-modulation signal created by an internal microprocessor timer.

## Changes to wiring_analog.c in wiring_analog_SAMD_TT.c

The changes made to *wiring_analog.c* within *wiring_analog_SAMD_TT.c* are:

**1.** Header file <wiring_analog_SAMD_TT.h> is provided to declare the .c functions.

**2.** Every function in *wiring_analog_SAMD_TT.c* has the same name as the original in *wiring_analog.c* but with
  "_SAMD_TT" appended, e.g. *analogRead()* becomes *analogRead_SAMD_TT()*. This allows both files to be used at the same time with minimal interference, if necessary.
  
**3.** The original *analogWrite()* function, when writing to a digital port, mapped the analog output value to 8-bits and then wrote LOW if < 128 else HIGH. The new analogWrite_SAMD_TT() writes LOW if the value is 0 and otherwise HIGH, following the usual C and C++ interpretation of an integer as being false only when it is 0.

**4.** The original *analogRead()* function didn't accept use of D4/D5/D6/D7 as analog inputs, even though they can be used that way on the SAMD21G18 microprocessor. Instead, it mapped those to A4/A5/A6/A7. The new function *analogRead_SAMD_TT()* removes that mapping and allows those pins to be used as analog inputs.

**5.** The original *analogRead()* function would disable the DAC when it had been previously written (and thus enabled) for the same pin as is being read, which may not be what is wanted if the DAC is being used to drive the ADC pin for calibration and you desire to read with the ADC the voltage that the DAC generates. New function *analogRead_disableDACoption_SAMD_TT()* is a copy of analogRead() with one additional bool argument named 'disableDAC'. The argument value is used to decide whether or not to disable the DAC. The new *analogRead_SAMD_TT()* function simply calls that function with the 'disableDAC' argument set to true.

**6.** The write resolution for PWM operations was shared with the write resolution for the DAC, and either could be changed at any time. This code separates the write resolutions, adding the functions *analogWriteResolution_DAC_SAMD_TT()* to set DAC resolution and *analogWriteResolution_PWM_SAMD_TT()* to set PWM resolution. The new *analogWriteResolution_SAMD_TT()* function sets BOTH of the new resolutions to maintain compatibility with *analogWriteResolution()*.

**7.** The original *analogWrite()* function had some problems with regard to using a timer for pulse width modulation:

> Some TCC timers are 16-bit and some are 24-bit and there is no function to tell you how many bits a given pin's timer has.
>
> It used a fixed value of 0xFFFF for the waveform period with a TCC timer even though some TCC timers are 16-bit and some are 24-bit.
>
> Often the user will want to vary the PWM period but this was not possible.
>
> The write resolution was not applied to the period value.
>
> *analogWrite()* both initialized the timer for PWM operations the first time it was called for each timer AND loaded it with the (fixed) pulse period and on-time values. There was no way to reinitialize the timer.

To solve these issues, these functions were added:

> **analogGetResolution_TCC_SAMD_TT():** return the resolution of a TCC.
>
> **analogStartPWM_TCC_SAMD_TT():** initialize a timer for PWM.
>
> **analogSetPWM_TCC_SAMD_TT():** write on-time and period values to the PWM, honoring PWM resolution when needed.
  
The PWM functionality in the **analogWrite_SAMD_TT()** function remains unchanged from *analogWrite()*.

### Example in wiring_analog_SAMD_TT.ino

See the file *wiring_analog_SAMD_TT.ino* in the library folder's examples subfolder for a program that calls several functions in *wiring_analog_SAMD_TT.c* to illustrate usage. The program also calls *calibSAMD_ADC_withPWM()* in *calibSAMD_ADC_withPWM.cpp* to calibrate the ADC, which means an external circuit with two capacitors and a resistor is required (see next section of this README file for a description). The calibration uses three port pins on the microcomputer, and the subsequent calls to *wiring_analog_SAMD_TT.c* functions use those same pins and one more, in order to illustrate writing to the digital-to-analog converter (DAC), configuring a TCC timer for pulse width modulation (PWM) and writing different PWM periods and duty cycles, and reading from the analog-to-digital converter (ADC). Once a second, a line is printed to the Arduino IDE serial monitor window show the voltage values read from PIN_ADC by the ADC along with the expected voltages produced on PIN_ADC by the PWM. An oscilloscope, if available, will show the voltages and PWM signals that are generated on three pins (PIN_DAC, PIN_PWM, and PIN_ADC) by this program.

## New ADC calibration function in calibSAMD_ADC_withPWM.h and .c

I've observed poor performance from the SAMD21G18A Nano 33 IoT analog-to-digital converter (ADC), even after using the calibration procedure in open-source program CorrectADCResponse.ino. That program assumes that 0 and 3.3V are the two voltage-ends of the ADC, but the ADC seems to have problems at the ends of its reference voltage range, at least in an earlier Nano I had. Subsequent Nanos have been better, though still not great. There is a lot of noise visible in the values read from the ADC. Turning on the averaging mode to average around 64 samples automatically helps a lot. However, I found that I could further improve the performance by recomputing the ADC gain and offset calibration parameters using least squares regression, which is the purpose of function calibSAMD_ADC_withPWM() in files calibSAMD_ADC_withPWM.h/.cpp.

The new calibration function is named **calibSAMD_ADC_withPWM()**. It computes the gain and offset parameters using the ADC and a TCC timer with pulse width modulation (PWM) and an external circuit consisting of the PWM output fed into a resistor and capacitor in series, used to generate reliable voltages between GND and 3.3V that are fed to the ADC analog input. A series of stepped voltages are converted using the ADC, and least squares regression is applied to the actual and expected results to obtain gain and offset correction values. These are loaded into the ADC registers before the function returns.

*12-bit ADC resolution and external voltage reference is set by the function and required for all ADC operation.*

This code is customized for use in a SAMD microcomputer, such as in the Arduino Nano 33 IoT. It won't work for non-SAMD systems.

### Required hardware and ports

This requires use of the following hardware and ports:

**1.** The internal analog-to-digital converter device and one ADC analog input (pinADC)

**2.** A TCC timer device and one digital output (pinPWM) that can be driven by the timer

**3.** A digital output (pin_AREF_OUT) that is tied to the AREF input for generating the ADC analog reference voltage in a manner that can be turned on and off. This is optional -- you can do away with this requirement.

**4.** Optional digital output (PIN_DEBUG) used for debugging, disableable.

### Required circuit wiring

You must provide the following circuitry on your microcontroller system:

**1.** Wire a 0.1 uF capacitor between GND and pinADC.

**2.** Wire a 10K ohm resistor between pinADC and pinPWM.

**3.** Wire a large (say 100 uF) capacitor between GND and pinAREF_OUT.

**4.** Wire pinAREF_OUT to the AREF (external analog reference voltage) pin.

**5.** Optionally, during testing you can connect PIN_DEBUG to an oscilloscope input to trigger the scope when events happen that affect pinADC and/or pinPWM. See CALIB_ADC_SCOPE.

Alternatively, wire the AREF pin directly to +3.3V and don't use pinAREF_OUT. Then delete the code herein that configures and drives pinAREF_OUT.

The pins used are defined by pin* arguments to calibSAMD_ADC_withPWM(). These are the standard "Arduino pin numbers", which are not really pin numbers at all. A "pin" number is actually an index into table g_APinDescription[], which can be found in SAMD core software file variant.cpp.

### To incorporate ADC calibration into your program

I incorporated this hardware into my thermostat system, and the calibration code runs each time the thermostat starts up. To incorporate ADC calibration into your program:

**1.** Choose pin constants for the pins you wish to use for pinADC, pinPWM, and pinAREF_OUT (optional).

**2.** Choose a value for cfgADCmultSampAvg, which determines whether or not automatic ADC multi-sampling and averaging is done. It works well and is recommended.

**3.** #include <calibSAMD_ADC_withPWM.h> in your program.

**4.** Call calibSAMD_ADC_withPWM() from setup() before using the ADC, passing it the appropriate 'pin' values and 'cfgADCmultSampAvg'.

**5.** Wire your system as shown above.

**6.** Compile your program and download it into a SAMD-based microcomputer.

**7.** When calibSAMD_ADC_withPWM() runs, it will compute and load gain and offset corrections into the ADC. Subsequent ADC reads should be more accurate.

*This requires use of the file named wiring_analog_SAMD_TT.c, which is part of this library.*

### To use debugging features to check out ADC calibration hardware and operation

*calibSAMD_ADC_withPWM()* optionally uses the library module named monitor.printf(). If you enable it by setting CALIB_ADC_DBG to 1 and calling monitor.begin() from your setup() function, calibSAMD_ADC_withPWM() will print out debugging information on the serial monitor of the Arduino IDE. Look at the values printed for gainError and offsetError and ADC values that you can check to make sure the function is working properly.

You can copy and paste ADC output values into the spreadsheet file "calibSAMD_ADC_withPWM.xlsx" (extras subfolder within the library folder) into one of the data columns to plot the input/output error curve.

I used an oscilloscope to debug this module. If you have a scope, you can enable the debugging code I used and monitor pinPWM and pinADC and PIN_DEBUG with it to check out the external PWM circuit. See CALIB_ADC_SCOPE.

You may find through debugging that you need to adjust certain constants:

> AREF_STABLE_DELAY
>
> PWM_STABLE_DELAY
>
> PERCENT_AT_ENDS

Most likely, the values already set will work fine, if you use the recommended capacitor and resistor values.

### Example in calibSAMD_ADC_withPWM.ino

See the file *calibSAMD_ADC_withPWM.ino* in the library folder's examples subfolder for a program that calls calibSAMD_ADC_withPWM() during its setup() function.

## Finding variant.cpp and other system files

Here are some ways to view and get information about system files.

Right-click on "#include <Arduino.h>" in Arduino IDE and choose "Go to Definition" to open the file in the IDE. From there, open WVariant.h, variant.h, and samd.h. In WVariant.h, right-click on "extern const PinDescription g_APinDescription[];" and choose "Go to Definition" to open variant.cpp. From samd.h, open samd21.h, and from there, open samd21g18a.h, and from there open both the instance and component .h files for the peripherals of interest, say adc.h and tcc.h. To open wiring_analog.c, insert a call to analogWrite(), right-click and choose "Go to Definition". In that file, find a call to pinPeripheral and right-click it and choose "Go to Definition" to open wiring_private.c, and look at that function to see what it does. In samd21g18a.h note the definitions of symbols like ADC and DAC (the second definition, not the first). To find the path of an open file, hover over its name in Arduino IDE open tab for that file. Also open wiring_analog_SAMD_TT.cpp and compare to wiring_analog.c.

## SAMD21G18 microprocessor datasheet

The Nana 33 IoT microcomputer uses an Atmel SAMD21G18 microprocessor. You can download the data sheet by going to:

> https://www.microchip.com/en-us/product/ATsamd21g18

and downloading the data sheet PDF (over 1000 pages, daunting!) To use it, study table 7-1 and note how g_APinDescription[] in variant.cpp uses its info. Then, for each peripheral of interest, in the Table of Contents at the top, find the chapter for that peripheral. You can read the details of the peripheral, or go to the "Register Summary" for it, click on registers and read their description. The .h files mentioned above define all registers of all peripherals in a clean easy-to-use manner. E.g., ADC CTRLB register can be accessed with ADC->CTRLB.reg, and the PRESCALER field in that register can be accessed with ADC->CTRLB.bit.PRESCALER.

## Contact

If you find a problem, please submit an issue report [here](https://github.com/tedtoal/wiring_analog_SAMD_TT/issues/new/choose).

You can contact me (the author) at [ted@tedtoal.net](ted@tedtoal.net)
