/*
  calibSAMD_ADC_withPWM.cpp - a function to calibrate the SAMD ADC.
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
*/

#include <Arduino.h>
#include <wiring_private.h>
#include <wiring_analog_SAMD_TT.h> // Modified version of wiring_analog.h, supports changing period of PWM waveform, supports using D4-D7 pins.
#include <calibSAMD_ADC_withPWM.h>
#if CALIB_ADC_DBG
#include <monitor_printf.h>
#include <floatToString.h>
#endif

// Gain correction value of 1 = no correction.
#define GAIN_CORR_1 0x800

// PWM period during ADC calibration.  We use a period of 1000 to make
// percentage easy.  Just multiply percent by 10 to get PWM on-time value.
#define PWM_PERIOD 1000

// Multiply percent by this to get PWM on-time value during ADC calibration.
#define PWM_ON_MULTIPLIER 10U

// Step size when obtaining samples for linear least squares or when generating
// output for plotting.
#define STEP_SIZE 25

// Corresponding step size for linear least squares, using range from
// PERCENT_AT_ENDS to 100-PERCENT_AT_ENDS.
#define NUM_STEPS_LSE ((PWM_ON_MULTIPLIER*(100-2*PERCENT_AT_ENDS))/25 + 1)

// Corresponding step size for plotting, using full range from 0% to 100%.
#define NUM_STEPS_PLOTTED (PWM_PERIOD/25 + 1)

//******************************************************************************
// Read the ADC value at PERCENT_AT_ENDS and 100-PERCENT_AT_ENDS of ref voltage.
//******************************************************************************
static void readADCatEnds(int* ADClow, int* ADChigh, int res, int pinPWM, int pinADC) {
  analogSetPWM_TCC_SAMD_TT(pinPWM, res, PWM_ON_MULTIPLIER*PERCENT_AT_ENDS, PWM_PERIOD);
  delay(PWM_STABLE_DELAY);
  #if PIN_DEBUG != -1
  digitalWrite(PIN_DEBUG, HIGH);
  #endif
  *ADClow = analogRead_SAMD_TT(pinADC);
  analogSetPWM_TCC_SAMD_TT(pinPWM, res,
    PWM_PERIOD-PWM_ON_MULTIPLIER*PERCENT_AT_ENDS,
    PWM_PERIOD);
  delay(PWM_STABLE_DELAY);
  #if PIN_DEBUG != -1
  digitalWrite(PIN_DEBUG, LOW);
  #endif
  *ADChigh = analogRead_SAMD_TT(pinADC);

  #if CALIB_ADC_DBG
  // Show measured values.
  monitor.printf(" At %2d%% of VREF, expected ADC = %5d, actual ADC = %5d\n",
    PERCENT_AT_ENDS, (PERCENT_AT_ENDS*ADC_MAX + 50)/100, *ADClow);
  monitor.printf(" At %2d%% of VREF, expected ADC = %5d, actual ADC = %5d\n",
    100-PERCENT_AT_ENDS, ((100-PERCENT_AT_ENDS)*ADC_MAX + 50)/100, *ADChigh);
  #endif
}

//******************************************************************************
// Read N equally spaced ADC values starting at startPct and ending at EndPct,
// returning results in X (expected ADC values) and Y (actual ADC output values).
//******************************************************************************
static void readADCinRange(char* title, int* X, int* Y, uint16_t N,
  uint8_t startPct, uint8_t endPct,
  int res, int pinPWM, int pinADC, uint8_t cfgADCmultSampAvg) {
  uint16_t startV = PWM_ON_MULTIPLIER*startPct;
  uint16_t endV = PWM_ON_MULTIPLIER*endPct;
  uint16_t range = endV-startV;

  for (uint16_t i = 0; i < N; i++) {
    uint16_t curV = startV + ((range*i*2/(N-1))+1)/2; // Round
    if (curV > endV)
      curV = endV;
    analogSetPWM_TCC_SAMD_TT(pinPWM, res, curV, PWM_PERIOD);
    delay(PWM_STABLE_DELAY);
    int expectedADC = (int)(((2UL*ADC_MAX*curV)/PWM_PERIOD+1)/2);
    X[i] = expectedADC;
    Y[i] = analogRead_SAMD_TT(pinADC);
  }

  #if CALIB_ADC_DBG
  // Show measured values.
  monitor.printf("%s\n", title);
  monitor.printf("%d steps from %d%% to %d%%, start value = %d, end value = %d, period = %d, cfgADCmultSampAvg = %d\n",
    N, startPct, endPct, startV, endV, PWM_PERIOD, cfgADCmultSampAvg);
  monitor.printf("expected (ideal) values:\n");
  for (uint16_t i = 0; i < N; i++)
    monitor.printf("%d\n", X[i]);
  monitor.printf("%s    cfgADCmultSampAvg = %d  actual ADC values:\n", title, cfgADCmultSampAvg);
  for (uint16_t i = 0; i < N; i++)
    monitor.printf("%d\n", Y[i]);
  int32_t e = 0;
  for (uint16_t i = 0; i < N; i++)
    e += Y[i];
  e -= N*(ADC_MAX+1)/2;
  e = (int32_t) ((e + N/2)/N);
  monitor.printf("Deviation from expected mean: %ld\n", e);
  #endif
}

//******************************************************************************
// Calibration function that computes and loads gain and offset error corrections.
//******************************************************************************
void calibSAMD_ADC_withPWM(int pinADC, int pinPWM, int pinAREF_OUT,
  uint8_t cfgADCmultSampAvg) {

  int res; // TCC resolution in bits.
  int ADClow, ADChigh;
  uint16_t gainError;
  int offsetError;
  int X[NUM_STEPS_LSE];
  int Y[NUM_STEPS_LSE];
  int mtx[NUM_STEPS_LSE][NUM_STEPS_LSE]; // About 1000 elements.
  #if CALIB_ADC_DBG
  int plotX[NUM_STEPS_PLOTTED];
  int plotY[NUM_STEPS_PLOTTED];
  #endif

  // Initialize the debug digital output and drive it low.
  #if PIN_DEBUG != -1
  pinMode(PIN_DEBUG, OUTPUT);
  digitalWrite(PIN_DEBUG, LOW);
  #endif

  #if CALIB_ADC_DBG
  // Show the current values of the ADC CALIB, GAINCORR, and OFFSETCORR registers.
  monitor.printf("ADC CALIB = %04X\n", ADC->CALIB.reg);
  monitor.printf("ADC GAINCORR = %04X\n", ADC->GAINCORR.reg);
  monitor.printf("ADC OFFSETCORR = %04X\n", ADC->OFFSETCORR.reg);
  #endif

  if (pinAREF_OUT != -1) {
    // Initialize the digital output that connects to the AREF input and drive it high.
    pinMode(pinAREF_OUT, OUTPUT);
    digitalWrite(pinAREF_OUT, HIGH);
    delay(AREF_STABLE_DELAY);
  }

  // Connect pinADC to the ADC.
  pinPeripheral(pinADC, PIO_ANALOG);

  // Do a quick test of pinPWM as a regular digital output pin. Watch it on a scope.
  #if CALIB_ADC_SCOPE == 1
  pinMode(pinPWM, OUTPUT);
  digitalWrite(pinPWM, LOW);
  digitalWrite(pinPWM, HIGH);
  digitalWrite(pinPWM, LOW);
  #endif

  // Select 16-bit PWM resolution (only need 8, but easiest to stick with 16).
  analogWriteResolution_PWM_SAMD_TT(16);

  // Initialize PWM TCC.
  res = analogStartPWM_TCC_SAMD_TT(pinPWM);

  // Select AREF-A as external voltage reference for the ADC.
  analogReference_SAMD_TT(AR_EXTERNAL);

  // Select 12-bit ADC resolution.
  analogReadResolution_SAMD_TT(12);

  // Load an initial gain and offset error of NO CORRECTION.
  ADC->GAINCORR.reg = ADC_GAINCORR_GAINCORR(GAIN_CORR_1);
  ADC->OFFSETCORR.reg = ADC_OFFSETCORR_OFFSETCORR(0);
  ADC->CTRLB.bit.CORREN = true;

  if (cfgADCmultSampAvg > 0) {
    // Configure multiple sampling and averaging.
    // We set ADJRES according to table 33-3 in the datasheet.
    uint8_t adjRes = (cfgADCmultSampAvg <= 4) ? cfgADCmultSampAvg : 4;
    ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM(cfgADCmultSampAvg) | ADC_AVGCTRL_ADJRES(adjRes);
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT;
  }

  // Test ADC values at 20% of max. Use a scope to watch the PWM waveform.
  #if CALIB_ADC_SCOPE == 2
  analogSetPWM_TCC_SAMD_TT(pinPWM, res, 0, PWM_PERIOD);
  delay(PWM_STABLE_DELAY);
  digitalWrite(PIN_DEBUG, HIGH);
  analogSetPWM_TCC_SAMD_TT(pinPWM, res, PWM_PERIOD/5, PWM_PERIOD);
  delay(PWM_STABLE_DELAY);
  digitalWrite(PIN_DEBUG, LOW);
  #endif

  // Test PWM output rise time.  Look at pwm and cap with scope.
  // It's a bit less than 5 ms from 0 to max voltage.
  #if CALIB_ADC_SCOPE == 3
  analogSetPWM_TCC_SAMD_TT(pinPWM, res, 0, PWM_PERIOD);
  delay(PWM_STABLE_DELAY);
  digitalWrite(PIN_DEBUG, HIGH);
  analogSetPWM_TCC_SAMD_TT(pinPWM, res, PWM_PERIOD, PWM_PERIOD);
  delay(PWM_STABLE_DELAY);
  digitalWrite(PIN_DEBUG, LOW);
  #endif

  // Test PWM output rise time from 0% to PERCENT_AT_ENDS of max.
  // It's a bit less than 3 ms.
  #if CALIB_ADC_SCOPE == 4
  analogSetPWM_TCC_SAMD_TT(pinPWM, res, 0, PWM_PERIOD);
  delay(PWM_STABLE_DELAY);
  digitalWrite(PIN_DEBUG, HIGH);
  analogSetPWM_TCC_SAMD_TT(pinPWM, res, PWM_ON_MULTIPLIER*PERCENT_AT_ENDS, PWM_PERIOD);
  delay(PWM_STABLE_DELAY);
  digitalWrite(PIN_DEBUG, LOW);
  #endif

  // Test PWM output rise time from 100-PERCENT_AT_ENDS to 100% of max.
  // It's a bit less than 2 ms.
  #if CALIB_ADC_SCOPE == 5
  analogSetPWM_TCC_SAMD_TT(pinPWM, res, PWM_PERIOD-PWM_ON_MULTIPLIER*PERCENT_AT_ENDS, PWM_PERIOD);
  delay(PWM_STABLE_DELAY);
  digitalWrite(PIN_DEBUG, HIGH);
  analogSetPWM_TCC_SAMD_TT(pinPWM, res, PWM_PERIOD, PWM_PERIOD);
  delay(PWM_STABLE_DELAY);
  digitalWrite(PIN_DEBUG, LOW);
  #endif

  // Test ADC values at 0, quarter-max, half-max, 3/4-max, and max.
  #if CALIB_ADC_SCOPE == 6
  digitalWrite(PIN_DEBUG, HIGH);
  for (int i = 0; i <= 4; i++) {
    analogSetPWM_TCC_SAMD_TT(pinPWM, res, i*PWM_PERIOD/4, PWM_PERIOD);
    delay(PWM_STABLE_DELAY);
    int ADCin = analogRead_SAMD_TT(pinADC);
    monitor.printf("At PWM = %d%%, ADC = %u\n", 25*i, ADCin);
  }
  digitalWrite(PIN_DEBUG, LOW);
  #endif


  #if CALIB_ADC_DBG
  // Read and show initial non-calibrated results for plotting.
  readADCinRange("Before-calibration results for .xlsx sheet for plotting",
    plotX, plotY, NUM_STEPS_PLOTTED, 0, 100, res, pinPWM, pinADC, cfgADCmultSampAvg);
  #endif

  /*
    Let's try an alternative: least squares estimation using values measured
    across the spectrum, but leaving out PERCENT_AT_ENDS on both ends.
    Assume we are looking for B1 and B2 where each observed ADC value y is
    mapped to a corrected value x = B1 + B2*y, where B1 = offset, B2 = gain.
    We have a vector of observed ADC values Y and a corresponding vector of
    expected (ideal, correct) values X. The parameters B1 and B2 form vector B.
    The formula for finding B from Y is:
        B = inv(Wt W) Wt X
    where W is a 2-column matrix whose second column is Y and whose first column
    is all 1's (see Wikipedia article on linear least squares). Wt is the
    transpose of W, and inv() is the matrix inverse function. The matrix is a
    2x2 matrix, so inversion is easy.
  */

  // Read NUM_STEPS_LSE samples between PERCENT_AT_ENDS and 100-PERCENT_AT_ENDS.
  readADCinRange("Results for linear least squares estimate", X, Y,
    NUM_STEPS_LSE, PERCENT_AT_ENDS, 100-PERCENT_AT_ENDS, res, pinPWM, pinADC,
    cfgADCmultSampAvg);

  // Compute B = [B1, B2] = inv(Wt W) Wt X
  // Use WW = inv(Wt W)
  // Since the first column of W is all 1's and W is a 2-column matrix, WW is a
  // 2x2 matrix, with:
  //    WW[1,1] = NUM_STEPS_LSE
  //    WW[1,2] = sum(Y)
  //    WW[2,1] = sum(Y)
  //    WW[2,2] = sum(Y*Y)
  // The inverse of a 2x2 matrix is computed as follows: Swap the positions of
  // [1,1] and [2,2], negate [1,2] and [2,1], and divide everything by the
  // determinant [1,1][2,2]-[1,2][2,1]
  // Use WWW = inv(WW)
  // Determinant d of WW = NUM_STEPS_LSE*sum(Y*Y) - sum(Y)*sum(Y)
  //    WWW[1,1] = sum(Y*Y)/d
  //    WWW[1,2] = WWW[2,1] = -sum(Y)/d
  //    WWW[2,2] = NUM_STEPS_LSE/d
  // Then, we need V = WWW Wt, a 2xNUM_STEPS_LSE matrix, where column j is:
  //    V[1,i] = WWW[1,1] + WWW[1,2]*Y[i] = sum(Y*Y)/d - Y[i]*sum(Y)/d
  //    V[2,i] = WWW[2,1] + WWW[2,2]*Y[i] = -sum(Y)/d + Y[i]*NUM_STEPS_LSE/d
  // Finally, we must compute B = (V X), which is a 2x1 matrix [B1, B2]:
  //    B1 = sum_over_i((sum(Y*Y) - sum(Y)Y[i])*X[i]/d)
  //    B2 = sum_over_i((-sum(Y) + Y[i]*NUM_STEPS_LSE)*X[i]/d)
  //  B1 = offset, B2 = gain
  float sumY = 0, sumYY = 0;
  for (uint8_t i = 0; i < NUM_STEPS_LSE; i++) {
    sumY += Y[i];
    sumYY += (float)Y[i]*Y[i];
  }
  float d = (NUM_STEPS_LSE*sumYY - sumY*sumY);
  float B1 = 0, B2 = 0;
  for (uint8_t i = 0; i < NUM_STEPS_LSE; i++) {
    B1 += (sumYY - sumY*Y[i])*X[i];
    B2 += (-sumY + NUM_STEPS_LSE*Y[i])*X[i];
  }
  B1 /= d;
  B2 /= d;
  // Multiply by GAIN_CORR_1 to scale it to a 1.11 bit number:
  gainError = (uint16_t) (GAIN_CORR_1*B2);
  // The offset is negated for some reason.
  offsetError = - (int) (B1 + (B1 > 0 ? 0.5 : -0.5));
  #if CALIB_ADC_DBG
  char S[20];
  size_t n = sizeof(S);
  monitor.printf("Offset computed with least squares: %s\n", floatToString(B1, S, n, 3));
  monitor.printf("Gain computed with least squares: %s\n", floatToString(B2, S, n, 3));
  monitor.printf("gainError = %d\n", gainError);
  monitor.printf("offsetError = %d\n", offsetError);
  #endif

#if 0

  // Determine gain error by measuring PERCENT_AT_ENDS and 100-PERCENT_AT_ENDS of
  // reference voltage.
  #if CALIB_ADC_DBG
  monitor.printf("Read ADC to compute gain error\n");
  #endif
  readADCatEnds(&ADClow, &ADChigh, res, pinPWM, pinADC);

  // The actual gain is the slope: ((ADChigh-ADClow)*100)/(ADC_MAX*(100-2*PERCENT_AT_ENDS))
  // The gain error is the inverse: (ADC_MAX*(100-2*PERCENT_AT_ENDS))/((ADChigh-ADClow)*100)
  // Multiply by GAIN_CORR_1 to scale it to a 1.11 bit number:
  //    (GAIN_CORR_1*ADC_MAX*(100-2*PERCENT_AT_ENDS))/((ADChigh-ADClow)*100)
  // I was going to perform rounding by adding half of the divisor before the divide,
  // but it doesn't really seem to improve the result.
  uint32_t divisor1 = (uint32_t)(ADChigh-ADClow)*100;
  gainError = (uint16_t) ((uint32_t)(GAIN_CORR_1*ADC_MAX*(100-2*PERCENT_AT_ENDS))/divisor1);
  #if CALIB_ADC_DBG
  monitor.printf("gainError = %d\n", gainError);
  #endif

  // Load the gain error into the ADC.
  ADC->GAINCORR.reg = ADC_GAINCORR_GAINCORR(gainError);
  ADC->OFFSETCORR.reg = ADC_OFFSETCORR_OFFSETCORR(0);
  ADC->CTRLB.bit.CORREN = true;

  // Determine offset error by again measuring PERCENT_AT_ENDS and 100-PERCENT_AT_ENDS
  // of reference voltage.
  #if CALIB_ADC_DBG
  monitor.printf("Read ADC to compute offset error\n");
  #endif
  readADCatEnds(&ADClow, &ADChigh, res, pinPWM, pinADC);

  // The offset error is the negative of the offset at 0 (y-axis intercept):
  //  - ((PERCENT_AT_ENDS*ADChigh - (100-PERCENT_AT_ENDS)*ADClow)/(100-2*PERCENT_AT_ENDS))
  // I was going to perform rounding by adding half of the divisor before the divide,
  // but it doesn't really seem to improve the result.
  int32_t divisor2 = (100-2*PERCENT_AT_ENDS);
  offsetError = - (int)((((int32_t)PERCENT_AT_ENDS*ADChigh - (int32_t)(100-PERCENT_AT_ENDS)*ADClow))/divisor2);
  #if CALIB_ADC_DBG
  monitor.printf("offsetError = %d\n", offsetError);
  #endif

#endif

  // Load both the gain and offset error into the ADC.
  ADC->GAINCORR.reg = ADC_GAINCORR_GAINCORR(gainError);
  ADC->OFFSETCORR.reg = ADC_OFFSETCORR_OFFSETCORR(offsetError);
  ADC->CTRLB.bit.CORREN = true;

  #if CALIB_ADC_DBG
  // Now see how well we did by again measuring PERCENT_AT_ENDS and 100-PERCENT_AT_ENDS of
  // reference voltage.
  monitor.printf("Read ADC a third time to view results of correction\n");
  readADCatEnds(&ADClow, &ADChigh, res, pinPWM, pinADC);
  #endif

  #if CALIB_ADC_DBG
  // Read and show final results for plotting.
  readADCinRange("After-calibration results for .xlsx sheet for plotting",
    plotX, plotY, NUM_STEPS_PLOTTED, 0, 100, res, pinPWM, pinADC, cfgADCmultSampAvg);
  #endif
}

// -------------------------------------------------------------------------
