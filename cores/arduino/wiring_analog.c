/*
  Copyright (c) 2017 MattairTech LLC. All right reserved.
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

#include "Arduino.h"
#include "wiring_private.h"

#ifdef __cplusplus
extern "C" {
#endif

static int _readResolution = 10;
static int _ADCResolution = 10;
static int _writeResolution = 8;

// Wait for synchronization of registers between the clock domains
static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
#if SAMC_SERIES
  while ( ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );
  while ( ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );
#else
  while (ADC->STATUS.bit.SYNCBUSY == 1);
#endif

}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncDAC() __attribute__((always_inline, unused));
static void syncDAC() {
#if SAMC_SERIES
  while ( DAC->SYNCBUSY.reg & DAC_SYNCBUSY_MASK );
#else
  while (DAC->STATUS.bit.SYNCBUSY == 1);
#endif
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTC_16(Tc* TCx) __attribute__((always_inline, unused));
static void syncTC_16(Tc* TCx) {
#if SAMC_SERIES
  while (TCx->COUNT16.SYNCBUSY.reg & (TC_SYNCBUSY_SWRST | TC_SYNCBUSY_ENABLE | TC_SYNCBUSY_CTRLB | TC_SYNCBUSY_STATUS | TC_SYNCBUSY_COUNT));
#else
  while (TCx->COUNT16.STATUS.bit.SYNCBUSY);
#endif
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}

void analogReadResolution(int res)
{
  _readResolution = res;
  if (res > 10) {
#if SAMC_SERIES
    ADC0->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_12BIT_Val;
    ADC1->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_12BIT_Val;
#else
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_12BIT_Val;
#endif
    _ADCResolution = 12;
  } else if (res > 8) {
#if SAMC_SERIES
    ADC0->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_10BIT_Val;
    ADC1->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_10BIT_Val;
#else
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;
#endif
    _ADCResolution = 10;
  } else {
#if SAMC_SERIES
    ADC0->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_8BIT_Val;
    ADC1->CTRLC.bit.RESSEL = ADC_CTRLC_RESSEL_8BIT_Val;
#else
    ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_8BIT_Val;
#endif
    _ADCResolution = 8;
  }
  syncADC();
}

void analogWriteResolution(int res)
{
  _writeResolution = res;
}

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

/*
 * Internal Reference is at 1.0v
 * External Reference should be between 1v and VDDANA-0.6v=2.7v
 *
 * Warning : On Arduino Zero board the input/output voltage for SAMD21G18 is 3.3 volts maximum
 */
void analogReference(eAnalogReference mode)
{
  syncADC();
#if SAMC_SERIES
  if (mode == 0) {              // Set to 1.0V for the SAML, 1.024V for the SAMC
    SUPC->VREF.reg &= ~SUPC_VREF_SEL_Msk;
  } else if (mode > 5) {                // Values above 5 are used for the Supply Controller reference (AR_INTREF)
    SUPC->VREF.reg &= ~SUPC_VREF_SEL_Msk;
    SUPC->VREF.reg |= SUPC_VREF_SEL(mode - 6);  // 
    mode = 0;
  }
  ADC0->REFCTRL.bit.REFSEL = mode;
  ADC1->REFCTRL.bit.REFSEL = mode;
#else
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

#endif
}

uint32_t analogRead(uint32_t pin)
{
  uint32_t valueRead = 0;

  if (pin < A0) {
    pin += A0;
  }

#if SAMC_SERIES
  Adc* ADC;
  if ( g_APinDescription[pin].ulPinType == PIO_ANALOG_ALT ) {
    ADC = ADC1;
  } else {
    ADC = ADC0;
  }
#endif

  pinPeripheral(pin, PIO_ANALOG);

  // Disable DAC, if analogWrite() was used previously to enable the DAC
  if ((g_APinDescription[pin].ulADCChannelNumber == ADC_Channel0) || (g_APinDescription[pin].ulADCChannelNumber == DAC_Channel0)) {
    syncDAC();
    DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC
    //DAC->CTRLB.bit.EOEN = 0x00; // The DAC output is turned off.
    syncDAC();
  }

  while ( ADC->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );
  //syncADC();
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
  while ( ADC->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );
  //syncADC();
  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC

  // Start conversion
  while ( ADC->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );
  //syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Waiting for the 1st conversion to complete
  while (ADC->INTFLAG.bit.RESRDY == 0);

  // Clear the Data Ready flag
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  // Start conversion again, since The first conversion after the reference is changed must not be used.
  while ( ADC->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );
  //syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Store the value
  while (ADC->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
  valueRead = ADC->RESULT.reg;

  while ( ADC->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );
  //syncADC();
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  while ( ADC->SYNCBUSY.reg & ADC_SYNCBUSY_MASK );
  //syncADC();

  return mapResolution(valueRead, _ADCResolution, _readResolution);
}


// Right now, PWM output only works on the pins with
// hardware support.  These are defined in the appropriate
// pins_*.c file.  For the rest of the pins, we default
// to digital output.
void analogWrite(uint32_t pin, uint32_t value)
{
  PinDescription pinDesc = g_APinDescription[pin];
  uint32_t attr = pinDesc.ulPinAttribute;

  if ((attr & PIN_ATTR_ANALOG) == PIN_ATTR_ANALOG)
  {
    // DAC handling code

    if ((pinDesc.ulADCChannelNumber != ADC_Channel0) && (pinDesc.ulADCChannelNumber != DAC_Channel0)) { // Only 1 DAC on AIN0 / PA02
      return;
    }

    value = mapResolution(value, _writeResolution, 10);

    syncDAC();
    DAC->DATA.reg = value & 0x3FF;  // DAC on 10 bits.
    syncDAC();
    DAC->CTRLA.bit.ENABLE = 0x01;     // Enable DAC
    syncDAC();
    return;
  }

  if ((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM)
  {
    value = mapResolution(value, _writeResolution, 16);

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

      uint16_t GCLK_CLKCTRL_IDs[] = {
#if SAMC_SERIES
        GCM_TCC0_TCC1,
        GCM_TCC0_TCC1,
        GCM_TCC2,
        GCM_TC0_TC1,
        GCM_TC0_TC1,
        GCM_TC2_TC3,
        GCM_TC2_TC3,
        GCM_TC4,
#else
        GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC0
        GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC1
        GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TCC2
        GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TC3
        GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC4
        GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC5
        GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC6
        GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC7
#endif
      };
#if SAMC_SERIES
      GCLK->PCHCTRL[GCLK_CLKCTRL_IDs[tcNum]].reg = (GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK0 );
      while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK );
#else
      GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_IDs[tcNum]);
      while (GCLK->STATUS.bit.SYNCBUSY == 1);
#endif

      // Set PORT
      if (tcNum >= TCC_INST_NUM) {
        // -- Configure TC
        Tc* TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);
        // Disable TCx
        TCx->COUNT16.CTRLA.bit.ENABLE = 0;
        syncTC_16(TCx);
        // Set Timer counter Mode to 16 bits, normal PWM
        TCx->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
        syncTC_16(TCx);
#if SAMC_SERIES
        TCx->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_NPWM;
#else
        TCx->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_NPWM;
#endif

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
#if SAMC_SERIES
        TCx->COUNT16.CCBUF[tcChannel].reg = (uint32_t) value;
#else
        TCx->COUNT16.CC[tcChannel].reg = (uint32_t) value;
#endif
        syncTC_16(TCx);
      } else {
#if SAMC_SERIES
// LUPD caused endless spinning in syncTCC() on SAML (and probably SAMC). Note that CCBUF writes are already
// atomic. The LUPD bit is intended for updating several registers at once, which analogWrite() does not do.
        // -- Configure TCC
        Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
        TCCx->CCBUF[tcChannel].reg = (uint32_t) value;
#else
        Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
        TCCx->CTRLBSET.bit.LUPD = 1;
        syncTCC(TCCx);
        TCCx->CCB[tcChannel].reg = (uint32_t) value;
        syncTCC(TCCx);
        TCCx->CTRLBCLR.bit.LUPD = 1;
        syncTCC(TCCx);
#endif
      }
    }
    return;
  }

  // -- Defaults to digital write
  pinMode(pin, OUTPUT);
  value = mapResolution(value, _writeResolution, 8);
  if (value < 128) {
    digitalWrite(pin, LOW);
  } else {
    digitalWrite(pin, HIGH);
  }
}

#ifdef __cplusplus
}
#endif
