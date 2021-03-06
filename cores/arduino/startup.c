/*
  Copyright (c) 2017 Scott Price. All right reserved.
  Copyright (c) 2017 MattairTech LLC. All right reserved.
  Copyright (c) 2015 Arduino LLC.  All right reserved.

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

#include "sam.h"
#include "variant.h"

#include <stdio.h>

/**
 * \brief SystemInit() configures the needed clocks and according Flash Read Wait States.
 * At reset:
 * - OSC8M clock source is enabled with a divider by 8 (1MHz).
 * - Generic Clock Generator 0 (GCLKMAIN) is using OSC8M as source.
 * We need to:
 * 1) Enable XOSC32K clock (External on-board 32.768Hz oscillator), will be used as DFLL48M reference.
 * 2) Put XOSC32K as source of Generic Clock Generator 1
 * 3) Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 0 (DFLL48M reference)
 * 4) Enable DFLL48M clock
 * 5) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
 * 6) Modify PRESCaler value of OSCM to have 8MHz
 * 7) Put OSC8M as source for Generic Clock Generator 3
 */
// Constants for Clock generators
#define GENERIC_CLOCK_GENERATOR_MAIN      (0u)
#define GENERIC_CLOCK_GENERATOR_XOSC32K   (1u)
#define GENERIC_CLOCK_GENERATOR_OSC32K    (1u)
#define GENERIC_CLOCK_GENERATOR_OSCULP32K (2u) /* Initialized at reset for WDT */
#define GENERIC_CLOCK_GENERATOR_OSC8M     (3u)
// Constants for Clock multiplexers
#define GENERIC_CLOCK_MULTIPLEXER_DFLL48M (0u)

void SystemInit( void )
{
#if SAMC_SERIES

    
  /* Set 2 Flash Wait States for the C21, cf table 45-34 in SAMC21 Datasheet */
  NVMCTRL->CTRLB.reg |= NVMCTRL_CTRLB_RWS_DUAL ; // two wait states

  /* Turn on the digital interface clock */
  MCLK->APBAMASK.reg |= MCLK_APBAMASK_GCLK ;


  /* ----------------------------------------------------------------------------------------------
   * Software reset the GCLK module to ensure it is re-initialized correctly
   */
  GCLK->CTRLA.reg = GCLK_CTRLA_SWRST ;

  while ( (GCLK->CTRLA.reg & GCLK_CTRLA_SWRST) && (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK) );  /* Wait for reset to complete */


#if defined(CLOCKCONFIG_32768HZ_CRYSTAL)
  /* ----------------------------------------------------------------------------------------------
   * Enable XOSC32K clock (External on-board 32.768Hz crystal oscillator)
   */

#if defined(PLL_FRACTIONAL_ENABLE)
  #define DPLLRATIO_LDR         2928u
  #define DPLLRATIO_LDRFRAC     11u
#else
  #define DPLLRATIO_LDR         2929u
  #define DPLLRATIO_LDRFRAC     0u
#endif

  
  OSC32KCTRL->XOSC32K.reg = (OSC32KCTRL_XOSC32K_STARTUP( 0x4u ) | OSC32KCTRL_XOSC32K_XTALEN | OSC32KCTRL_XOSC32K_EN32K | OSC32KCTRL_XOSC32K_EN1K) ;
  OSC32KCTRL->XOSC32K.bit.ENABLE = 1 ;
  
  while ( (OSC32KCTRL->STATUS.reg & OSC32KCTRL_STATUS_XOSC32KRDY) == 0 );       /* Wait for oscillator stabilization */
  
  OSCCTRL->DPLLRATIO.reg = ( OSCCTRL_DPLLRATIO_LDR(DPLLRATIO_LDR) | OSCCTRL_DPLLRATIO_LDRFRAC(DPLLRATIO_LDRFRAC) ) ;  /* set PLL multiplier */
  while ( OSCCTRL->DPLLSYNCBUSY.reg & OSCCTRL_DPLLSYNCBUSY_MASK );
  
  OSCCTRL->DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_REFCLK(0) ;  /* select 32KHz crystal input */
  
  OSCCTRL->DPLLPRESC.reg = 0;
  while ( OSCCTRL->DPLLSYNCBUSY.reg & OSCCTRL_DPLLSYNCBUSY_MASK );
  
  OSCCTRL->DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE ;
  while ( OSCCTRL->DPLLSYNCBUSY.reg & OSCCTRL_DPLLSYNCBUSY_MASK );
  
  while ( (OSCCTRL->DPLLSTATUS.reg & OSCCTRL_DPLLSTATUS_CLKRDY) != OSCCTRL_DPLLSTATUS_CLKRDY );
  
  /* Switch Generic Clock Generator 0 to PLL. Divide by two and the CPU will run at 48MHz. */
  GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_MAIN].reg = ( GCLK_GENCTRL_DIV(2) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
  while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK );

#elif defined(CLOCKCONFIG_HS_CRYSTAL)
  /* ----------------------------------------------------------------------------------------------
   * Enable XOSC clock (External on-board high speed crystal oscillator)
   */

#if ((HS_CRYSTAL_FREQUENCY_HERTZ < 400000UL) || (HS_CRYSTAL_FREQUENCY_HERTZ > 32000000UL))
  #error "board.init.c: HS_CRYSTAL_FREQUENCY_HERTZ must be between 400000UL and 32000000UL"
#endif

#if defined(PLL_FAST_STARTUP)
  #if (HS_CRYSTAL_FREQUENCY_HERTZ < 1000000UL)
    #error "board.init.c: HS_CRYSTAL_FREQUENCY_HERTZ must be at least 1000000UL when PLL_FAST_STARTUP is defined"
  #else
    #define HS_CRYSTAL_DIVISOR  1000000UL
  #endif
#else
  #define HS_CRYSTAL_DIVISOR    32000UL
#endif

#define HS_CRYSTAL_DIVIDER      (HS_CRYSTAL_FREQUENCY_HERTZ / HS_CRYSTAL_DIVISOR)
#define DPLLRATIO_FLOAT         (96000000.0 / ((float)HS_CRYSTAL_FREQUENCY_HERTZ / HS_CRYSTAL_DIVIDER))

#if defined(PLL_FRACTIONAL_ENABLED)
  #define DPLLRATIO_LDR         (uint16_t)DPLLRATIO_FLOAT
  #define DPLLRATIO_LDRFRAC     (uint8_t)((DPLLRATIO_FLOAT - (uint16_t)DPLLRATIO_FLOAT) * 16.0)
#else
  #define DPLLRATIO_LDR         (uint16_t)DPLLRATIO_FLOAT
  #define DPLLRATIO_LDRFRAC     0
#endif

  OSCCTRL->XOSCCTRL.reg = (OSCCTRL_XOSCCTRL_STARTUP( 0x8u ) | OSCCTRL_XOSCCTRL_GAIN( 0x4u ) | OSCCTRL_XOSCCTRL_XTALEN | OSCCTRL_XOSCCTRL_ENABLE) ; // startup time is 8ms
  while ( (OSCCTRL->STATUS.reg & OSCCTRL_STATUS_XOSCRDY) == 0 );        /* Wait for oscillator stabilization */

  OSCCTRL->XOSCCTRL.reg |= OSCCTRL_XOSCCTRL_AMPGC ;     // set only after startup time
  
  /* Connect GCLK1 to XOSC and set prescaler */
  GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_XOSC].reg = ( GCLK_GENCTRL_DIV(HS_CRYSTAL_DIVIDER) | GCLK_GENCTRL_SRC_XOSC | GCLK_GENCTRL_GENEN );
  while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK );
  
  /* Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 1 (FDPLL reference) */
  GCLK->PCHCTRL[GENERIC_CLOCK_MULTIPLEXER_FDPLL].reg = ( GCLK_PCHCTRL_CHEN | GCLK_PCHCTRL_GEN_GCLK1 );
  while ( (GCLK->PCHCTRL[GENERIC_CLOCK_MULTIPLEXER_FDPLL].reg & GCLK_PCHCTRL_CHEN) != GCLK_PCHCTRL_CHEN );      // wait for sync
  
  /* Configure PLL */
  OSCCTRL->DPLLRATIO.reg = ( OSCCTRL_DPLLRATIO_LDR(DPLLRATIO_LDR) | OSCCTRL_DPLLRATIO_LDRFRAC(DPLLRATIO_LDRFRAC) ) ;  /* set PLL multiplier */
  while ( OSCCTRL->DPLLSYNCBUSY.reg & OSCCTRL_DPLLSYNCBUSY_MASK );
  
  OSCCTRL->DPLLCTRLB.reg = OSCCTRL_DPLLCTRLB_REFCLK(2) ;  /* select GCLK input */
  
  OSCCTRL->DPLLPRESC.reg = 0;
  while ( OSCCTRL->DPLLSYNCBUSY.reg & OSCCTRL_DPLLSYNCBUSY_MASK );
  
  OSCCTRL->DPLLCTRLA.reg = OSCCTRL_DPLLCTRLA_ENABLE ;
  while ( OSCCTRL->DPLLSYNCBUSY.reg & OSCCTRL_DPLLSYNCBUSY_MASK );
  
  while ( (OSCCTRL->DPLLSTATUS.reg & OSCCTRL_DPLLSTATUS_CLKRDY) != OSCCTRL_DPLLSTATUS_CLKRDY );
  
  /* Switch Generic Clock Generator 0 to PLL. Divide by two and the CPU will run at 48MHz. */
  GCLK->GENCTRL[GENERIC_CLOCK_GENERATOR_MAIN].reg = ( GCLK_GENCTRL_DIV(2) | GCLK_GENCTRL_SRC_DPLL96M | GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN );
  while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK );

#else // Internal Clock
  /* Change OSC48M divider to /1. CPU will run at 48MHz */
  OSCCTRL->OSC48MDIV.reg = OSCCTRL_OSC48MDIV_DIV(0);
  while ( OSCCTRL->OSC48MSYNCBUSY.reg & OSCCTRL_OSC48MSYNCBUSY_OSC48MDIV );

#endif


  SystemCoreClock=VARIANT_MCK ;

  MCLK->CPUDIV.reg  = MCLK_CPUDIV_CPUDIV_DIV1 ;

  /*
   * Disable automatic NVM write operations (errata reference 13134, applies to D21/D11/L21, but not C21)
   */
  NVMCTRL->CTRLB.bit.MANW = 1;



#else // !SAMC_SERIES
  /* Set 1 Flash Wait State for 48MHz, cf tables 20.9 and 35.27 in SAMD21 Datasheet */
  NVMCTRL->CTRLB.bit.RWS = NVMCTRL_CTRLB_RWS_HALF_Val ;

  /* Turn on the digital interface clock */
  PM->APBAMASK.reg |= PM_APBAMASK_GCLK ;


#if defined(CRYSTALLESS)

  /* ----------------------------------------------------------------------------------------------
   * 1) Enable OSC32K clock (Internal 32.768Hz oscillator)
   */

  uint32_t calib = (*((uint32_t *) FUSES_OSC32K_CAL_ADDR) & FUSES_OSC32K_CAL_Msk) >> FUSES_OSC32K_CAL_Pos;

  SYSCTRL->OSC32K.reg = SYSCTRL_OSC32K_CALIB(calib) |
                        SYSCTRL_OSC32K_STARTUP( 0x6u ) | // cf table 15.10 of product datasheet in chapter 15.8.6
                        SYSCTRL_OSC32K_EN32K |
                        SYSCTRL_OSC32K_ENABLE;

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_OSC32KRDY) == 0 ); // Wait for oscillator stabilization

#else // has crystal

  /* ----------------------------------------------------------------------------------------------
   * 1) Enable XOSC32K clock (External on-board 32.768Hz oscillator)
   */
  SYSCTRL->XOSC32K.reg = SYSCTRL_XOSC32K_STARTUP( 0x6u ) | /* cf table 15.10 of product datasheet in chapter 15.8.6 */
                         SYSCTRL_XOSC32K_XTALEN | SYSCTRL_XOSC32K_EN32K ;
  SYSCTRL->XOSC32K.bit.ENABLE = 1 ; /* separate call, as described in chapter 15.6.3 */

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_XOSC32KRDY) == 0 )
  {
    /* Wait for oscillator stabilization */
  }

#endif

  /* Software reset the module to ensure it is re-initialized correctly */
  /* Note: Due to synchronization, there is a delay from writing CTRL.SWRST until the reset is complete.
   * CTRL.SWRST and STATUS.SYNCBUSY will both be cleared when the reset is complete, as described in chapter 13.8.1
   */
  GCLK->CTRL.reg = GCLK_CTRL_SWRST ;

  while ( (GCLK->CTRL.reg & GCLK_CTRL_SWRST) && (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) )
  {
    /* Wait for reset to complete */
  }

  /* ----------------------------------------------------------------------------------------------
   * 2) Put XOSC32K as source of Generic Clock Generator 1
   */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_XOSC32K ) ; // Generic Clock Generator 1

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* Write Generic Clock Generator 1 configuration */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_OSC32K ) | // Generic Clock Generator 1
#if defined(CRYSTALLESS)
                      GCLK_GENCTRL_SRC_OSC32K | // Selected source is Internal 32KHz Oscillator
#else
                      GCLK_GENCTRL_SRC_XOSC32K | // Selected source is External 32KHz Oscillator
#endif
//                      GCLK_GENCTRL_OE | // Output clock to a pin for tests
                      GCLK_GENCTRL_GENEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* ----------------------------------------------------------------------------------------------
   * 3) Put Generic Clock Generator 1 as source for Generic Clock Multiplexer 0 (DFLL48M reference)
   */
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GENERIC_CLOCK_MULTIPLEXER_DFLL48M ) | // Generic Clock Multiplexer 0
                      GCLK_CLKCTRL_GEN_GCLK1 | // Generic Clock Generator 1 is source
                      GCLK_CLKCTRL_CLKEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* ----------------------------------------------------------------------------------------------
   * 4) Enable DFLL48M clock
   */

  /* DFLL Configuration in Closed Loop mode, cf product datasheet chapter 15.6.7.1 - Closed-Loop Operation */

  /* Remove the OnDemand mode, Bug http://avr32.icgroup.norway.atmel.com/bugzilla/show_bug.cgi?id=9905 */
  SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
  {
    /* Wait for synchronization */
  }

  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP( 31 ) | // Coarse step is 31, half of the max value
                         SYSCTRL_DFLLMUL_FSTEP( 511 ) | // Fine step is 511, half of the max value
                         SYSCTRL_DFLLMUL_MUL( (VARIANT_MCK + VARIANT_MAINOSC/2) / VARIANT_MAINOSC ) ; // External 32KHz is the reference

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
  {
    /* Wait for synchronization */
  }

#if defined(CRYSTALLESS)

  #define NVM_SW_CALIB_DFLL48M_COARSE_VAL 58

  // Turn on DFLL
  uint32_t coarse =( *((uint32_t *)(NVMCTRL_OTP4) + (NVM_SW_CALIB_DFLL48M_COARSE_VAL / 32)) >> (NVM_SW_CALIB_DFLL48M_COARSE_VAL % 32) )
                   & ((1 << 6) - 1);
  if (coarse == 0x3f) {
    coarse = 0x1f;
  }
  // TODO(tannewt): Load this value from memory we've written previously. There
  // isn't a value from the Atmel factory.
  uint32_t fine = 0x1ff;

  SYSCTRL->DFLLVAL.bit.COARSE = coarse;
  SYSCTRL->DFLLVAL.bit.FINE = fine;
  /* Write full configuration to DFLL control register */
  SYSCTRL->DFLLMUL.reg = SYSCTRL_DFLLMUL_CSTEP( 0x1f / 4 ) | // Coarse step is 31, half of the max value
                         SYSCTRL_DFLLMUL_FSTEP( 10 ) |
                         SYSCTRL_DFLLMUL_MUL( (48000) ) ;

  SYSCTRL->DFLLCTRL.reg = 0;

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
  {
    /* Wait for synchronization */
  }

  SYSCTRL->DFLLCTRL.reg =  SYSCTRL_DFLLCTRL_MODE |
                           SYSCTRL_DFLLCTRL_CCDIS |
                           SYSCTRL_DFLLCTRL_USBCRM | /* USB correction */
                           SYSCTRL_DFLLCTRL_BPLCKC;

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
  {
    /* Wait for synchronization */
  }

  /* Enable the DFLL */
  SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_ENABLE ;

#else   // has crystal

  /* Write full configuration to DFLL control register */
  SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_MODE | /* Enable the closed loop mode */
                           SYSCTRL_DFLLCTRL_WAITLOCK |
                           SYSCTRL_DFLLCTRL_QLDIS ; /* Disable Quick lock */

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
  {
    /* Wait for synchronization */
  }

  /* Enable the DFLL */
  SYSCTRL->DFLLCTRL.reg |= SYSCTRL_DFLLCTRL_ENABLE ;

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKC) == 0 ||
          (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLLCKF) == 0 )
  {
    /* Wait for locks flags */
  }

#endif

  while ( (SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY) == 0 )
  {
    /* Wait for synchronization */
  }

  /* ----------------------------------------------------------------------------------------------
   * 5) Switch Generic Clock Generator 0 to DFLL48M. CPU will run at 48MHz.
   */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_MAIN ) ; // Generic Clock Generator 0

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* Write Generic Clock Generator 0 configuration */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_MAIN ) | // Generic Clock Generator 0
                      GCLK_GENCTRL_SRC_DFLL48M | // Selected source is DFLL 48MHz
//                      GCLK_GENCTRL_OE | // Output clock to a pin for tests
                      GCLK_GENCTRL_IDC | // Set 50/50 duty cycle
                      GCLK_GENCTRL_GENEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /* ----------------------------------------------------------------------------------------------
   * 6) Modify PRESCaler value of OSC8M to have 8MHz
   */
  SYSCTRL->OSC8M.bit.PRESC = SYSCTRL_OSC8M_PRESC_0_Val ;  //CMSIS 4.5 changed the prescaler defines
  SYSCTRL->OSC8M.bit.ONDEMAND = 0 ;

  /* ----------------------------------------------------------------------------------------------
   * 7) Put OSC8M as source for Generic Clock Generator 3
   */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID( GENERIC_CLOCK_GENERATOR_OSC8M ) ; // Generic Clock Generator 3

  /* Write Generic Clock Generator 3 configuration */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID( GENERIC_CLOCK_GENERATOR_OSC8M ) | // Generic Clock Generator 3
                      GCLK_GENCTRL_SRC_OSC8M | // Selected source is RC OSC 8MHz (already enabled at reset)
//                      GCLK_GENCTRL_OE | // Output clock to a pin for tests
                      GCLK_GENCTRL_GENEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  /*
   * Now that all system clocks are configured, we can set CPU and APBx BUS clocks.
   * There values are normally the one present after Reset.
   */
  PM->CPUSEL.reg  = PM_CPUSEL_CPUDIV_DIV1 ;
  PM->APBASEL.reg = PM_APBASEL_APBADIV_DIV1_Val ;
  PM->APBBSEL.reg = PM_APBBSEL_APBBDIV_DIV1_Val ;
  PM->APBCSEL.reg = PM_APBCSEL_APBCDIV_DIV1_Val ;

  SystemCoreClock=VARIANT_MCK ;

  /* ----------------------------------------------------------------------------------------------
   * 8) Load ADC factory calibration values
   */

  // ADC Bias Calibration
  uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;

  // ADC Linearity bits 4:0
  uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;

  // ADC Linearity bits 7:5
  linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;

  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);

  /*
   * 9) Disable automatic NVM write operations
   */
  NVMCTRL->CTRLB.bit.MANW = 1;
#endif
}
