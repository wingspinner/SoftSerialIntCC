#ifndef SSSTimerCapture_h
#define SSSTimerCapture_h

/******************************************************************************
* SSSTimerCapture.h
* Simultaneous Multi-instance Full-duplex Software Serial Library for STM32Duino
* Using Timer Capture/Compare Hardware Assist
* 
* Copyright 2015 Ron Curry, InSyte Technologies
* 
* Permission is hereby granted, free of charge, to any person
* obtaining a copy of this software and associated documentation
* files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy,
* modify, merge, publish, distribute, sublicense, and/or sell copies
* of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*****************************************************************************/

/******************************************************************************   
Notes

This file is part of the SoftSerialIntCC Library. The existing STM32Duino/Maple
libraries are missing convenience routines for timer capture. Those needed for
purposes are defined below.
*****************************************************************************/



#include <Arduino.h>
#include <libmaple/libmaple.h>
#include <HardwareTimer.h>
#include <libmaple/timer.h>
//#include <libmaple/stm32.h>


/**
 * Timer input capture modes.
 */
typedef enum timer_ic_mode {
    /**
	*/
    TIMER_IC_MODE_TI1 = TIMER_CCMR_CCS_INPUT_TI1,
    /**
	*/
    TIMER_IC_MODE_TI2 = TIMER_CCMR_CCS_INPUT_TI2,
    /**
	*/	
    TIMER_IC_MODE_TRC = TIMER_CCMR_CCS_INPUT_TRC,
} timer_ic_mode;

typedef enum timer_ic_prescaler {
  TIMER_IC_PRESCALER_NONE = 0 << 2,
  TIMER_IC_PRESCALER_2 = 1 << 2,
  TIMER_IC_PRESCALER_4 = 2 << 2,
  TIMER_IC_PRESCALER_8 = 3 << 2,
} timer_ic_prescaler;

typedef enum timer_ic_filter {
	TIMER_IC_FILTER_NONE = 0 << 4,
	TIMER_IC_FILTER_FCK_INT_N2 = 1 << 4,
	TIMER_IC_FILTER_FCK_INT_N4 = 2 << 4,
	TIMER_IC_FILTER_FCK_INT_N8 = 3 << 4,
	TIMER_IC_FILTER_FDTS_2_N6 = 4 << 4,
	TIMER_IC_FILTER_FDTS_2_N8 = 5 << 4,
	TIMER_IC_FILTER_FDTS_4_N6 = 6 << 4,
	TIMER_IC_FILTER_FDTS_4_N8 = 7 << 4,
	TIMER_IC_FILTER_FDTS_8_N6 = 8 << 4,
	TIMER_IC_FILTER_FDTS_8_N8 = 9 << 4,
	TIMER_IC_FILTER_FDTS_16_N5 = 10 << 4,
	TIMER_IC_FILTER_FDTS_16_N6 = 11 << 4,
	TIMER_IC_FILTER_FDTS_16_N8 = 12 << 4,
	TIMER_IC_FILTER_FDTS_32_N5 = 13 << 4,
	TIMER_IC_FILTER_FDTS_32_N6 = 14 << 4,
	TIMER_IC_FILTER_FDTS_32_N8 = 15 << 4,
} timer_ic_filter;

/**
 * @brief Configure a channel's input capture mode.
 *
 * @param dev Timer device, must have type TIMER_ADVANCED or TIMER_GENERAL.
 * @param channel Channel to configure in input capture mode.
 * @param mode Channel mode to set.
 * @param prescale Input capture prescaler.
 * @param filter Input capture filter.
 * @see timer_ic_mode
 * @see timer_ic_prescaler
 * @see timer_ic_filter
 */
static inline void timer_ic_set_mode(timer_dev *dev,
                                     uint8 channel,
                                     timer_ic_mode mode,
								   timer_ic_prescaler prescale,
								   timer_ic_filter filter) {

    /* channel == 1,2 -> CCMR1; channel == 3,4 -> CCMR2 */
    __io uint32 *ccmr = &(dev->regs).gen->CCMR1 + (((channel - 1) >> 1) & 1);

    /* channel == 1,3 -> shift = 0, channel == 2,4 -> shift = 8 */
    uint8 shift = 8 * (1 - (channel & 1));

    uint32 tmp = *ccmr;
    tmp &= ~(0xFF << shift);
    tmp |= (filter | prescale | mode) << shift;
    *ccmr = tmp;
}


static void disable_channel(timer_dev *dev, uint8 channel) {
    timer_detach_interrupt(dev, channel);
    timer_cc_disable(dev, channel);
}

static void pwm_mode(timer_dev *dev, uint8 channel) {
    timer_disable_irq(dev, channel);
    timer_oc_set_mode(dev, channel, TIMER_OC_MODE_PWM_1, TIMER_OC_PE);
    timer_cc_enable(dev, channel);
}

static void output_compare_mode(timer_dev *dev, uint8 channel) {
    timer_oc_set_mode(dev, channel, TIMER_OC_MODE_ACTIVE_ON_MATCH, 0);
    timer_cc_enable(dev, channel);
}

static void input_capture_mode(timer_dev *dev, uint8 channel) {
    timer_ic_set_mode(dev, channel, TIMER_IC_MODE_TI1, TIMER_IC_PRESCALER_NONE, TIMER_IC_FILTER_NONE);
    timer_cc_enable(dev, channel);
}



#endif


