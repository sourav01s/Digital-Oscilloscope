/*
 * adc.h
 *
 * Pure bare-metal ADC1 driver for STM32L072CZ oscilloscope project.
 * TIM2-triggered, DMA-fed (circular), half/full-buffer double buffering.
 * No CMSIS, no HAL - raw register addresses only.
 *
 * Pipeline:
 *   TIM2 update event  --> triggers ADC1 conversion
 *   ADC1 conversion done --> DMA1 channel1 copies ADC1->DR into adc_buf[]
 *   DMA1 channel1 circular buffer of ADC_BUF_SIZE samples, wraps forever
 *   Half-transfer + transfer-complete interrupts give you two safe
 *   "done" windows per wrap (first half / second half) so you can
 *   process one half while DMA fills the other.
 *
 * Wiring assumed:
 *   PA0 = ADC1_IN0 (analog input, e.g. signal or potentiometer wiper)
 */

#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include <stdint.h>

#define ADC_BUF_SIZE     256          /* total circular buffer length      */
#define ADC_HALF_SIZE    (ADC_BUF_SIZE / 2)
#define ADC_FULLSCALE    4095U        /* 12-bit resolution                 */

/*
 * Initialize GPIO (PA0 analog), ADC1, TIM2, and DMA1 channel1.
 * Does NOT start sampling - call ADC_Start() after this.
 *
 * sample_rate_hz: desired conversion rate, paced by TIM2. Practical range
 * depends on your ADC sampling time setting and system clock - keep it
 * well under a few hundred kHz for a 16MHz-clocked HSI16 async ADC clock.
 */
void ADC_Init(uint32_t sample_rate_hz);

/* Arms TIM2 + starts ADC hardware-triggered conversions. Runs forever
   until ADC_Stop() is called. */
void ADC_Start(void);

/* Stops TIM2 and any in-progress ADC conversion. */
void ADC_Stop(void);

/*
 * Returns 1 exactly once per wrap when the FIRST half of adc_buf[]
 * (indices 0 .. ADC_HALF_SIZE-1) has just been completely written by DMA
 * and is safe to read. Auto-clears after being read.
 */
uint8_t ADC_FirstHalfReady(void);

/*
 * Returns 1 exactly once per wrap when the SECOND half of adc_buf[]
 * (indices ADC_HALF_SIZE .. ADC_BUF_SIZE-1) has just been completely
 * written by DMA and is safe to read. Auto-clears after being read.
 */
uint8_t ADC_SecondHalfReady(void);

/* Returns a pointer to the raw circular buffer. Only read the half that
   the corresponding ADC_FirstHalfReady()/ADC_SecondHalfReady() indicated
   is safe - the other half may be actively being overwritten by DMA. */
volatile uint16_t *ADC_GetBuffer(void);

/*
 * DMA1 Channel1 IRQ handler - this driver defines it directly (channel1
 * on STM32L0 has its own dedicated IRQ, not a shared one), so make sure
 * your startup file's vector table entry for DMA1_Channel1_IRQHandler
 * is either left as the default weak symbol (so this definition links
 * against it) or points here explicitly.
 */
void DMA1_Channel1_IRQHandler(void);

/* Convert a raw 12-bit ADC code to millivolts given a reference voltage
   in millivolts (e.g. 3300 for a 3.3V Vref). */
static inline uint32_t ADC_RawToMillivolts(uint16_t raw, uint32_t vref_mv)
{
    return ((uint32_t)raw * vref_mv) / ADC_FULLSCALE;
}

#endif /* ADC_DRIVER_H */
