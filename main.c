#include <stdint.h>
#include "adc.h"
#include "spi.h"

/* ---------------- Raw RCC registers (HSI16 clock bring-up) ---------------- */
#define RCC_BASE        0x40021000UL
#define RCC_CR          (*(volatile uint32_t *)(RCC_BASE + 0x00UL))
#define RCC_CFGR        (*(volatile uint32_t *)(RCC_BASE + 0x0CUL))

#define RCC_CR_HSION        (1UL << 0)
#define RCC_CR_HSIRDY       (1UL << 2)
#define RCC_CFGR_SW_MASK    (3UL << 0)
#define RCC_CFGR_SW_HSI     (1UL << 0)
#define RCC_CFGR_SWS_MASK   (3UL << 2)
#define RCC_CFGR_SWS_HSI    (1UL << 2)

static void clock_init(void)       // setting HSI clock
{
    RCC_CR |= RCC_CR_HSION;
    while (!(RCC_CR & RCC_CR_HSIRDY));

    RCC_CFGR &= ~RCC_CFGR_SW_MASK;
    RCC_CFGR |= RCC_CFGR_SW_HSI;
    while ((RCC_CFGR & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_HSI);    // using HSI CLOCK
}
static uint32_t frames_sent = 0;
static uint32_t frames_dropped = 0;

int main(void)
{
    clock_init();

    ADC_Init(10000UL);  // currently using 10khz adc freq
    SPI_Init();

    ADC_Start();

    volatile uint16_t *buf = ADC_GetBuffer();

    while (1)
    {
        (void)SPI_TxBusy();

        if (ADC_FirstHalfReady())
        {
            if (SPI_SendWaveform(&buf[0], ADC_HALF_SIZE))
                frames_sent++;
            else
                frames_dropped++;
        }

        if (ADC_SecondHalfReady())
        {
            if (SPI_SendWaveform(&buf[ADC_HALF_SIZE], ADC_HALF_SIZE))
                frames_sent++;
            else
                frames_dropped++;
        }
    }
}
