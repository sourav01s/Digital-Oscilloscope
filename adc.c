#include "adc.h"

/* ---- RCC ---- */
#define RCC_BASE            0x40021000UL
#define RCC_IOPENR          (*(volatile uint32_t *)(RCC_BASE + 0x2CUL))
#define RCC_AHBENR          (*(volatile uint32_t *)(RCC_BASE + 0x30UL))
#define RCC_APB2ENR         (*(volatile uint32_t *)(RCC_BASE + 0x34UL))
#define RCC_APB1ENR         (*(volatile uint32_t *)(RCC_BASE + 0x38UL))

#define RCC_IOPENR_GPIOAEN  (1UL << 0)
#define RCC_AHBENR_DMA1EN   (1UL << 0)
#define RCC_APB2ENR_ADC1EN  (1UL << 9)
#define RCC_APB1ENR_TIM2EN  (1UL << 0)

/* ---- GPIOA ---- */
#define GPIOA_BASE          0x50000000UL
#define GPIOA_MODER         (*(volatile uint32_t *)(GPIOA_BASE + 0x00UL))

/* ---- ADC1 ---- */
#define ADC1_BASE           0x40012400UL
#define ADC1_ISR            (*(volatile uint32_t *)(ADC1_BASE + 0x00UL))
#define ADC1_IER            (*(volatile uint32_t *)(ADC1_BASE + 0x04UL))
#define ADC1_CR             (*(volatile uint32_t *)(ADC1_BASE + 0x08UL))
#define ADC1_CFGR1          (*(volatile uint32_t *)(ADC1_BASE + 0x0CUL))
#define ADC1_CFGR2          (*(volatile uint32_t *)(ADC1_BASE + 0x10UL))
#define ADC1_SMPR           (*(volatile uint32_t *)(ADC1_BASE + 0x14UL))
#define ADC1_CHSELR         (*(volatile uint32_t *)(ADC1_BASE + 0x28UL))
#define ADC1_DR             (*(volatile uint32_t *)(ADC1_BASE + 0x40UL))

#define ADC_ISR_ADRDY       (1UL << 0)
#define ADC_ISR_EOC         (1UL << 2)

#define ADC_CR_ADEN         (1UL << 0)
#define ADC_CR_ADDIS        (1UL << 1)
#define ADC_CR_ADSTART      (1UL << 2)
#define ADC_CR_ADSTP        (1UL << 4)
#define ADC_CR_ADCAL        (1UL << 31)

#define ADC_CFGR1_DMAEN     (1UL << 0)
#define ADC_CFGR1_DMACFG    (1UL << 1)
#define ADC_CFGR1_EXTSEL_SHIFT  6
#define ADC_CFGR1_EXTSEL_MASK   (0x7UL << ADC_CFGR1_EXTSEL_SHIFT)
#define ADC_CFGR1_EXTSEL_TIM2_TRGO  (2UL << ADC_CFGR1_EXTSEL_SHIFT)
#define ADC_CFGR1_EXTEN_SHIFT   10
#define ADC_CFGR1_EXTEN_RISING  (1UL << ADC_CFGR1_EXTEN_SHIFT)

#define ADC_CFGR2_CKMODE_MASK   (3UL << 30) /* 00 = HSI16 async clock */

#define ADC_CHSELR_CHSEL0   (1UL << 0)

#define ADC_SMPR_SMP_39_5CYC   (4UL << 0)

/* ---- TIM2 ---- */
#define TIM2_BASE           0x40000000UL
#define TIM2_CR1            (*(volatile uint32_t *)(TIM2_BASE + 0x00UL))
#define TIM2_CR2            (*(volatile uint32_t *)(TIM2_BASE + 0x04UL))
#define TIM2_CNT            (*(volatile uint32_t *)(TIM2_BASE + 0x24UL))
#define TIM2_PSC            (*(volatile uint32_t *)(TIM2_BASE + 0x28UL))
#define TIM2_ARR            (*(volatile uint32_t *)(TIM2_BASE + 0x2CUL))

#define TIM2_CR1_CEN        (1UL << 0)
#define TIM2_CR2_MMS_SHIFT  4
#define TIM2_CR2_MMS_UPDATE (2UL << TIM2_CR2_MMS_SHIFT) /* 010 = TRGO on update event */

/* ---- DMA1 channel1 (fixed mapping for ADC1 on STM32L0) ---- */
#define DMA1_BASE           0x40020000UL
#define DMA1_ISR            (*(volatile uint32_t *)(DMA1_BASE + 0x00UL))
#define DMA1_IFCR           (*(volatile uint32_t *)(DMA1_BASE + 0x04UL))


#define DMA1_CH1_CCR        (*(volatile uint32_t *)(DMA1_BASE + 0x08UL))
#define DMA1_CH1_CNDTR      (*(volatile uint32_t *)(DMA1_BASE + 0x0CUL))
#define DMA1_CH1_CPAR       (*(volatile uint32_t *)(DMA1_BASE + 0x10UL))
#define DMA1_CH1_CMAR       (*(volatile uint32_t *)(DMA1_BASE + 0x14UL))

#define DMA1_CSELR          (*(volatile uint32_t *)(DMA1_BASE + 0xA8UL))

#define DMA_CSELR_C1S_MASK  (0xFUL << 0)
#define DMA_CSELR_C1S_ADC_PLACEHOLDER  (0UL << 0)

#define DMA_CCR_EN          (1UL << 0)
#define DMA_CCR_TCIE        (1UL << 1)
#define DMA_CCR_HTIE        (1UL << 2)
#define DMA_CCR_CIRC        (1UL << 5)
#define DMA_CCR_MINC        (1UL << 7)
#define DMA_CCR_PSIZE_16BIT (1UL << 8)
#define DMA_CCR_MSIZE_16BIT (1UL << 10)

#define DMA_ISR_HTIF1       (1UL << 2)
#define DMA_ISR_TCIF1       (1UL << 1)
#define DMA_IFCR_CHTIF1     (1UL << 2)
#define DMA_IFCR_CTCIF1     (1UL << 1)

/* ---- NVIC (Cortex-M0+) ---- */
#define NVIC_ISER           (*(volatile uint32_t *)(0xE000E100UL))
#define DMA1_Channel1_IRQn  9UL   /* verify vector table position for your part */

/* =====================================================================
 *  Local state
 * ===================================================================== */

static volatile uint16_t adc_buf[ADC_BUF_SIZE];
static volatile uint8_t  first_half_ready = 0;
static volatile uint8_t  second_half_ready = 0;

/* =====================================================================
 *  Init helpers
 * ===================================================================== */

static void adc_gpio_init(void)
{
    RCC_IOPENR |= RCC_IOPENR_GPIOAEN;

    /* PA0 -> analog mode (11) */
    GPIOA_MODER |= (3UL << (0 * 2));
}

static void adc_peripheral_init(void)
{
    RCC_APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* Ensure ADC is disabled before calibration/config */
    if (ADC1_CR & ADC_CR_ADEN)
    {
        ADC1_CR |= ADC_CR_ADDIS;
        while (ADC1_CR & ADC_CR_ADEN);
    }

    /* Async clock mode: dedicated 16MHz (HSI16) ADC clock, independent
       of AHB/APB prescalers */
    ADC1_CFGR2 &= ~ADC_CFGR2_CKMODE_MASK;

    /* Calibration - must run with ADEN = 0 */
    ADC1_CR |= ADC_CR_ADCAL;
    while (ADC1_CR & ADC_CR_ADCAL);

    /* Channel 0 (PA0) selected */
    ADC1_CHSELR = ADC_CHSELR_CHSEL0;

    /* Sampling time */
    ADC1_SMPR = ADC_SMPR_SMP_39_5CYC;

    ADC1_CFGR1 &= ~(ADC_CFGR1_EXTSEL_MASK | (3UL << ADC_CFGR1_EXTEN_SHIFT));
    ADC1_CFGR1 |= ADC_CFGR1_EXTSEL_TIM2_TRGO | ADC_CFGR1_EXTEN_RISING;

    /* DMA enabled, circular-compatible mode so ADC keeps issuing DMA
       requests indefinitely (matches DMA channel's own CIRC bit) */
    ADC1_CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG;

    /* Enable ADC and wait for it to become ready */
    ADC1_ISR |= ADC_ISR_ADRDY; /* clear any stale ready flag first */
    ADC1_CR |= ADC_CR_ADEN;
    while (!(ADC1_ISR & ADC_ISR_ADRDY));
}

static void adc_dma_init(void)
{
    RCC_AHBENR |= RCC_AHBENR_DMA1EN;

    DMA1_CSELR &= ~DMA_CSELR_C1S_MASK;
    DMA1_CSELR |= DMA_CSELR_C1S_ADC_PLACEHOLDER;

    DMA1_CH1_CCR = 0; /* disable + clear config before reconfiguring */

    DMA1_CH1_CPAR  = (uint32_t)&ADC1_DR;
    DMA1_CH1_CMAR  = (uint32_t)adc_buf;
    DMA1_CH1_CNDTR = ADC_BUF_SIZE;

    DMA1_CH1_CCR |= DMA_CCR_MINC
                  |  DMA_CCR_CIRC
                  |  DMA_CCR_PSIZE_16BIT
                  |  DMA_CCR_MSIZE_16BIT
                  |  DMA_CCR_HTIE
                  |  DMA_CCR_TCIE;

    NVIC_ISER = (1UL << DMA1_Channel1_IRQn);

    DMA1_CH1_CCR |= DMA_CCR_EN;
}

static void tim2_init(uint32_t sample_rate_hz)
{
    RCC_APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2_CR1 &= ~TIM2_CR1_CEN; /* stopped until ADC_Start() */
    TIM2_PSC = 0;

    uint32_t arr = (16000000UL / sample_rate_hz);
    TIM2_ARR = (arr > 0) ? (arr - 1UL) : 0UL;

    TIM2_CR2 &= ~(0x7UL << TIM2_CR2_MMS_SHIFT);
    TIM2_CR2 |= TIM2_CR2_MMS_UPDATE; /* TRGO fires on update event */
}

/* =====================================================================
 *  Public API
 * ===================================================================== */

void ADC_Init(uint32_t sample_rate_hz)
{
    adc_gpio_init();
    adc_peripheral_init();
    adc_dma_init();
    tim2_init(sample_rate_hz);
}

void ADC_Start(void)
{
    ADC1_CR |= ADC_CR_ADSTART;
    TIM2_CR1 |= TIM2_CR1_CEN;
}

void ADC_Stop(void)
{
    TIM2_CR1 &= ~TIM2_CR1_CEN;

    if (ADC1_CR & ADC_CR_ADSTART)
    {
        ADC1_CR |= ADC_CR_ADSTP;
        while (ADC1_CR & ADC_CR_ADSTART);
    }
}

uint8_t ADC_FirstHalfReady(void)
{
    if (first_half_ready)
    {
        first_half_ready = 0;
        return 1;
    }
    return 0;
}

uint8_t ADC_SecondHalfReady(void)
{
    if (second_half_ready)
    {
        second_half_ready = 0;
        return 1;
    }
    return 0;
}

volatile uint16_t *ADC_GetBuffer(void)
{
    return adc_buf;
}

void DMA1_Channel1_IRQHandler(void)
{
    if (DMA1_ISR & DMA_ISR_HTIF1)
    {
        DMA1_IFCR = DMA_IFCR_CHTIF1;
        first_half_ready = 1;
    }
    if (DMA1_ISR & DMA_ISR_TCIF1)
    {
        DMA1_IFCR = DMA_IFCR_CTCIF1;
        second_half_ready = 1;
    }
}
