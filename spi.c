#include "spi.h"
#include <string.h>

/* ---------------- RCC ---------------- */
#define RCC_BASE            0x40021000UL
#define RCC_IOPENR          (*(volatile uint32_t *)(RCC_BASE + 0x2CUL))
#define RCC_APB1ENR         (*(volatile uint32_t *)(RCC_BASE + 0x38UL))

#define RCC_IOPENR_GPIOAEN  (1UL << 0)
#define RCC_IOPENR_GPIOBEN  (1UL << 1)
#define RCC_APB1ENR_SPI2EN  (1UL << 14)

/* ---------------- GPIOA (DRDY = PA8) ---------------- */
#define GPIOA_BASE          0x50000000UL
#define GPIOA_MODER         (*(volatile uint32_t *)(GPIOA_BASE + 0x00UL))
#define GPIOA_BSRR          (*(volatile uint32_t *)(GPIOA_BASE + 0x18UL))

#define DRDY_PIN            8U /* PA8 */

/* ---------------- GPIOB (SPI2 pins) ---------------- */
#define GPIOB_BASE          0x50000400UL
#define GPIOB_MODER         (*(volatile uint32_t *)(GPIOB_BASE + 0x00UL))
#define GPIOB_OSPEEDR       (*(volatile uint32_t *)(GPIOB_BASE + 0x08UL))
#define GPIOB_AFRH          (*(volatile uint32_t *)(GPIOB_BASE + 0x24UL)) /* pins 8-15 */

/* ---------------- SPI2 ---------------- */
#define SPI2_BASE           0x40003800UL
#define SPI2_CR1            (*(volatile uint32_t *)(SPI2_BASE + 0x00UL))
#define SPI2_CR2            (*(volatile uint32_t *)(SPI2_BASE + 0x04UL))
#define SPI2_SR             (*(volatile uint32_t *)(SPI2_BASE + 0x08UL))
#define SPI2_DR             (*(volatile uint32_t *)(SPI2_BASE + 0x0CUL))

#define SPI_CR1_SPE         (1UL << 6)   /* MSTR=0 -> slave, CPOL=0, CPHA=0 */

#define SPI_CR2_FRXTH       (1UL << 12)
#define SPI_CR2_DS_SHIFT    8
#define SPI_CR2_DS_8BIT     (0x7UL << SPI_CR2_DS_SHIFT)

#define SPI_SR_RXNE         (1UL << 0)
#define SPI_SR_TXE          (1UL << 1)
#define SPI_SR_BSY          (1UL << 7)

/* =====================================================================
 *  Local state
 * ===================================================================== */

#define FRAME_HEADER_LEN  5U
#define FRAME_MAX_LEN     (FRAME_HEADER_LEN + (SPI_FRAME_MAX_SAMPLES * 2U) + 1U)

static uint8_t tx_frame[FRAME_MAX_LEN];

static void spi_gpio_init(void)
{
    RCC_IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;

    /* PB12 (NSS), PB13 (SCK), PB14 (MISO), PB15 (MOSI) -> alternate function */
    GPIOB_MODER &= ~((3UL << (12*2)) | (3UL << (13*2)) | (3UL << (14*2)) | (3UL << (15*2)));
    GPIOB_MODER |=  ((2UL << (12*2)) | (2UL << (13*2)) | (2UL << (14*2)) | (2UL << (15*2)));

    GPIOB_OSPEEDR |= (3UL << (12*2)) | (3UL << (13*2)) | (3UL << (14*2)) | (3UL << (15*2));

    GPIOB_AFRH &= ~((0xFUL << ((12-8)*4)) | (0xFUL << ((13-8)*4)) |
                     (0xFUL << ((14-8)*4)) | (0xFUL << ((15-8)*4)));

    /* PA8 = DRDY, push-pull output, starts low (no frame ready yet) */
    GPIOA_MODER &= ~(3UL << (DRDY_PIN * 2));
    GPIOA_MODER |=  (1UL << (DRDY_PIN * 2));
    GPIOA_BSRR = (1UL << (DRDY_PIN + 16)); /* force low at boot */
}

static void spi_peripheral_init(void)
{
    RCC_APB1ENR |= RCC_APB1ENR_SPI2EN;

    SPI2_CR1 = 0; /* slave mode, CPOL=0, CPHA=0 */
    SPI2_CR2 = SPI_CR2_DS_8BIT | SPI_CR2_FRXTH;
    SPI2_CR1 |= SPI_CR1_SPE;
}

void SPI_Init(void)
{
    spi_gpio_init();
    spi_peripheral_init();
}

uint8_t SPI_TxBusy(void)
{
    return 0;
}

uint8_t SPI_SendWaveform(volatile uint16_t *buf, uint16_t len)
{
    if (len > SPI_FRAME_MAX_SAMPLES)
        return 0;

    uint16_t frame_len = (uint16_t)(FRAME_HEADER_LEN + (len * 2U) + 1U);

    tx_frame[0] = 0xAA;
    tx_frame[1] = 0x55;
    tx_frame[2] = SPI_FRAME_CMD_WAVEFORM;
    tx_frame[3] = (uint8_t)(len & 0xFF);
    tx_frame[4] = (uint8_t)((len >> 8) & 0xFF);

    uint8_t checksum = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t lo = (uint8_t)(buf[i] & 0xFF);
        uint8_t hi = (uint8_t)((buf[i] >> 8) & 0xFF);
        tx_frame[FRAME_HEADER_LEN + i * 2U]      = lo;
        tx_frame[FRAME_HEADER_LEN + i * 2U + 1U] = hi;
        checksum ^= lo ^ hi;
    }
    tx_frame[FRAME_HEADER_LEN + len * 2U] = checksum;

    while (!(SPI2_SR & SPI_SR_TXE));
    SPI2_DR = tx_frame[0];

    GPIOA_BSRR = (1UL << DRDY_PIN);

    for (uint16_t i = 1; i < frame_len; i++)
    {
        while (!(SPI2_SR & SPI_SR_TXE));
        SPI2_DR = tx_frame[i];

        if (SPI2_SR & SPI_SR_RXNE)
        {
            (void)SPI2_DR; // READING A DATA REGISTER VOID TO DRAIN IT
        }
    }
    while (SPI2_SR & SPI_SR_BSY);

    (void)SPI2_DR;
    (void)SPI2_SR;     // CLEARING THE STATUS REGISTER BY READING IT VOID

    GPIOA_BSRR = (1UL << (DRDY_PIN + 16)); // set DRDY 0

    return 1;
}
