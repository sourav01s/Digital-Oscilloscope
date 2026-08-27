/*
 * ssd1306.c
 *
 * Pure bare-metal SSD1306 128x64 OLED driver (I2C1) for STM32L072CZ.
 * No CMSIS structs/headers - every peripheral register is accessed as a
 * raw memory address defined below.
 *
 * ====================== VERIFY BEFORE TRUSTING ======================
 *  - All base addresses/offsets are taken from RM0376 (STM32L0x2 RM).
 *    Cross check every one against the memory map chapter for your exact
 *    part variant (L072 vs L071 vs L073 can shift some offsets).
 *  - I2C1_TIMINGR value targets ~400kHz off a 16MHz I2CCLK. Recompute
 *    with ST's I2C timing configuration tool for your real clock tree.
 *  - DMA channel chosen for I2C1_TX (channel 4) and its CSELR selector
 *    value are PLACEHOLDERS. Confirm the correct channel + selector
 *    against the "DMA1 request mapping" table in RM0376 for I2C1_TX.
 *  - PB8/PB9 AF4 = I2C1 assumption must be checked against the
 *    alternate function table in your datasheet.
 *  - IRQ vector name (DMA1_Channel4_5_6_7_IRQHandler) and its position
 *    in your startup file's vector table must match - if you pick a
 *    different DMA channel above, the shared IRQ line/name changes too.
 * ======================================================================
 */

#include "ssd1306.h"
#include <string.h>

/* =====================================================================
 *  Raw register access macros - direct memory-mapped addresses.
 *  Each is a dereferenced volatile pointer, matching classic
 *  register-programming style (no vendor struct overlays).
 * ===================================================================== */

/* ---- RCC ---- */
#define RCC_BASE            0x40021000UL
#define RCC_IOPENR          (*(volatile uint32_t *)(RCC_BASE + 0x2CUL))
#define RCC_APB1ENR         (*(volatile uint32_t *)(RCC_BASE + 0x38UL))
#define RCC_AHBENR          (*(volatile uint32_t *)(RCC_BASE + 0x30UL))

#define RCC_IOPENR_GPIOBEN  (1UL << 1)
#define RCC_APB1ENR_I2C1EN  (1UL << 21)
#define RCC_AHBENR_DMA1EN   (1UL << 0)

/* ---- GPIOB ---- */
#define GPIOB_BASE          0x50000400UL
#define GPIOB_MODER         (*(volatile uint32_t *)(GPIOB_BASE + 0x00UL))
#define GPIOB_OTYPER        (*(volatile uint32_t *)(GPIOB_BASE + 0x04UL))
#define GPIOB_OSPEEDR       (*(volatile uint32_t *)(GPIOB_BASE + 0x08UL))
#define GPIOB_PUPDR         (*(volatile uint32_t *)(GPIOB_BASE + 0x0CUL))
#define GPIOB_AFRH          (*(volatile uint32_t *)(GPIOB_BASE + 0x24UL)) /* AFR[1], pins 8-15 */

/* ---- I2C1 ---- */
#define I2C1_BASE           0x40005400UL
#define I2C1_CR1            (*(volatile uint32_t *)(I2C1_BASE + 0x00UL))
#define I2C1_CR2            (*(volatile uint32_t *)(I2C1_BASE + 0x04UL))
#define I2C1_TIMINGR        (*(volatile uint32_t *)(I2C1_BASE + 0x10UL))
#define I2C1_ISR            (*(volatile uint32_t *)(I2C1_BASE + 0x18UL))
#define I2C1_ICR            (*(volatile uint32_t *)(I2C1_BASE + 0x1CUL))
#define I2C1_TXDR           (*(volatile uint32_t *)(I2C1_BASE + 0x28UL))

#define I2C_CR1_PE          (1UL << 0)
#define I2C_CR1_TXDMAEN     (1UL << 14)

#define I2C_CR2_START       (1UL << 13)
#define I2C_CR2_AUTOEND     (1UL << 25)
#define I2C_CR2_SADD_SHIFT  0
#define I2C_CR2_NBYTES_SHIFT 16

#define I2C_ISR_TXIS        (1UL << 1)
#define I2C_ISR_NACKF       (1UL << 4)
#define I2C_ISR_STOPF       (1UL << 5)

#define I2C_ICR_NACKCF      (1UL << 4)
#define I2C_ICR_STOPCF      (1UL << 5)

/* ---- DMA1 (channel 4 used for I2C1_TX - VERIFY, see header comment) ---- */
#define DMA1_BASE           0x40020000UL
#define DMA1_ISR            (*(volatile uint32_t *)(DMA1_BASE + 0x00UL))
#define DMA1_IFCR           (*(volatile uint32_t *)(DMA1_BASE + 0x04UL))

/* Channel register block: CCRx/CNDTRx/CPARx/CMARx, 0x14 bytes apart,
   channel 1 block starts at offset 0x08. Channel 4 -> index 3. */
#define DMA1_CH4_CCR        (*(volatile uint32_t *)(DMA1_BASE + 0x08UL + 3UL * 0x14UL))
#define DMA1_CH4_CNDTR       (*(volatile uint32_t *)(DMA1_BASE + 0x0CUL + 3UL * 0x14UL))
#define DMA1_CH4_CPAR        (*(volatile uint32_t *)(DMA1_BASE + 0x10UL + 3UL * 0x14UL))
#define DMA1_CH4_CMAR        (*(volatile uint32_t *)(DMA1_BASE + 0x14UL + 3UL * 0x14UL))

#define DMA1_CSELR          (*(volatile uint32_t *)(DMA1_BASE + 0xA8UL))

#define DMA_CCR_EN          (1UL << 0)
#define DMA_CCR_TCIE        (1UL << 1)
#define DMA_CCR_DIR         (1UL << 4)   /* 1 = read from memory (mem->periph) */
#define DMA_CCR_MINC        (1UL << 7)

/* Channel 4 status/clear flags: GIF4=bit12, TCIF4=bit13, HTIF4=bit14, TEIF4=bit15 */
#define DMA_ISR_TCIF4       (1UL << 13)
#define DMA_IFCR_CTCIF4     (1UL << 13)

/* CSELR field for channel 4 = bits [15:12]. Selector value is a
   PLACEHOLDER - confirm the correct code for I2C1_TX in RM0376. */
#define DMA_CSELR_C4S_SHIFT 12
#define DMA_CSELR_C4S_MASK  (0xFUL << DMA_CSELR_C4S_SHIFT)
#define DMA_CSELR_C4S_I2C1_TX_PLACEHOLDER  (6UL << DMA_CSELR_C4S_SHIFT)

/* ---- NVIC (Cortex-M0+, single ISER register covers IRQ0-31) ---- */
#define NVIC_ISER           (*(volatile uint32_t *)(0xE000E100UL))
#define DMA1_CH4_5_6_7_IRQn 11UL   /* verify vector table position for your part */

/* =====================================================================
 *  Local state
 * ===================================================================== */

static uint8_t framebuf[SSD1306_BUF_SIZE];
static uint8_t dma_tx_buf[1 + SSD1306_BUF_SIZE]; /* control byte + full framebuffer */
static volatile uint8_t dma_busy = 0;

/* =====================================================================
 *  Low-level I2C1 (bare metal, blocking)
 * ===================================================================== */

static void i2c1_gpio_init(void)
{
    RCC_IOPENR |= RCC_IOPENR_GPIOBEN;

    /* PB8/PB9 -> alternate function mode (10) */
    GPIOB_MODER &= ~((3UL << (8 * 2)) | (3UL << (9 * 2)));
    GPIOB_MODER |=  ((2UL << (8 * 2)) | (2UL << (9 * 2)));

    /* Open-drain (required for I2C) */
    GPIOB_OTYPER |= (1UL << 8) | (1UL << 9);

    /* Pull-ups on (add external pull-ups too if your board lacks them) */
    GPIOB_PUPDR &= ~((3UL << (8 * 2)) | (3UL << (9 * 2)));
    GPIOB_PUPDR |=  ((1UL << (8 * 2)) | (1UL << (9 * 2)));

    /* High speed */
    GPIOB_OSPEEDR |= (3UL << (8 * 2)) | (3UL << (9 * 2));

    /* AF4 = I2C1 on PB8/PB9 - verify against datasheet AF table.
       AFRH covers pins 8-15, 4 bits each, pin8 -> bits[3:0], pin9 -> bits[7:4] */
    GPIOB_AFRH &= ~((0xFUL << ((8 - 8) * 4)) | (0xFUL << ((9 - 8) * 4)));
    GPIOB_AFRH |=  ((4UL   << ((8 - 8) * 4)) | (4UL   << ((9 - 8) * 4)));
}

static void i2c1_peripheral_init(void)
{
    RCC_APB1ENR |= RCC_APB1ENR_I2C1EN;

    I2C1_CR1 &= ~I2C_CR1_PE;

    /* Fast Mode ~400kHz off 16MHz I2CCLK - VERIFY with timing calculator */
    I2C1_TIMINGR = 0x00300F38UL;

    I2C1_CR1 |= I2C_CR1_PE;
}

/* Blocking write of `len` bytes (first byte is normally the SSD1306 control byte). */
static void i2c1_write(uint8_t addr7, const uint8_t *data, uint16_t len)
{
    I2C1_CR2 = 0;
    I2C1_CR2 |= ((uint32_t)addr7 << 1)
              |  ((uint32_t)len << I2C_CR2_NBYTES_SHIFT)
              |  I2C_CR2_AUTOEND
              |  I2C_CR2_START;

    for (uint16_t i = 0; i < len; i++)
    {
        while (!(I2C1_ISR & I2C_ISR_TXIS))
        {
            if (I2C1_ISR & I2C_ISR_NACKF)
            {
                I2C1_ICR |= I2C_ICR_NACKCF;
                return; /* NACK - bail out; add retry/error handling if needed */
            }
        }
        I2C1_TXDR = data[i];
    }

    while (!(I2C1_ISR & I2C_ISR_STOPF));
    I2C1_ICR |= I2C_ICR_STOPCF;
}

/* =====================================================================
 *  DMA1 channel 4 for I2C1 TX
 * ===================================================================== */

static void i2c1_dma_init(void)
{
    RCC_AHBENR |= RCC_AHBENR_DMA1EN;

    /* Select I2C1_TX as request source for channel 4 - PLACEHOLDER value */
    DMA1_CSELR &= ~DMA_CSELR_C4S_MASK;
    DMA1_CSELR |= DMA_CSELR_C4S_I2C1_TX_PLACEHOLDER;

    DMA1_CH4_CPAR = (uint32_t)&I2C1_TXDR;

    DMA1_CH4_CCR = 0;
    DMA1_CH4_CCR |= DMA_CCR_MINC   /* memory address increments      */
                  |  DMA_CCR_DIR   /* memory -> peripheral direction */
                  |  DMA_CCR_TCIE; /* transfer complete interrupt    */
    /* PSIZE/MSIZE left at 00 (8-bit), correct for I2C1_TXDR */

    NVIC_ISER = (1UL << DMA1_CH4_5_6_7_IRQn);

    I2C1_CR1 |= I2C_CR1_TXDMAEN;
}

/* Kick a DMA-driven I2C write of `len` bytes starting at dma_tx_buf. */
static void i2c1_write_dma(uint8_t addr7, uint16_t len)
{
    dma_busy = 1;

    DMA1_CH4_CCR &= ~DMA_CCR_EN;
    DMA1_CH4_CMAR = (uint32_t)dma_tx_buf;
    DMA1_CH4_CNDTR = len;
    DMA1_CH4_CCR |= DMA_CCR_EN;

    I2C1_CR2 = 0;
    I2C1_CR2 |= ((uint32_t)addr7 << 1)
              |  ((uint32_t)len << I2C_CR2_NBYTES_SHIFT)
              |  I2C_CR2_AUTOEND
              |  I2C_CR2_START;
    /* TXDMAEN already set in i2c1_dma_init() - I2C1 pulls bytes from DMA automatically */
}

void SSD1306_I2C_DMA_IRQHandler(void)
{
    if (DMA1_ISR & DMA_ISR_TCIF4)
    {
        DMA1_IFCR = DMA_IFCR_CTCIF4;
        dma_busy = 0;
    }
}

uint8_t SSD1306_DMA_Busy(void)
{
    return dma_busy;
}

/* =====================================================================
 *  SSD1306 command helpers
 * ===================================================================== */

static void ssd1306_cmd(uint8_t cmd)
{
    uint8_t buf[2] = { 0x00, cmd }; /* 0x00 = control byte, command stream */
    i2c1_write(SSD1306_I2C_ADDR, buf, 2);
}

/* =====================================================================
 *  Public API
 * ===================================================================== */

void SSD1306_Init(void)
{
    i2c1_gpio_init();
    i2c1_peripheral_init();
    i2c1_dma_init();

    static const uint8_t init_cmds[] = {
        0xAE,             /* display off */
        0x20, 0x00,       /* memory addressing mode: horizontal */
        0x21, 0x00, 0x7F, /* column address range 0..127 */
        0x22, 0x00, 0x07, /* page address range 0..7 */
        0xB0,             /* page start (redundant in horizontal mode, kept for safety) */
        0xC8,             /* COM output scan direction, remapped */
        0x00, 0x10,       /* lower/upper column start address = 0 */
        0x40,             /* display start line = 0 */
        0x81, 0x7F,       /* contrast */
        0xA1,             /* segment remap */
        0xA6,             /* normal (not inverted) display */
        0xA8, 0x3F,       /* multiplex ratio = 64 */
        0xA4,             /* entire display follows RAM contents */
        0xD3, 0x00,       /* display offset = 0 */
        0xD5, 0x80,       /* display clock divide ratio / osc freq */
        0xD9, 0xF1,       /* pre-charge period */
        0xDA, 0x12,       /* COM pins hardware config */
        0xDB, 0x40,       /* VCOMH deselect level */
        0x8D, 0x14,       /* charge pump enable (required on most SSD1306 modules) */
        0xAF              /* display ON */
    };

    for (uint16_t i = 0; i < sizeof(init_cmds); i++)
        ssd1306_cmd(init_cmds[i]);

    SSD1306_Clear();
}

void SSD1306_Clear(void)
{
    memset(framebuf, 0x00, SSD1306_BUF_SIZE);
}

void SSD1306_Fill(uint8_t color)
{
    memset(framebuf, color ? 0xFF : 0x00, SSD1306_BUF_SIZE);
}

void SSD1306_SetPixel(int16_t x, int16_t y, uint8_t color)
{
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT)
        return;

    uint16_t byte_idx = (uint16_t)x + (uint16_t)(y / 8) * SSD1306_WIDTH;
    uint8_t  bit_mask  = (uint8_t)(1u << (y % 8));

    if (color)
        framebuf[byte_idx] |= bit_mask;
    else
        framebuf[byte_idx] &= (uint8_t)~bit_mask;
}

void SSD1306_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color)
{
    int16_t dx = (int16_t)((x1 > x0) ? (x1 - x0) : (x0 - x1));
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t dy = (int16_t)-((y1 > y0) ? (y1 - y0) : (y0 - y1));
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = (int16_t)(dx + dy);
    int16_t e2;

    while (1)
    {
        SSD1306_SetPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1)
            break;
        e2 = (int16_t)(2 * err);
        if (e2 >= dy) { err = (int16_t)(err + dy); x0 = (int16_t)(x0 + sx); }
        if (e2 <= dx) { err = (int16_t)(err + dx); y0 = (int16_t)(y0 + sy); }
    }
}

void SSD1306_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color)
{
    SSD1306_DrawLine(x,                    y,                    (int16_t)(x + w - 1), y,                    color);
    SSD1306_DrawLine(x,                    (int16_t)(y + h - 1), (int16_t)(x + w - 1), (int16_t)(y + h - 1), color);
    SSD1306_DrawLine(x,                    y,                    x,                    (int16_t)(y + h - 1), color);
    SSD1306_DrawLine((int16_t)(x + w - 1), y,                    (int16_t)(x + w - 1), (int16_t)(y + h - 1), color);
}

void SSD1306_UpdateScreen(void)
{
    for (uint8_t page = 0; page < SSD1306_PAGES; page++)
    {
        ssd1306_cmd((uint8_t)(0xB0 + page)); /* page start */
        ssd1306_cmd(0x00);                    /* column low nibble  */
        ssd1306_cmd(0x10);                    /* column high nibble */

        uint8_t buf[SSD1306_WIDTH + 1];
        buf[0] = 0x40; /* data control byte */
        memcpy(&buf[1], &framebuf[page * SSD1306_WIDTH], SSD1306_WIDTH);

        i2c1_write(SSD1306_I2C_ADDR, buf, SSD1306_WIDTH + 1);
    }
}

void SSD1306_UpdateScreen_DMA(void)
{
    if (dma_busy)
        return; /* previous update still in flight - caller should poll SSD1306_DMA_Busy() */

    /* Horizontal addressing mode (set at init) lets the whole 1024-byte
       framebuffer go out as one continuous data stream - no per-page loop. */
    dma_tx_buf[0] = 0x40; /* data control byte */
    memcpy(&dma_tx_buf[1], framebuf, SSD1306_BUF_SIZE);

    /* Reset the column/page window so the controller writes from the
       start of RAM again on this update. */
    ssd1306_cmd(0x21); ssd1306_cmd(0x00); ssd1306_cmd(0x7F);
    ssd1306_cmd(0x22); ssd1306_cmd(0x00); ssd1306_cmd(0x07);

    i2c1_write_dma(SSD1306_I2C_ADDR, SSD1306_BUF_SIZE + 1);
}

void SSD1306_DrawWaveform(volatile uint16_t *buf, uint16_t len, uint16_t adc_max)
{
    SSD1306_Clear();

    if (len < 2 || adc_max == 0)
        return;

    uint16_t step = (uint16_t)(len / SSD1306_WIDTH);
    if (step == 0) step = 1;

    int16_t prev_y = -1;

    for (int16_t x = 0; x < SSD1306_WIDTH; x++)
    {
        uint16_t sample_idx = (uint16_t)(x * step);
        if (sample_idx >= len) sample_idx = (uint16_t)(len - 1);

        uint16_t sample = buf[sample_idx];
        if (sample > adc_max) sample = adc_max; /* clamp defensively */

        int16_t y = (int16_t)((SSD1306_HEIGHT - 1) -
                               ((uint32_t)sample * (SSD1306_HEIGHT - 1)) / adc_max);

        if (prev_y == -1)
            prev_y = y;

        SSD1306_DrawLine((int16_t)(x - 1), prev_y, x, y, SSD1306_COLOR_WHITE);
        prev_y = y;
    }
}
