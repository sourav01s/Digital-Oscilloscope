/*
 * ssd1306.h
 *
 * Pure bare-metal SSD1306 128x64 OLED driver (I2C1) for STM32L072CZ.
 * No CMSIS, no HAL - all registers accessed as raw memory addresses.
 *
 * Wiring assumed:
 *   PB8 = I2C1_SCL (AF4)
 *   PB9 = I2C1_SDA (AF4)
 *
 * All base addresses / offsets are taken from RM0376 (STM32L0x2 reference
 * manual). Double-check them against your exact part's memory map before
 * trusting this blindly - address typos here are silent, ugly bugs.
 */

#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>

/* ---------------- Display geometry ---------------- */
#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          64
#define SSD1306_PAGES           (SSD1306_HEIGHT / 8)
#define SSD1306_BUF_SIZE        (SSD1306_WIDTH * SSD1306_PAGES)   /* 1024 bytes */

#define SSD1306_I2C_ADDR        0x3C   /* 7-bit address */

/* Pixel color */
#define SSD1306_COLOR_BLACK     0
#define SSD1306_COLOR_WHITE     1

/* ---------------- Public API ---------------- */

/* Bring up I2C1 peripheral + GPIO AF + SSD1306 init sequence. Blocking. */
void SSD1306_Init(void);

/* Clear the local framebuffer (does NOT push to screen, call Update after). */
void SSD1306_Clear(void);

/* Fill entire local framebuffer with given color. */
void SSD1306_Fill(uint8_t color);

/* Set/clear a single pixel in the local framebuffer. Bounds-checked. */
void SSD1306_SetPixel(int16_t x, int16_t y, uint8_t color);

/* Bresenham line draw into the local framebuffer. */
void SSD1306_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t color);

/* Draw a hollow rectangle - handy for axes / grid / borders. */
void SSD1306_DrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color);

/* Push the entire local framebuffer to the OLED over I2C, BLOCKING. */
void SSD1306_UpdateScreen(void);

/*
 * Push the entire local framebuffer to the OLED over I2C using DMA.
 * Returns immediately; poll SSD1306_DMA_Busy() for completion.
 * Do NOT touch the framebuffer or call this again while busy.
 */
void SSD1306_UpdateScreen_DMA(void);

/* Returns 1 if a DMA-driven screen update is still in progress. */
uint8_t SSD1306_DMA_Busy(void);

/*
 * Downsample an ADC sample buffer to screen width and draw it as a
 * connected waveform trace. adc_max is the full-scale ADC code
 * (e.g. 4095 for 12-bit). Clears the framebuffer first.
 * Does NOT push to the screen - call an Update function after.
 */
void SSD1306_DrawWaveform(volatile uint16_t *buf, uint16_t len, uint16_t adc_max);

/*
 * Call this from your DMA1 IRQ handler that services the channel used
 * for I2C1 TX. On STM32L072 that vector is shared: DMA1_Channel4_5_6_7_IRQHandler.
 * See the top comment in ssd1306.c for the exact channel/IRQ used and
 * where to double check it against RM0376.
 */
void SSD1306_I2C_DMA_IRQHandler(void);

#endif /* SSD1306_H */
