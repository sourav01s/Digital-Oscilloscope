/*
 * spi.h
 *
 * Pure bare-metal SPI2 SLAVE driver for STM32L072CZ, DMA-fed TX, with a
 * DRDY handshake pin so the ESP32 (SPI master) knows when to come pull
 * a waveform frame. No CMSIS, no HAL.
 *
 * SPI2 (not SPI1) is used deliberately: on STM32L072CZ + SX1276 LoRa
 * boards, SPI1 (PA5/PA6/PA7) is normally wired internally to the onboard
 * radio module and PA7 isn't broken out - that's why it's missing on
 * your board. SPI2 uses a completely separate set of pins, so this
 * leaves the radio/SPI1 untouched.
 *
 * Why STM32 is the SLAVE here: SPI slaves can't initiate transfers, so
 * we can't just "push" data whenever we want. Instead, STM32 raises
 * DRDY (PB0) when a frame is ready; the ESP32 watches that pin via
 * interrupt and, once high, becomes bus master and clocks the frame out.
 * See the framing protocol below - matches the accompanying ESP32 sketch.
 *
 * Wiring assumed:
 *   PB12 = SPI2_NSS  (hardware slave select input, driven by ESP32's CS)
 *   PB13 = SPI2_SCK
 *   PB14 = SPI2_MISO (STM32 -> ESP32, this is the direction we actually use)
 *   PB15 = SPI2_MOSI (ESP32 -> STM32, unused payload-wise but must be wired)
 *   PA8  = DRDY      (STM32 -> ESP32, GPIO output, "frame ready" signal)
 *
 * NOTE ON PB0: earlier revisions of this driver used PB0 for DRDY. On
 * STM32L072CZ + SX1276 (Murata CMWX1ZZABZ-091) boards, PB0 is wired
 * INTERNALLY to the radio module's DIO2 pin - that's why it isn't
 * broken out on the board silkscreen at all, under any name. DRDY was
 * moved to PA8 (exposed as Arduino header pin "D7" on most of these
 * boards) instead. Confirm PA8 is free on your specific board revision
 * before wiring - check for LED/button/radio conflicts in the schematic.
 *
 * VERIFY PB12-PB15 are actually free/broken-out on your specific board
 * revision too - LoRa dev boards sometimes reuse GPIOs for onboard
 * LEDs, the radio's DIO/RESET lines, or other peripherals. Check your
 * board's schematic before wiring.
 *
 * Frame protocol (matches esp32_spi_web.ino):
 *   Byte 0-1: 0xAA 0x55      - magic/sync bytes
 *   Byte 2  : CMD             - 0x02 = waveform data push
 *   Byte 3-4: LEN_LO, LEN_HI  - number of uint16 samples, little-endian
 *   Byte 5..: payload         - LEN * 2 bytes, each sample little-endian
 *   Last byte: checksum       - XOR of every payload byte
 */

#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include <stdint.h>

#define SPI_FRAME_CMD_WAVEFORM   0x02U
#define SPI_FRAME_MAX_SAMPLES    256U   /* must be >= largest buffer you'll send */

/* Init GPIO (SPI1 AF pins + DRDY output) and SPI1 peripheral in slave mode
   with DMA TX. Call once at startup. */
void SPI_Init(void);

/*
 * Build a framed packet from `buf`/`len` and send it to the ESP32 over
 * SPI2, POLLED (blocking) - not DMA. Raises DRDY once the first byte is
 * pre-loaded, then feeds the rest of the frame as the ESP32 clocks it
 * out. Returns once the whole frame has physically finished
 * transmitting. Returns 0 only if len exceeds SPI_FRAME_MAX_SAMPLES.
 *
 * Blocking cost is small relative to the ADC half-buffer interval at
 * typical sample rates (low single-digit milliseconds per frame at a
 * 1-4MHz SPI clock), so this does not need to be asynchronous for this
 * project's data rates.
 */
uint8_t SPI_SendWaveform(volatile uint16_t *buf, uint16_t len);

/* Returns 1 if a frame is currently being transmitted (DRDY high / DMA
   or SPI shift register still active). */
uint8_t SPI_TxBusy(void);

#endif /* SPI_DRIVER_H */
