STM32L072CZ Bare-Metal Oscilloscope

A DIY oscilloscope: an STM32L072CZ samples an analog signal on its ADC and streams it over SPI to an ESP32, which serves a live waveform display to any browser on the network. Everything on the STM32 side is written bare-metal .

How it works

[Analog signal 0-3.3V] -> STM32 PA0 (ADC1_IN0)
                                |
                    TIM2-triggered, DMA-fed ADC sampling
                                |
                    rising-edge trigger (stable display)
                                |
                    SPI2 slave, polled TX, DRDY handshake
                                |
                                v
                    ESP32 (SPI master) -> WiFi -> WebSocket
                                |
                                v
                    Browser: live canvas plot + voltage/frequency stats
                      ![Waveform display](images/waveform.png)
#Hardware

STM32L072CZ board
ESP32 dev board (classic ESP32, WROOM-32 based)
A signal source for testing (potentiometer wiper works fine)
Common ground between STM32 and ESP32 is required

#Wiring

Signal	     STM32 pin	          ESP32 pin

Analog in	  PA0	
SPI SCK    	PB13               GPIO18
SPI MISO	   PB14              	GPIO19
SPI MOSI	   PB15	              GPIO23
SPI CS/NSS 	PB12	              GPIO5
DRDY	       PA8	               GPIO4
GND	       (common)

#Files

File	Purpose
adc.h / adc.c	ADC1 + TIM2 + DMA1 driver - samples PA0 at a fixed rate into a circular buffer
spi.h / spi.c	SPI2 slave driver - transmit with DRDY handshake framing
esp32_spi_web.ino	ESP32 sketch: SPI master, WiFi, WebSocket server, embedded web page

Building and flashing

STM32 side:

Put adc.h, spi.h in Core/Inc/
Put adc.c, spi.c, and test_spi_main.c (as your main.c) in Core/Src/
Build and flash via STM32CubeIDE or your Makefile toolchain

ESP32 side:

Install ESPAsyncWebServer and AsyncTCP libraries (Arduino Library Manager)
Edit WIFI_SSID / WIFI_PASS in esp32_spi_web.ino
Flash via Arduino IDE
Open Serial Monitor (115200 baud) to get the assigned IP address
Visit http://<that-ip>/ in a browser on the same network
Features
10 kHz sampling by default (configurable via ADC_Init(rate))
Rising-edge trigger - stabilizes the displayed waveform for periodic signals instead of scrolling randomly
Live voltage readouts: Vmax, Vmin, Vpp, Vavg (converted from raw ADC codes using a 3.3V reference)
Frequency estimation via zero-crossing detection
Voltage-labeled gridlines and timebase display on the canvas
Important limits and things to verify
ADC input range is strictly 0-3.3V. Never feed a signal outside this range into PA0 - use a voltage divider and/or DC offset circuit for larger/bipolar signals.
Several register values (ADC trigger source encoding, some AF pin mappings) were taken from best-available documentation/community sources and marked in code comments as needing verification against RM0376 for your exact chip revision. Cross-check anything marked PLACEHOLDER or VERIFY before relying on this in a real measurement context.
SAMPLE_RATE_HZ in the web page's JavaScript must match whatever value you pass to ADC_Init() on the STM32, or the frequency/timebase readouts will be wrong.
The SPI link is deliberately polled, not DMA-driven - an earlier DMA-based version had persistent framing issues traced to an unconfirmed DMA channel mapping. Polled transmit is slightly less CPU-efficient but has proven reliable at this project's data rates.
Troubleshooting

If nothing shows up in the browser, work through the pipeline in this order:

Check ESP32 Serial Monitor - does it print an IP address and post-connection stats
If ok stays at 0, run test_spi_raw_main.c + test_spi_raw.ino to isolate whether the raw SPI electrical link (wiring/mode/clock) works at all, independent of the ADC/framing logic.
If the raw link works but the full pipeline doesn't, check common ground and pin mapping again - most issues so far have come from a swapped or mismapped pin.
