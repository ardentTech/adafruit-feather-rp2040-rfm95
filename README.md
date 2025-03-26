# Adafruit Feather RP2040 RFM95
This repo contains various forms of tire-kicking conducted in an effort to familiarize myself with the board while
working towards an environmental monitoring system for my shop. It leverages [embassy](https://github.com/embassy-rs/embassy)
and implements a handful of different protocols while integrating various peripherals.

## Debugging
The board doesn't expose the SWD pins on the RP2040, so logging is handled via the USB-C connector.

## Hardware
* [Adafruit Feather RP2040 RFM95 board](https://www.adafruit.com/product/5714)

## Resources
* [Adafruit Feather RP2040 RFM95 guide](https://learn.adafruit.com/feather-rp2040-rfm95)
* [Semtech SX1276 LoRa transceiver datasheet](https://cdn-shop.adafruit.com/product-files/5714/SX1276-7-8.pdf)
* [LoRa1276-C1 module datasheet](https://cdn-shop.adafruit.com/product-files/5714/5714_LoRa127X-C1+100mW+LoRa+Wireless+Transceiver+Module+V3.0.pdf)