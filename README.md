# Adafruit Feather RP2040 RFM95
This repo contains various forms of tire-kicking conducted in an effort to familiarize myself with the board while
working towards an environmental monitoring system for my shop. It leverages [embassy](https://github.com/embassy-rs/embassy)
and implements a handful of different protocols while integrating various peripherals.

## Debugging
The board doesn't expose the SWD pins on the RP2040, so logging is handled via the USB-C connector.

## Hardware
* Adafruit Feather RP2040 RFM95 board
    - [Guide](https://learn.adafruit.com/feather-rp2040-rfm95)
    - [Product](https://www.adafruit.com/product/5714)
    - [LoRa Module Datasheet](https://cdn-shop.adafruit.com/product-files/5714/SX1276-7-8.pdf)
    - [915 MHz Antenna](https://www.adafruit.com/product/4269)
* Adafruit PMSA003I Air Quality Breakout
    - [Datasheet](https://cdn-shop.adafruit.com/product-files/4632/4505_PMSA003I_series_data_manual_English_V2.6.pdf)
    - [Product](https://www.adafruit.com/product/4632)
* Adafruit FeatherWing OLED
    - [Datasheet](https://cdn-shop.adafruit.com/product-files/4650/4650_C14586.pdf)
    - [Product]((https://www.adafruit.com/product/4650))

## Commands
* Build: `$ cargo build --release`
* Flash: `$ cargo run --release`