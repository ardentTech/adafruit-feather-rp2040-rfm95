# Adafruit Feather RP2040 RFM95
This repo contains various forms of tire-kicking conducted in an effort to familiarize myself with the board while
working towards an environmental monitoring system for my shop. It leverages [embassy](https://github.com/embassy-rs/embassy)
and implements a handful of different protocols while integrating various peripherals.

## Debugging
The board doesn't expose the SWD pins on the RP2040, so logging is handled via the USB-C connector.

## Boot2
The board uses an external Winbond W25Q64JV Flash chip, and since this chip can and does vary amongst different
boards, a second stage bootloader is required to configure the external Flash memory. While `embassy-rp` is a dependency
and the `rp2040` feature is brought into scope, what is NOT scoped is a related boot2 implementation. This is because
the W25Q64JV is not explicitly supported within `embassy-rp`. With no explicit boot2, `embassy-rp` defaults to the
W25Q080 boot2 implementation, which **appears to just work** with the Adafruit Feather RP2040 RFM95 board. Wild.

FWIW I do plan on digging into the two datasheets to verify this, and eventually opening a PR on `embassy-rp` to make
the W25Q64JV chip an explicit option.

If you want to want more info about the RP2040's boot sequence, check out this great article from [Van Hunter Adams](https://vanhunteradams.com/Pico/Bootloader/Boot_sequence.html).

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