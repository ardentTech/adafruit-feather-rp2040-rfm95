use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pin, Pull};
use embassy_rp::peripherals::{PIN_4, PIN_3, PIN_2, I2C1, USB, PIN_17, PIN_16, PIN_21, PIN_22, PIN_23, PIN_19, PIN_20, PIN_18, SPI1, PIN_14, PIN_15, PIN_8, PIN_0, PIN_1, PIN_5, PIN_9, PIN_10, PIN_11, PIN_12, PIN_25, PIN_24, PIN_26, PIN_27, PIN_28, PIN_29, DMA_CH0, DMA_CH1};

pub type Button = Input<'static>;
pub type Led = Output<'static>;

// Human-readable names are much easier to work with than numeric pins. Furthermore, introducing
// this abstraction will cut down on the number of times you need to reference the manufacturer's
// pin mappings anytime you need to use a new protocol or integrate a new peripheral.
//
// see https://github.com/embassy-rs/embassy/blob/main/embassy-rp/src/lib.rs#L207
// and https://learn.adafruit.com/feather-rp2040-rfm95/pinouts

#[allow(dead_code)]
pub struct AdafruitFeatherRp2040Rfm95 {
    pub boot: Button,
    pub dma_ch0: DMA_CH0,
    pub dma_ch1: DMA_CH1,
    pub led: Led,
    pub neo_pixel: PIN_4,
    pub usb: USB,

    // i2c
    pub i2c1: I2C1,
    pub i2c1_scl: PIN_3,
    pub i2c1_sda: PIN_2,

    // gpio
    pub p5: PIN_5,
    pub p9: PIN_9,
    pub p10: PIN_10,
    pub p11: PIN_11,
    pub p12: PIN_12,
    pub p24: PIN_24,
    pub p25: PIN_25,
    pub p26: PIN_26,
    pub p27: PIN_27,
    pub p28: PIN_28,
    pub p29: PIN_29,

    // rfm
    pub rfm_cs: PIN_16,
    pub rfm_reset: PIN_17,
    pub rfm_io0: PIN_21,
    pub rfm_io1: PIN_22,
    pub rfm_io2: PIN_23,
    pub rfm_io3: PIN_19,
    pub rfm_io4: PIN_20,
    pub rfm_io5: PIN_18,

    // spi
    pub spi1: SPI1,
    pub spi1_sck: PIN_14,
    pub spi1_mosi: PIN_15,
    pub spi1_miso: PIN_8,

    // uart
    pub uart0_tx: PIN_0,
    pub uart0_rx: PIN_1,
}

impl AdafruitFeatherRp2040Rfm95 {
    pub fn new(config: embassy_rp::config::Config) -> Self {
        let p = embassy_rp::init(config);

        Self {
            boot: button(p.PIN_7.degrade()),
            dma_ch0: p.DMA_CH0,
            dma_ch1: p.DMA_CH1,
            led: output_pin(p.PIN_13.degrade()),
            neo_pixel: p.PIN_4,
            usb: p.USB,
            i2c1: p.I2C1,
            i2c1_scl: p.PIN_3,
            i2c1_sda: p.PIN_2,
            p5: p.PIN_5,
            p9: p.PIN_9,
            p10: p.PIN_10,
            p11: p.PIN_11,
            p12: p.PIN_12,
            p24: p.PIN_24,
            p25: p.PIN_25,
            p26: p.PIN_26,
            p27: p.PIN_27,
            p28: p.PIN_28,
            p29: p.PIN_29,
            rfm_cs: p.PIN_16,
            rfm_reset: p.PIN_17,
            rfm_io0: p.PIN_21,
            rfm_io1: p.PIN_22,
            rfm_io2: p.PIN_23,
            rfm_io3: p.PIN_19,
            rfm_io4: p.PIN_20,
            rfm_io5: p.PIN_18,
            spi1: p.SPI1,
            spi1_sck: p.PIN_14,
            spi1_mosi: p.PIN_15,
            spi1_miso: p.PIN_8,
            uart0_rx: p.PIN_1,
            uart0_tx: p.PIN_0
        }
    }
}

impl Default for AdafruitFeatherRp2040Rfm95 {
    fn default() -> Self {
        Self::new(Default::default())
    }
}

fn button(pin: AnyPin) -> Input<'static> {
    Input::new(pin, Pull::Up)
}

fn output_pin(pin: AnyPin) -> Output<'static> {
    Output::new(pin, Level::Low)
}