#![no_std]
#![no_main]

mod bsp;
mod logger;
mod common;
mod radio;
mod aq;
mod display;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pin, Pull};
use embassy_rp::i2c::{self};
use embassy_rp::peripherals::{I2C1, SPI1, USB};
use embassy_rp::spi::Spi;
use embassy_rp::usb::Driver;
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex};
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use panic_halt as _;
use static_cell::StaticCell;
use crate::aq::read_aq;
use crate::bsp::AdafruitFeatherRp2040Rfm95;
use crate::logger::logger;
use crate::radio::{radio_rx, radio_tx};
use crate::common::{AQSensor, I2c1Bus, Spi1Bus};
use crate::display::oled_display;

static CHANNEL: Channel<ThreadModeRawMutex, AQSensor, 64> = Channel::new();

bind_interrupts!(struct Irqs {
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut board = AdafruitFeatherRp2040Rfm95::default();

    let usb_driver = Driver::new(board.usb, Irqs);
    // enable logging on host via USB-C
    spawner.must_spawn(logger(usb_driver));

    // defaults to 100 kbps, which is the only speed the AQ sensor works with
    let i2c = i2c::I2c::new_async(board.i2c1, board.i2c1_scl, board.i2c1_sda, Irqs, i2c::Config::default());
    static I2C_BUS: StaticCell<I2c1Bus> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));

    // oled display
    let btn_a = Input::new(board.p9, Pull::Up);
    let btn_b = Input::new(board.p6, Pull::Up);
    let btn_c = Input::new(board.p5, Pull::Up);
    spawner.must_spawn(oled_display(i2c_bus, btn_a, btn_b, btn_c));

    // aq sensor
    //spawner.must_spawn(read_aq(i2c_bus, CHANNEL.sender()));

    // let spi = Spi::new(
    //     board.spi1,
    //     board.spi1_sck,
    //     board.spi1_mosi,
    //     board.spi1_miso,
    //     board.dma_ch0,
    //     board.dma_ch1,
    //     embassy_rp::spi::Config::default()
    // );
    // static SPI_BUS: StaticCell<Spi1Bus> = StaticCell::new();
    // let spi_bus = SPI_BUS.init(Mutex::new(spi));

    // radio tx
    // let nss = Output::new(board.rfm_cs.degrade(), Level::High);
    // let reset = Output::new(board.rfm_reset.degrade(), Level::High);
    // let dio0 = Input::new(board.rfm_io0.degrade(), Pull::None);
    // //spawner.must_spawn(radio_tx(spi_bus, nss, reset, dio0));
    // spawner.must_spawn(radio_rx(spi_bus, nss, reset, dio0));

    loop {
        match CHANNEL.receive().await {
            AQSensor::DataReady(data) => {
                log::info!("AQ is operational: {:?}", data);
            }
            AQSensor::Error => {
                log::info!("AQ is nonoperational")
            }
        }
    }
}
