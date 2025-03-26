#![no_std]
#![no_main]

mod bsp;
mod logger;
mod types;
mod radio;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pin, Pull};
use embassy_rp::peripherals::{SPI1, USB};
use embassy_rp::spi::Spi;
use embassy_rp::usb::Driver;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use panic_halt as _;
use static_cell::StaticCell;
use crate::bsp::AdafruitFeatherRp2040Rfm95;
use crate::logger::logger;
use crate::radio::{radio_rx, radio_tx};
use crate::types::Spi1Bus;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut board = AdafruitFeatherRp2040Rfm95::default();
    let usb_driver = Driver::new(board.usb, Irqs);
    let spi = Spi::new(
        board.spi1,
        board.spi1_sck,
        board.spi1_mosi,
        board.spi1_miso,
        board.dma_ch0,
        board.dma_ch1,
        embassy_rp::spi::Config::default()
    );
    static SPI_BUS: StaticCell<Spi1Bus> = StaticCell::new();
    let spi_bus = SPI_BUS.init(Mutex::new(spi));

    // enable logging on host via USB-C
    spawner.must_spawn(logger(usb_driver));

    // radio tx
    let nss = Output::new(board.rfm_cs.degrade(), Level::High);
    let reset = Output::new(board.rfm_reset.degrade(), Level::High);
    let dio0 = Input::new(board.rfm_io0.degrade(), Pull::None);
    //spawner.must_spawn(radio_tx(spi_bus, nss, reset, dio0));
    spawner.must_spawn(radio_rx(spi_bus, nss, reset, dio0));

    loop {
        board.led.set_high();
        Timer::after_millis(1000).await;

        board.led.set_low();
        Timer::after_millis(1000).await;
    }
}
