#![no_std]
#![no_main]

mod bsp;
mod logger;

use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_time::Timer;
use panic_halt as _;
use crate::bsp::AdafruitFeatherRp2040Rfm95;
use crate::logger::logger;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut board = AdafruitFeatherRp2040Rfm95::default();
    let usb_driver = Driver::new(board.usb, Irqs);

    // enable logging on host via USB-C
    spawner.must_spawn(logger(usb_driver));

    loop {
        log::info!("tick");
        board.led.set_high();
        Timer::after_millis(1000).await;

        board.led.set_low();
        Timer::after_millis(1000).await;
    }
}
