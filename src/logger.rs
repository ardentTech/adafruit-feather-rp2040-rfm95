use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;

// The Adafruit board doesn't expose the SWD pins on the RP2040, so logging is handled via the
// USB-C connector. From what I currently understand, this also rules out using RTT, which then
// effectively rules out using `defmt`. Hopefully I'll figure out a way around this eventually.

#[embassy_executor::task]
pub async fn logger(driver: Driver<'static, USB>) {
    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}