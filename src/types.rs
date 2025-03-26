use embassy_rp::peripherals::SPI1;
use embassy_rp::spi::Spi;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;

pub type Spi1Bus = Mutex<NoopRawMutex, Spi<'static, SPI1, embassy_rp::spi::Async>>;