use embassy_rp::i2c::I2c;
use embassy_rp::peripherals::{I2C1, SPI1};
use embassy_rp::spi::Spi;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use pmsa003i::Reading;

pub type I2c1Bus = Mutex<NoopRawMutex, I2c<'static, I2C1, embassy_rp::i2c::Async>>;
pub type Spi1Bus = Mutex<NoopRawMutex, Spi<'static, SPI1, embassy_rp::spi::Async>>;

pub enum AQSensor {
    // TODO change this so the lib struct is leaking into app space
    DataReady(Reading),
    Error,
}