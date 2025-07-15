#![no_std]

use crc::{Crc, CRC_8_NRSC_5};
use embedded_hal::i2c::{I2c, SevenBitAddress};

const SHT30_ADDRESS: SevenBitAddress = 0x44;
// low repeatability with clock stretching
const READ_CMD: [u8; 2] = [0x2c, 0x10];


#[derive(Debug, PartialEq)]
pub enum Sht30Error<E> {
    I2C(E),
    InvalidCrc
}

#[derive(Debug)]
pub struct Sht30Reading {
    pub humidity: u16,
    pub temperature_f: u16,
}

impl Sht30Reading {
    pub fn new(humidity: u16, temperature: u16) -> Self {
        Self { humidity, temperature_f: temperature }
    }
}

pub struct Sht30<I2C> {
    i2c: I2C
}

impl<I2C: I2c> Sht30<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    fn calculate_crc(a: &[u8; 2]) -> u8 {
        let crc = Crc::<u8>::new(&CRC_8_NRSC_5);
        let mut digest = crc.digest();
        digest.update(a);
        digest.finalize()
    }

    fn check_crc(a: &[u8; 2], b: u8) -> Result<(), Sht30Error<I2C::Error>> {
        if Self::calculate_crc(a) != b {
            Err(Sht30Error::InvalidCrc)
        } else {
            Ok(())
        }
    }

    #[inline]
    fn join_u16(data: &[u8; 2]) -> u16 {
        (data[0] as u16) << 8 | (data[1] as u16)
    }

    /// Perform a single-shot measurement
    ///
    /// This driver uses clock stretching so the result of the measurement is returned
    /// as soon as the data is available after the measurement command has been sent to the sensor.
    /// Therefore this call will take at least 4 ms and at most 15.5 ms depending on the chosen
    /// repeatability and the supply voltage of the sensor.
    pub fn read(&mut self) -> Result<Sht30Reading, Sht30Error<I2C::Error>> {
        let mut data = [0u8; 6];
        self.i2c.write_read(SHT30_ADDRESS, &READ_CMD, &mut data).map_err(Sht30Error::I2C)?;

        let temperature: &[u8; 2] = &data[0..2].try_into().unwrap();
        let temperature_crc = data[2];
        let humidity: &[u8; 2] = &data[3..5].try_into().unwrap();
        let humidity_crc = data[5];
        Self::check_crc(temperature, temperature_crc)?;
        Self::check_crc(humidity, humidity_crc)?;
        let temperature = (((Self::join_u16(temperature) as f32 * 315.0) / 65535.0) - 49.0) as u16;
        let humidity = ((Self::join_u16(humidity) as f32 * 100.0) / 65535.0) as u16;

        let reading = Sht30Reading::new(humidity, temperature);
        Ok(reading)
    }
}