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

#[cfg(test)]
mod tests {
    use crate::*;
    use embedded_hal::i2c::ErrorKind;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    #[test]
    fn read_i2c_error() {
        let expectations = [
            I2cTransaction::write_read(SHT30_ADDRESS, READ_CMD.to_vec(), [2u8, 4u8, 156u8, 8u8, 16u8, 245u8].to_vec()).with_error(ErrorKind::Other)
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut sht30 = Sht30::new(&mut i2c);
        let err = sht30.read().unwrap_err();
        assert_eq!(err, Sht30Error::I2C(ErrorKind::Other));
        i2c.done();
        assert!(true);
    }

    #[test]
    fn read_invalid_crc_error() {
        let expectations = [
            I2cTransaction::write_read(SHT30_ADDRESS, READ_CMD.to_vec(), [0u8; 6].to_vec())
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut sht30 = Sht30::new(&mut i2c);
        match sht30.read() {
            Err(e) => assert_eq!(e, Sht30Error::InvalidCrc),
            _ => panic!("expected an error")
        };
        i2c.done();
    }

    #[test]
    fn read_ok() {
        let expectations = [
            I2cTransaction::write_read(SHT30_ADDRESS, READ_CMD.to_vec(), [0x5f, 0x58, 0x38, 0x7b, 0xb2, 0x7d].to_vec(),)
        ];
        let mut i2c = I2cMock::new(&expectations);
        let mut sht30 = Sht30::new(&mut i2c);
        match sht30.read() {
            Ok(reading) => {
                assert_eq!(reading.humidity, 48);
                assert_eq!(reading.temperature_f, 68);
            },
            Err(e) => panic!("unexpected error: {:?}", e)
        };
        i2c.done();
    }
}