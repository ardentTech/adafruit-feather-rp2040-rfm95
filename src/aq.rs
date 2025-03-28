use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Sender;
use embassy_time::{Duration, Ticker};
use pmsa003i::Pmsa003i;
use crate::common::{AQSensor, I2c1Bus};

// The PMSA003I sensor ONLY works with 100 kbps I2C. The datasheet states: "100K sps, Standard NXP
// EEPROM Protocol." and bc of the "sps" unit I glanced over this without fully registering it.
// Fast-forward nearly a day after troubleshooting with 400 kbps I2C, and, yeah. Rookie.

#[embassy_executor::task]
pub async fn read_aq(
    i2c_bus: &'static I2c1Bus,
    // channel will buffer up to 64 messages
    control: Sender<'static, ThreadModeRawMutex, AQSensor, 64>
) {
    let i2c_dev = I2cDevice::new(i2c_bus);
    let mut aq_sensor = Pmsa003i::new(i2c_dev);
    let mut ticker = Ticker::every(Duration::from_secs(3));

    loop {
        match aq_sensor.read().await {
            Ok(reading) => control.send(AQSensor::DataReady(reading)).await,
            Err(_) => control.send(AQSensor::Error).await
        }
        ticker.next().await;
    }
}