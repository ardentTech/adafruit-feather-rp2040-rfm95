use core::fmt::Write;
use heapless::String;
use packed_struct::derive::PackedStruct;

#[derive(PackedStruct, Clone, Debug)]
#[packed_struct(endian="lsb")]
pub(crate) struct EnvReading {
    #[packed_field()]
    aq_pm2_5: u16,
    #[packed_field()]
    aq_pm10: u16,
    #[packed_field()]
    humidity: u16,
    #[packed_field()]
    temperature: u16,
}

impl EnvReading {
    pub(crate) fn new(aq_pm2_5: u16, aq_pm10: u16, humidity: u16, temperature: u16) -> Self {
        Self { aq_pm2_5, aq_pm10, humidity, temperature }
    }
}

impl Into<String<32>> for EnvReading {

    fn into(self) -> String<32> {
        let mut msg: String<32> = String::new();
        core::write!(
            &mut msg,
            "T={},H={},PM2.5={},PM10={}\r\n",
            self.temperature, self.humidity, self.aq_pm2_5, self.aq_pm10
        ).unwrap();
        msg
    }
}