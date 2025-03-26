use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_rp::gpio::{Input, Output};
use embassy_rp::peripherals::SPI1;
use embassy_rp::spi::{Async, Spi};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_time::{Delay, Timer};
use lora_phy::{sx127x, LoRa, RxMode};
use lora_phy::iv::GenericSx127xInterfaceVariant;
use lora_phy::mod_params::{Bandwidth, CodingRate, ModulationParams, PacketParams, PacketStatus, RadioError, SpreadingFactor};
use lora_phy::mod_traits::RadioKind;
use lora_phy::sx127x::{Sx1276, Sx127x};
use crate::Spi1Bus;

const LORA_FREQUENCY: u32 = 915_000_000;
const PREAMBLE_LENGTH: u16 = 4;
const IMPLICIT_HEADER: bool = false;
const CRC_ON: bool = true;
const IQ_INVERTED: bool = false;
const OUTPUT_POWER: i32 = 20;
const SPREADING_FACTOR: SpreadingFactor = SpreadingFactor::_10;
const BANDWIDTH: Bandwidth = Bandwidth::_250KHz;
const CODING_RATE: CodingRate = CodingRate::_4_8;

#[derive(Debug)]
pub enum RadioErr {
    GetModParams,
    GetRxPacketParams,
    GetTxPacketParams,
    PrepareRx,
    PrepareTx,
    Rx,
    Sleep,
    Tx,
}

pub struct Radio {
    lora: LoRa<Sx127x<SpiDevice<'static, NoopRawMutex, Spi<'static, SPI1, Async>, Output<'static>>, GenericSx127xInterfaceVariant<Output<'static>, Input<'static>>, Sx1276>, Delay>
}

impl Radio {
    pub async fn new(
        spi_bus: &'static Spi1Bus,
        chip_select: Output<'static>,
        reset: Output<'static>,
        dio0: Input<'static>,
    ) -> Self {
        let spi_device = SpiDevice::new(spi_bus, chip_select);
        let config = sx127x::Config {
            chip: Sx1276,
            // The lora-rs rp example (https://github.com/lora-rs/lora-rs/blob/main/examples/rp/src/bin/lora_p2p_send.rs)
            // targets the 1262 module which has some differences from the 1276 module on the
            // Adafruit Feather RP2040 RFM95. So I also referenced the lora-rs stm32l0 example (https://github.com/lora-rs/lora-rs/blob/main/examples/stm32l0/src/bin/lora_p2p_send.rs#L40)
            // which DOES target the 1276. I didn't pay much attention to `tcxo_used` being set to
            // `true`, and spent over a day poking, prodding and rewriting my integration to
            // troubleshoot tx and rx silent failures.
            tcxo_used: false,
            tx_boost: false,
            rx_boost: false,
        };
        // To use a `RxMode` other than `Continuous` create a new InterfaceVariant (based on
        // `GenericSx127xInterfaceVariant` that also requires dio1 so that the `RxTimeout` interrupt
        // can be handled.
        // See page 41 of https://cdn-shop.adafruit.com/product-files/5714/SX1276-7-8.pdf
        let iv = GenericSx127xInterfaceVariant::new(
            reset,
            dio0,
            None,
            None
        ).unwrap();
        let lora = LoRa::new(
            Sx127x::new(spi_device, iv, config), true, Delay
        ).await.unwrap();
        Self { lora }
    }

    fn get_mod_params(&mut self) -> Result<ModulationParams, RadioErr> {
        self.lora.create_modulation_params(SPREADING_FACTOR, BANDWIDTH, CODING_RATE, LORA_FREQUENCY).map_err(|e| RadioErr::GetModParams)
    }

    // fn get_rx_packet_params(
    //     &mut self,
    //     mdltn_params: &ModulationParams,
    //     buffer_len: u8,
    // ) -> Result<PacketParams, RadioErr> {
    //     self.lora.create_rx_packet_params(PREAMBLE_LENGTH, IMPLICIT_HEADER, buffer_len, CRC_ON, IQ_INVERTED, &mdltn_params).map_err(|e| RadioErr::GetRxPacketParams)
    // }

    fn get_tx_packet_params(
        &mut self,
        mdltn_params: &ModulationParams,
    ) -> Result<PacketParams, RadioErr> {
        self.lora.create_tx_packet_params(PREAMBLE_LENGTH, IMPLICIT_HEADER, CRC_ON, IQ_INVERTED, &mdltn_params).map_err(|e| RadioErr::GetTxPacketParams)
    }

    // async fn prepare_rx(
    //     &mut self,
    //     mdltn_params: &ModulationParams,
    //     pkt_params: &PacketParams,
    // ) -> Result<(), RadioErr> {
    //     // `RxMode::Continuous` is hard-coded because of the interface variant restriction described above.
    //     self.lora.prepare_for_rx(RxMode::Continuous, &mdltn_params, &pkt_params).await.map_err(|e| RadioErr::PrepareRx)
    // }

    async fn prepare_tx(
        &mut self,
        buffer: &[u8],
        mdltn_params: &ModulationParams,
        pkt_params: &mut PacketParams,
    ) -> Result<(), RadioErr> {
        self.lora.prepare_for_tx(&mdltn_params, pkt_params, OUTPUT_POWER, buffer).await.map_err(|e| RadioErr::PrepareTx)
    }





    // pub async fn rx(
    //     &mut self,
    //     pkt_params: &PacketParams,
    //     buffer: &mut [u8]
    // ) -> Result<(u8, PacketStatus), RadioErr> {
    //     self.lora.rx(&pkt_params, buffer).await.map_err(|e| RadioErr::Rx)
    // }

    pub async fn tx(
        &mut self,
        buffer:&[u8],
        mdltn_params: &ModulationParams,
        pkt_params: &mut PacketParams
    ) -> Result<(), RadioErr> {
        self.lora.prepare_for_tx(&mdltn_params, pkt_params, OUTPUT_POWER, buffer).await.map_err(|e| RadioErr::PrepareTx)?;
        self.lora.tx().await.map_err(|e| RadioErr::Tx)
    }

    pub async fn init_tx(&mut self) -> Result<((ModulationParams, PacketParams)), RadioErr> {
        let mod_params = self.get_mod_params()?;
        let packet_params = self.get_tx_packet_params(&mod_params)?;
        Ok((mod_params, packet_params))
    }

    pub async fn sleep(&mut self) -> Result<(), RadioErr> {
        self.lora.sleep(false).await.map_err(|e| RadioErr::Sleep)
    }
}

#[embassy_executor::task]
pub async fn radio_tx(
    spi_bus: &'static Spi1Bus,
    nss: Output<'static>,
    reset: Output<'static>,
    dio0: Input<'static>
) {
    let mut radio = Radio::new(spi_bus, nss, reset, dio0).await;
    let (mod_params, mut packet_params) = radio.init_tx().await.unwrap();

    // let mod_params = radio.get_mdltn_params().unwrap();
    // let mut pkt_params = radio.get_tx_packet_params(&mod_params).unwrap();
    let buffer = [0x01u8, 0x02u8, 0x03u8];

    loop {
        log::info!("TX loop");
        // radio.prepare_tx(&buffer, &mod_params, &mut packet_params).await.unwrap();
        // log::info!("TX prepared");
        radio.tx(&buffer, &mod_params, &mut packet_params).await.unwrap();
        log::info!("TX done");
        Timer::after_secs(3).await;
    }
}