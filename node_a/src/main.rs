#![no_std]
#![cfg_attr(not(test), no_main)]

mod env_reading;

use bsp::hal::{Sio, Watchdog, pac, Clock, Timer, rtc, I2C, Spi};
use bsp::hal::clocks::init_clocks_and_plls;
use bsp::hal::fugit::RateExtU32;
use bsp::hal::gpio::bank0::{Gpio13, Gpio14, Gpio15, Gpio16, Gpio17, Gpio2, Gpio3, Gpio8};
use bsp::hal::gpio::{FunctionI2C, FunctionSio, FunctionSpi, Pin, PullDown, PullNone, PullUp, SioOutput};
use bsp::hal::pac::{interrupt, I2C1, SPI1};
use bsp::hal::rtc::RealTimeClock;
use core::cell::RefCell;
use critical_section::Mutex;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{OutputPin, PinState};
use heapless::String;
use packed_struct::PackedStruct;
#[cfg(not(test))]
use panic_halt as _;
use pmsa003i::{Pmsa003i, Reading};
use sx127x_lora::LoRa;
use bsp::hal::multicore::{Multicore, Stack};
use bsp::hal::spi::Enabled;
use sht30::{Sht30, Sht30Reading};
use crate::env_reading::EnvReading;

#[cfg(debug_assertions)]
use usb_device::bus::UsbBusAllocator;
#[cfg(debug_assertions)]
use usb_device::prelude::{StringDescriptors, UsbDeviceBuilder, UsbVidPid};
#[cfg(debug_assertions)]
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use bsp::hal::sio::SioFifo;

type I2cBus = I2C<I2C1, (Pin<Gpio2, FunctionI2C, PullUp>, Pin<Gpio3, FunctionI2C, PullUp>)>;
type LedPin = Pin<Gpio13, FunctionSio<SioOutput>, PullDown>;
type Lora = LoRa<SpiBus, Pin<Gpio16, FunctionSio<SioOutput>, PullDown>, Pin<Gpio17, FunctionSio<SioOutput>, PullDown>, Timer>;
type SpiBus = Spi<Enabled, SPI1, (Pin<Gpio15, FunctionSpi, PullNone>, Pin<Gpio8, FunctionSpi, PullNone>, Pin<Gpio14, FunctionSpi, PullNone>)>;

/// Stack for core 1
///
/// Core 0 gets its stack via the normal route - any memory not used by static values is
/// reserved for stack and initialised by cortex-m-rt.
/// To get the same for Core 1, we would need to compile everything separately and
/// modify the linker file for both programs, and that's quite annoying.
/// So instead, core1.spawn takes a [usize] which gets used for the stack.
/// NOTE: We use the `Stack` struct here to ensure that it has 32-byte alignment, which allows
/// the stack guard to take up the least amount of usable RAM.
#[cfg(debug_assertions)]
static CORE1_STACK: Stack<4096> = Stack::new();

const ALARM_INTERVAL_SECONDS: u8 = 15;
//const HAPPY: [u8; 4] = [0x3a, 0x29, 0x0d, 0x0a];
const LORA_FREQUENCY_MHZ: i64 = 915;
const LORA_TX_OK: u32 = 0x0;
const LORA_TX_ERR: u32 = 0x1;
const SECONDS_PER_MINUTE: u8 = 60;
const UNEXPECTED: [u8; 4] = [0x3f, 0x3f, 0x0d, 0x0a];
const UNHAPPY: [u8; 4] = [0x3a, 0x28, 0x0d, 0x0a];

// TODO diff between release and dev statics
static I2C_BUS: Mutex<RefCell<Option<I2cBus>>> = Mutex::new(RefCell::new(None));
static LED: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));
static LORA: Mutex<RefCell<Option<Lora>>> = Mutex::new(RefCell::new(None));
static RTC: Mutex<RefCell<Option<RealTimeClock>>> = Mutex::new(RefCell::new(None));
static SIO_FIFO: Mutex<RefCell<Option<SioFifo>>> = Mutex::new(RefCell::new(None));
static TIMER: Mutex<RefCell<Option<Timer>>> = Mutex::new(RefCell::new(None));
static LAST_ENV_READING: Mutex<RefCell<Option<EnvReading>>> = Mutex::new(RefCell::new(None));

fn blink(delay: u32) {
    critical_section::with(|cs| {
        let mut maybe_led = LED.borrow_ref_mut(cs);
        let mut maybe_timer = TIMER.borrow_ref_mut(cs);

        if let (Some(led), Some(timer)) = (maybe_led.as_mut(), maybe_timer.as_mut()) {
            led.set_high().unwrap();
            timer.delay_ms(delay);
            led.set_low().unwrap();
            timer.delay_ms(delay);
        }
    });
}

#[cfg(debug_assertions)]
fn core1_task(_sys_freq: u32) -> ! {
    let mut pac = unsafe { pac::Peripherals::steal() };
    let mut sio = Sio::new(pac.SIO);
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
        .ok()
        .unwrap();

    let usb_bus = UsbBusAllocator::new(bsp::hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Raspberry Pi")
            .product("RP2 USB Serial")
            .serial_number("E0C9125B0D9B")])
        .unwrap()
        .device_class(USB_CLASS_CDC) // from: https://www.usb.org/defined-class-codes
        .build();

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }
        match sio.fifo.read() {
            Some(word) => {

                match word {
                    LORA_TX_OK => {
                        critical_section::with(|cs| {
                            if let Some(reading) = LAST_ENV_READING.replace(cs, None) {
                                let s: String<32> = reading.into();
                                serial.write(s.as_bytes()).unwrap();
                            }
                        });
                    },
                    LORA_TX_ERR => { serial.write(&UNHAPPY).unwrap(); }
                    _ => { serial.write(&UNEXPECTED).unwrap(); }
                }
            },
            None => continue,
        }
    }
}

fn read_aq() -> Result<Reading, ()> {
    critical_section::with(|cs| {
        let mut maybe_i2c = I2C_BUS.borrow_ref_mut(cs);
        if let Some(i2c_bus) = maybe_i2c.as_mut() {
            let mut pmsa = Pmsa003i::new(i2c_bus);
            Ok(pmsa.read().unwrap())
        } else {
            Err(())
        }
    })
}

fn read_th() -> Result<Sht30Reading, ()> {
    critical_section::with(|cs| {
        let mut maybe_i2c = I2C_BUS.borrow_ref_mut(cs);
        if let Some(i2c_bus) = maybe_i2c.as_mut() {
            let mut sht30 = Sht30::new(i2c_bus);
            Ok(sht30.read().unwrap())
        } else {
            Err(())
        }
    })
}

fn read_sensors() -> Result<EnvReading, ()> {
    let aq_reading = read_aq()?;
    let th_reading = read_th()?;
    Ok(
        EnvReading::new(
            aq_reading.pm2_5,
            aq_reading.pm10,
            th_reading.humidity,
            th_reading.temperature_f
        )
    )
}

fn transmit(env_reading: EnvReading) {
    critical_section::with(|cs| {
        let mut maybe_lora = LORA.borrow_ref_mut(cs);
        let mut maybe_sio_fifo = SIO_FIFO.borrow_ref_mut(cs); // TODO only needed for dev profile

        if let (Some(lora), Some(sio_fifo)) = (maybe_lora.as_mut(), maybe_sio_fifo.as_mut()) {
            let payload: [u8; 8] = env_reading.pack().unwrap();
            let mut buffer = [0; 255];
            for (i, b) in payload.iter().enumerate() {
                buffer[i] = *b;
            }
            match lora.transmit_payload(buffer, payload.len()) {
                Ok(_) => {
                    #[cfg(debug_assertions)]
                    {
                        LAST_ENV_READING.borrow(cs).replace(Some(env_reading));
                        sio_fifo.write(LORA_TX_OK);
                        blink(500);
                    }
                },
                Err(_) => {
                    #[cfg(debug_assertions)]
                    {
                        sio_fifo.write(LORA_TX_ERR);
                        blink(3_000);
                    }
                }
            }
        }
    });
}

// schedule an RTC alarm every ALARM_INTERVAL_SECONDS, wrapping around as needed
fn schedule_next_alarm(rtc: &mut RealTimeClock) {
    let now = rtc.now().unwrap();
    let next = get_next_alarm_secs(now.second);
    rtc.schedule_alarm(rtc::DateTimeFilter::default().second(next));
}

fn get_next_alarm_secs(now_secs: u8) -> u8 {
    if now_secs > (SECONDS_PER_MINUTE - ALARM_INTERVAL_SECONDS - 1) {
        ALARM_INTERVAL_SECONDS - (SECONDS_PER_MINUTE - now_secs)
    } else {
        ALARM_INTERVAL_SECONDS + now_secs
    }
}

#[allow(non_snake_case)]
#[interrupt]
fn RTC_IRQ() { // IRQ25
    critical_section::with(|cs| {
        let mut maybe_rtc = RTC.borrow_ref_mut(cs);
        if let Some(rtc) = maybe_rtc.as_mut() {
            rtc.clear_interrupt();
            schedule_next_alarm(rtc);
        }
    });
}

// TODO create discrete init functions
// TODO logging
#[cfg_attr(not(test), bsp::entry)]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let clocks = init_clocks_and_plls(
        bsp::XOSC_CRYSTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
        .ok()
        .unwrap();
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    critical_section::with(|cs| {
        TIMER.borrow(cs).replace(Some(timer));
    });

    // I2C
    let sda = pins.sda.reconfigure();
    let scl = pins.scl.reconfigure();
    let i2c_bus = bsp::hal::I2C::i2c1(
        pac.I2C1,
        sda,
        scl,
        100.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );
    critical_section::with(|cs| {
        I2C_BUS.borrow(cs).replace(Some(i2c_bus));
    });

    // SPI
    let mosi: Pin<Gpio15, FunctionSpi, PullNone> = pins.mosi.reconfigure();
    let miso: Pin<Gpio8, FunctionSpi, PullNone> = pins.miso.reconfigure();
    let sclk: Pin<Gpio14, FunctionSpi, PullNone> = pins.sclk.reconfigure();
    let spi = bsp::hal::spi::Spi::<_, _, _, 8>::new(pac.SPI1, (mosi, miso, sclk));
    let spi_bus = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16.MHz(),
        embedded_hal::spi::MODE_0,
    );

    // RTC
    let mut rtc = RealTimeClock::new(
        pac.RTC,
        clocks.rtc_clock,
        &mut pac.RESETS,
        rtc::DateTime {
            year: 0,
            month: 1,
            day: 1,
            day_of_week: rtc::DayOfWeek::Monday,
            hour: 0,
            minute: 0,
            second: 0,
        },
    ).unwrap();
    schedule_next_alarm(&mut rtc);
    rtc.enable_interrupt();
    critical_section::with(|cs| {
        RTC.borrow(cs).replace(Some(rtc));
    });

    // LoRa
    let nss = pins.rfm_cs.into_push_pull_output_in_state(PinState::High);
    let rst = pins.rfm_rst.into_push_pull_output_in_state(PinState::High);
    let mut lora = LoRa::new(spi_bus, nss, rst, LORA_FREQUENCY_MHZ, timer).unwrap();
    lora.set_coding_rate_4(8).unwrap();
    lora.set_crc(true).unwrap();
    lora.set_signal_bandwidth(250_000).unwrap();
    lora.set_spreading_factor(10).unwrap();
    critical_section::with(|cs| {
        LORA.borrow(cs).replace(Some(lora));
    });

    let led = pins.led.into_push_pull_output();
    critical_section::with(|cs| {
        LED.borrow(cs).replace(Some(led));
    });

    #[cfg(debug_assertions)]
    {
        let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
        let cores = mc.cores();
        let core1 = &mut cores[1];
        let sys_freq = clocks.system_clock.freq().to_Hz();
        let _ = core1.spawn(CORE1_STACK.take().unwrap(), move || core1_task(sys_freq));
    }
    critical_section::with(|cs| {
        SIO_FIFO.borrow(cs).replace(Some(sio.fifo));
    });

    // TODO use dormant once external RTC is available
    core.SCB.set_sleepdeep();

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::RTC_IRQ);
    }

    loop {
        cortex_m::asm::wfi();
        let env_reading = read_sensors().unwrap();
        transmit(env_reading);
    }
}

#[cfg(test)]
mod tests {
    use crate::*;

    #[test]
    fn get_next_alarm_secs_floor() {
        let res = get_next_alarm_secs(0_u8);
        assert_eq!(res, 15);
    }

    #[test]
    fn get_next_alarm_secs_ceiling() {
        let res = get_next_alarm_secs(59_u8);
        assert_eq!(res, 14);
    }
}