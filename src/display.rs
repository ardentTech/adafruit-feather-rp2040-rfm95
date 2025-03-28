use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_rp::gpio::Input;
use embassy_time::Timer;
use embedded_graphics::Drawable;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::Point;
use embedded_graphics::text::{Baseline, Text};
use oled_async::Builder;
use oled_async::displayrotation::DisplayRotation;
use oled_async::prelude::GraphicsMode;
use crate::common::I2c1Bus;

type Display = oled_async::displays::sh1107::Sh1107_64_128;

#[embassy_executor::task]
pub async fn oled_display(
    i2c_bus: &'static I2c1Bus,
    btn_a: Input<'static>,
    btn_b: Input<'static>,
    btn_c: Input<'static>,
) {
    let i2c_dev = I2cDevice::new(i2c_bus);
    let di = display_interface_i2c::I2CInterface::new(
        i2c_dev,
        0x3c,
        0x40
    );
    let raw_disp = Builder::new(Display {})
        .with_rotation(DisplayRotation::Rotate90)
        .connect(di);
    let mut display: GraphicsMode<_, _, { 128 * 64 / 8 }> = raw_disp.into();
    // reset is mapped appropriately by stacking the oled on top of the feather
    display.init().await.unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    loop {
        display.clear();
        let msg = if btn_a.is_low() {
            "button a!"
        } else if btn_b.is_low() {
            "button b!"
        } else if btn_c.is_low() {
            "button c!"
        }  else {
            "idle..."
        };
        Text::with_baseline(msg, Point::new(0, 16), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        display.flush().await.unwrap();
        Timer::after_millis(250).await;
    }
}