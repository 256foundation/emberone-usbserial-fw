use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Alignment, Text},
};
use heapless::Vec;
use profont::{PROFONT_18_POINT, PROFONT_10_POINT};
use ssd1306::{
    mode::BufferedGraphicsMode,
    prelude::*,
    I2CDisplayInterface,
    Ssd1306,
};

use super::CommandError;

pub type DisplayI2c = embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Async>;

pub struct Display {
    driver: Ssd1306<I2CInterface<DisplayI2c>, DisplaySize128x32, BufferedGraphicsMode<DisplaySize128x32>>,
}

impl Display {
    pub fn new(i2c: DisplayI2c) -> Self {
        let interface = I2CDisplayInterface::new(i2c);
        let mut driver = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        driver.init().unwrap();
        driver.clear_buffer();

        // Show startup splash
        let style = MonoTextStyleBuilder::new()
            .font(&PROFONT_18_POINT)
            .text_color(BinaryColor::On)
            .build();
        let _ = Text::with_alignment("BITCRANE", Point::new(64, 22), style, Alignment::Center)
            .draw(&mut driver);
        driver.flush().unwrap();

        Self { driver }
    }

    pub fn set_hashrate(&mut self, text: &str) -> Result<(), CommandError> {
        self.driver.clear_buffer();

        // Split on comma to separate top line from bottom line
        // e.g. "1.25 TH/s,mining" -> ("1.25 TH/s", "mining")
        if let Some(pos) = text.find(',') {
            let (top, bottom) = text.split_at(pos);
            let bottom = &bottom[1..]; // skip the comma

            let large_style = MonoTextStyleBuilder::new()
                .font(&PROFONT_18_POINT)
                .text_color(BinaryColor::On)
                .build();

            let small_style = MonoTextStyleBuilder::new()
                .font(&PROFONT_10_POINT)
                .text_color(BinaryColor::On)
                .build();

            // Top line: large text, centered at y=16 (baseline for 18pt on 32px tall display)
            Text::with_alignment(top, Point::new(64, 16), large_style, Alignment::Center)
                .draw(&mut self.driver)
                .map_err(|_| CommandError::Message("Display draw error"))?;

            // Bottom line: smaller text, centered at y=30
            Text::with_alignment(bottom, Point::new(64, 30), small_style, Alignment::Center)
                .draw(&mut self.driver)
                .map_err(|_| CommandError::Message("Display draw error"))?;
        } else {
            // No comma found — render entire string as single large line, vertically centered
            let large_style = MonoTextStyleBuilder::new()
                .font(&PROFONT_18_POINT)
                .text_color(BinaryColor::On)
                .build();

            Text::with_alignment(text, Point::new(64, 22), large_style, Alignment::Center)
                .draw(&mut self.driver)
                .map_err(|_| CommandError::Message("Display draw error"))?;
        }

        self.driver.flush().map_err(|_| CommandError::Message("Display flush error"))?;
        Ok(())
    }
}

#[derive(defmt::Format)]
pub enum Command {
    SetHashrate { text: Vec<u8, 32> }, // 0x10
}

impl Command {
    pub fn from_bytes(buf: &[u8]) -> Result<Self, CommandError> {
        match buf {
            [0x10, data @ ..] => {
                let text = Vec::from_slice(data).map_err(|_| CommandError::BufferOverflow)?;
                Ok(Self::SetHashrate { text })
            }
            _ => Err(CommandError::Invalid),
        }
    }
}

impl super::ControllerCommand for Command {
    async fn handle(&self, controller: &mut super::Controller) -> Result<Vec<u8, 256>, CommandError> {
        match self {
            Command::SetHashrate { text } => {
                let s = core::str::from_utf8(text).map_err(|_| CommandError::Message("Invalid UTF-8"))?;
                controller.display.set_hashrate(s)?;
                // Echo back the text length as confirmation
                Ok(Vec::from_slice(&[text.len() as u8]).unwrap())
            }
        }
    }
}
