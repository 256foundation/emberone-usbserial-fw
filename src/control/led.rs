use embassy_rp::{
    peripherals::{PIO0},
    pio::{Common, PioPin, StateMachine},
    pio_programs::ws2812::{PioWs2812, PioWs2812Program},
};
use heapless::Vec;
use smart_leds::RGB8;

use super::CommandError;

pub struct Led<'d> {
    ws2812: PioWs2812<'d, PIO0, 0, 1>,
}

impl<'d> Led<'d> {
    pub fn new(common: &mut Common<'d, PIO0>, sm: StateMachine<'d, PIO0, 0>, pin: impl PioPin, dma: embassy_rp::dma::AnyChannel) -> Self {
        let program = PioWs2812Program::new(common);
        let ws2812 = PioWs2812::new(common, sm, dma, pin, &program);
        Self { ws2812 }
    }

    pub async fn set_color(&mut self, r: u8, g: u8, b: u8) {
        let data = [RGB8::new(r, g, b)];
        self.ws2812.write(&data).await;
    }
}

#[derive(defmt::Format)]
pub enum Command {
    SetRGB { r: u8, g: u8, b: u8 }, // 0x10
}

impl Command {
    pub fn from_bytes(buf: &[u8]) -> Result<Self, CommandError> {
        //defmt::println!("SETTING LED {:x}", buf);
        match buf {
            [0x10, r, g, b] => Ok(Self::SetRGB { r: *r, g: *g, b: *b }),
            _ => Err(CommandError::Invalid),
        }
    }
}

impl super::ControllerCommand for Command {
    async fn handle(&self, controller: &mut super::Controller) -> Result<Vec<u8, 256>, CommandError> {
        match self {
            Command::SetRGB { r, g, b } => {
                controller.led.set_color(*r, *g, *b).await;
                Ok(Vec::from_slice(&[*r, *g, *b]).unwrap()) // Echo back the set color
            }
        }
    }
}
