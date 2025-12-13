use super::CommandError;
use heapless::Vec;

pub struct Pins<'d> {
    pub rst0: embassy_rp::gpio::Output<'d>,
    pub plug0: embassy_rp::gpio::Input<'d>,
    pub rst1: embassy_rp::gpio::Output<'d>,
    pub plug1: embassy_rp::gpio::Input<'d>,
    pub rst2: embassy_rp::gpio::Output<'d>,
    pub plug2: embassy_rp::gpio::Input<'d>,
    pub psu_en: embassy_rp::gpio::Output<'d>,
}

#[derive(defmt::Format)]
pub enum Command {
    SetRst0 { level: bool },
    GetRst0,
    GetPlug0,

    SetRst1 { level: bool },
    GetRst1,
    GetPlug1,

    SetRst2 { level: bool },
    GetRst2,
    GetPlug2,

    SetPsuEn { level: bool },
    GetPsuEn,
}

impl Command {
    pub fn from_bytes(buf: &[u8]) -> Result<Self, CommandError> {
        defmt::println!("GETTING GPIO COMMAND FROM BYTES {:x}", buf);
        match buf {
            [0x00] => Ok(Self::GetRst0),
            [0x00, level] => Ok(Self::SetRst0 { level: *level > 0 }),
            [0x01] => Ok(Self::GetPlug0),

            [0x10] => Ok(Self::GetRst1),
            [0x10, level] => Ok(Self::SetRst1 { level: *level > 0 }),
            [0x11] => Ok(Self::GetPlug1),

            [0x20] => Ok(Self::GetRst2),
            [0x20, level] => Ok(Self::SetRst2 { level: *level > 0 }),
            [0x21] => Ok(Self::GetPlug2),

            [0x50] => Ok(Self::GetPsuEn),
            [0x50, level] => Ok(Self::SetPsuEn { level: *level > 0 }),

            _ => Err(CommandError::Invalid),
        }
    }
}

impl super::ControllerCommand for Command {
    async fn handle(&self, controller: &mut super::Controller) -> Result<Vec<u8, 256>, CommandError> {
        let level = match self {
            Command::GetRst0 => bool::from(controller.gpio.rst0.get_output_level()),
            Command::SetRst0 { level } => {
                controller.gpio.rst0.set_level((*level).into());
                *level
            }
            Command::GetPlug0 => controller.gpio.plug0.is_high(),

            Command::GetRst1 => bool::from(controller.gpio.rst1.get_output_level()),
            Command::SetRst1 { level } => {
                controller.gpio.rst1.set_level((*level).into());
                *level
            }
            Command::GetPlug1 => controller.gpio.plug1.is_high(),

            Command::GetRst2 => bool::from(controller.gpio.rst2.get_output_level()),
            Command::SetRst2 { level } => {
                controller.gpio.rst2.set_level((*level).into());
                *level
            }
            Command::GetPlug2 => controller.gpio.plug2.is_high(),

            Command::GetPsuEn => bool::from(controller.gpio.psu_en.get_output_level()),
            Command::SetPsuEn { level } => {
                controller.gpio.psu_en.set_level((*level).into());
                *level
            }
        };

        Ok(Vec::from_slice(&[level as u8]).unwrap())
    }
}
