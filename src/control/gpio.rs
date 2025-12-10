use super::CommandError;
use heapless::Vec;

pub struct Pins<'d> {
    pub rst0: embassy_rp::gpio::Output<'d>,
    pub plug0: embassy_rp::gpio::Output<'d>,
    pub rst1: embassy_rp::gpio::Output<'d>,
    pub plug1: embassy_rp::gpio::Output<'d>,
    pub rst2: embassy_rp::gpio::Output<'d>,
    pub plug2: embassy_rp::gpio::Output<'d>,
}

#[derive(defmt::Format)]
pub enum Command {
    SetRst0 { level: bool },
    GetRst0,
    SetPlug0 { level: bool },
    GetPlug0,

    SetRst1 { level: bool },
    GetRst1,
    SetPlug1 { level: bool },
    GetPlug1,

    SetRst2 { level: bool },
    GetRst2,
    SetPlug2 { level: bool },
    GetPlug2,
}

impl Command {
    pub fn from_bytes(buf: &[u8]) -> Result<Self, CommandError> {
        defmt::println!("GETTING GPIO COMMAND FROM BYTES {:x}", buf);
        match buf {
            [0x00] => Ok(Self::GetRst0),
            [0x00, level] => Ok(Self::SetRst0 { level: *level > 0 }),
            [0x01] => Ok(Self::GetPlug0),
            [0x01, level] => Ok(Self::SetPlug0 { level: *level > 0 }),

            [0x02] => Ok(Self::GetRst1),
            [0x02, level] => Ok(Self::SetRst1 { level: *level > 0 }),
            [0x03] => Ok(Self::GetPlug1),
            [0x03, level] => Ok(Self::SetPlug1 { level: *level > 0 }),

            [0x04] => Ok(Self::GetRst2),
            [0x04, level] => Ok(Self::SetRst2 { level: *level > 0 }),
            [0x05] => Ok(Self::GetPlug2),
            [0x05, level] => Ok(Self::SetPlug2 { level: *level > 0 }),

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
            Command::GetPlug0 => bool::from(controller.gpio.plug0.get_output_level()),
            Command::SetPlug0 { level } => {
                controller.gpio.plug0.set_level((*level).into());
                *level
            }

            Command::GetRst1 => bool::from(controller.gpio.rst1.get_output_level()),
            Command::SetRst1 { level } => {
                controller.gpio.rst1.set_level((*level).into());
                *level
            }
            Command::GetPlug1 => bool::from(controller.gpio.plug1.get_output_level()),
            Command::SetPlug1 { level } => {
                controller.gpio.plug1.set_level((*level).into());
                *level
            }

            Command::GetRst2 => bool::from(controller.gpio.rst2.get_output_level()),
            Command::SetRst2 { level } => {
                controller.gpio.rst2.set_level((*level).into());
                *level
            }
            Command::GetPlug2 => bool::from(controller.gpio.plug2.get_output_level()),
            Command::SetPlug2 { level } => {
                controller.gpio.plug2.set_level((*level).into());
                *level
            }
        };

        Ok(Vec::from_slice(&[level as u8]).unwrap())
    }
}
