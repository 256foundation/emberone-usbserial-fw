use super::CommandError;
use heapless::Vec;

pub struct Pins<'d> {
    pub asic_resetn: embassy_rp::gpio::Output<'d>,
    pub asic_pwr_en: embassy_rp::gpio::Output<'d>,
    pub asic_io_pwr_en: embassy_rp::gpio::Output<'d>,
    pub pgood: embassy_rp::gpio::Input<'d>,
    pub therm_n: embassy_rp::gpio::Input<'d>,
    pub smb_alrt_n: embassy_rp::gpio::Input<'d>,
}

#[derive(defmt::Format)]
pub enum Command {
    SetAsicResetn { level: bool },
    GetAsicResetn,

    SetAsicPowerEnable { level: bool },
    GetAsicPowerEnable,

    SetAsicIoPowerEnable { level: bool },
    GetAsicIoPowerEnable,

    GetPgood,
    GetThermN,
    GetSmbAlrtN,
}

impl Command {
    pub fn from_bytes(buf: &[u8]) -> Result<Self, CommandError> {
        defmt::println!("GETTING GPIO COMMAND FROM BYTES {:x}", buf);
        match buf {
            // Get ASIC Reset (Active Low)
            [0x00] => Ok(Self::GetAsicResetn),
            // Set ASIC Reset (Active Low)
            [0x00, level] => Ok(Self::SetAsicResetn { level: *level > 0 }),
            // Get ASIC Power Enable (Active High)
            [0x01] => Ok(Self::GetAsicPowerEnable),
            // Set ASIC Power EN (Active High)
            [0x01, level] => Ok(Self::SetAsicPowerEnable { level: *level > 0 }),
            // Get ASIC IO Power Enable (Active High)
            [0x02] => Ok(Self::GetAsicIoPowerEnable),
            // Set ASIC IO Power EN (Active High)
            [0x02, level] => Ok(Self::SetAsicIoPowerEnable { level: *level > 0 }),
            // Get PGOOD (Input)
            [0x03] => Ok(Self::GetPgood),
            // Get THERM_N (Input, Active Low)
            [0x04] => Ok(Self::GetThermN),
            // Get SMB_ALRT_N (Input, Active Low)
            [0x05] => Ok(Self::GetSmbAlrtN),
            _ => Err(CommandError::Invalid),
        }
    }
}

impl super::ControllerCommand for Command {
    async fn handle(&self, controller: &mut super::Controller) -> Result<Vec<u8, 256>, CommandError> {
        let level = match self {
            Command::GetAsicResetn => bool::from(controller.gpio.asic_resetn.get_output_level()),
            Command::SetAsicResetn { level } => {
                controller.gpio.asic_resetn.set_level((*level).into());
                *level
            }
            Command::GetAsicPowerEnable => bool::from(controller.gpio.asic_pwr_en.get_output_level()),
            Command::SetAsicPowerEnable { level } => {
                controller.gpio.asic_pwr_en.set_level((*level).into());
                *level
            }
            Command::GetAsicIoPowerEnable => bool::from(controller.gpio.asic_io_pwr_en.get_output_level()),
            Command::SetAsicIoPowerEnable { level } => {
                controller.gpio.asic_io_pwr_en.set_level((*level).into());
                *level
            }
            Command::GetPgood => controller.gpio.pgood.is_high(),
            Command::GetThermN => controller.gpio.therm_n.is_high(),
            Command::GetSmbAlrtN => controller.gpio.smb_alrt_n.is_high(),
        };

        Ok(Vec::from_slice(&[level as u8]).unwrap())
    }
}
