use defmt::{info, panic};
use heapless::Vec;

use super::CommandError;

// Commands require a magic payload to guard against accidental reboot
// from line noise or a framing desync.
const REBOOT_MAGIC: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
const REBOOT_TO_BOOTLOADER_MAGIC: [u8; 4] = [0xB0, 0x07, 0x10, 0xAD];

#[derive(defmt::Format)]
pub enum Command {
    Reboot,
    RebootToBootloader,
}

impl Command {
    pub fn from_bytes(buf: &[u8]) -> Result<Self, CommandError> {
        match buf {
            [0x01, rest @ ..] if rest == REBOOT_MAGIC => Ok(Self::Reboot),
            [0x02, rest @ ..] if rest == REBOOT_TO_BOOTLOADER_MAGIC => {
                Ok(Self::RebootToBootloader)
            }
            _ => Err(CommandError::Invalid),
        }
    }
}

impl super::ControllerCommand for Command {
    async fn handle(
        &self,
        _controller: &mut super::Controller,
    ) -> Result<Vec<u8, 256>, CommandError> {
        match self {
            Command::Reboot => {
                info!("System: rebooting");
                cortex_m::peripheral::SCB::sys_reset()
            }
            Command::RebootToBootloader => {
                info!("System: rebooting to bootloader");
                embassy_rp::rom_data::reset_to_usb_boot(0, 0);
                panic!("reset_to_usb_boot returned");
            }
        }
    }
}
