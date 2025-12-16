//! PSU I2C interface using bit-banged I2C on GPIO14 (SDA) / GPIO15 (SCL)
//! Operating at 400Hz

use embassy_rp::gpio::{Flex, Pull};
use embassy_time::Timer;
use heapless::Vec;

use super::CommandError;

/// Bit-banged I2C master for PSU communication
pub struct BitBangI2c<'d> {
    sda: Flex<'d>,
    scl: Flex<'d>,
}

// Half period for 400Hz = 1.25ms (1250µs)
const HALF_PERIOD_US: u64 = 1250;
// Timeout for clock stretching (50ms)
const CLOCK_STRETCH_TIMEOUT_US: u64 = 50_000;

impl<'d> BitBangI2c<'d> {
    pub fn new(
        sda_pin: impl embassy_rp::gpio::Pin + 'd,
        scl_pin: impl embassy_rp::gpio::Pin + 'd,
    ) -> Self {
        let mut sda = Flex::new(sda_pin);
        let mut scl = Flex::new(scl_pin);

        // Configure as open-drain with pull-ups
        sda.set_as_input();
        sda.set_pull(Pull::Up);
        scl.set_as_input();
        scl.set_pull(Pull::Up);

        Self { sda, scl }
    }

    #[inline(always)]
    fn sda_high(&mut self) {
        self.sda.set_as_input();
    }

    #[inline(always)]
    fn sda_low(&mut self) {
        self.sda.set_as_output();
        self.sda.set_low();
    }

    #[inline(always)]
    fn scl_release(&mut self) {
        self.scl.set_as_input();
    }

    #[inline(always)]
    fn scl_low(&mut self) {
        self.scl.set_as_output();
        self.scl.set_low();
    }

    #[inline(always)]
    fn read_sda(&self) -> bool {
        self.sda.is_high()
    }

    #[inline(always)]
    fn read_scl(&self) -> bool {
        self.scl.is_high()
    }

    async fn delay(&self) {
        Timer::after_micros(HALF_PERIOD_US).await;
    }

    /// Release SCL and wait for it to go high (handles clock stretching)
    /// Returns Ok(()) if SCL went high, Err(()) if timeout
    async fn scl_high_wait(&mut self) -> Result<(), ()> {
        self.scl_release();
        
        // Wait for SCL to actually go high (slave may be stretching clock)
        let mut waited: u64 = 0;
        while !self.read_scl() {
            Timer::after_micros(10).await;
            waited += 10;
            if waited > CLOCK_STRETCH_TIMEOUT_US {
                return Err(());
            }
        }
        Ok(())
    }

    /// Generate I2C start condition
    async fn start(&mut self) -> Result<(), ()> {
        // Ensure both lines are high
        self.sda_high();
        self.scl_high_wait().await?;
        self.delay().await;

        // SDA goes low while SCL is high
        self.sda_low();
        self.delay().await;

        // Then SCL goes low
        self.scl_low();
        self.delay().await;
        Ok(())
    }

    /// Generate I2C stop condition
    async fn stop(&mut self) -> Result<(), ()> {
        // Ensure SDA is low
        self.sda_low();
        self.delay().await;

        // SCL goes high (wait for clock stretching)
        self.scl_high_wait().await?;
        self.delay().await;

        // SDA goes high while SCL is high
        self.sda_high();
        self.delay().await;
        Ok(())
    }

    /// Write a byte and return the ACK bit (false = ACK, true = NACK)
    async fn write_byte(&mut self, byte: u8) -> Result<bool, ()> {
        for i in (0..8).rev() {
            if (byte >> i) & 1 == 1 {
                self.sda_high();
            } else {
                self.sda_low();
            }
            self.delay().await;

            self.scl_high_wait().await?;
            self.delay().await;
            self.scl_low();
            self.delay().await;
        }

        // Release SDA for ACK
        self.sda_high();
        self.delay().await;

        self.scl_high_wait().await?;
        self.delay().await;
        let nack = self.read_sda();
        self.scl_low();
        self.delay().await;

        Ok(nack)
    }

    /// Read a byte and send ACK (ack=true) or NACK (ack=false)
    async fn read_byte(&mut self, ack: bool) -> Result<u8, ()> {
        let mut byte = 0u8;

        self.sda_high(); // Release SDA for reading

        for i in (0..8).rev() {
            self.delay().await;
            self.scl_high_wait().await?;
            self.delay().await;

            if self.read_sda() {
                byte |= 1 << i;
            }

            self.scl_low();
        }

        // Send ACK or NACK
        if ack {
            self.sda_low();
        } else {
            self.sda_high();
        }
        self.delay().await;

        self.scl_high_wait().await?;
        self.delay().await;
        self.scl_low();
        self.delay().await;

        self.sda_high();

        Ok(byte)
    }

    /// Write data to an I2C device
    pub async fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), ()> {
        self.start().await?;

        // Send address with write bit (0)
        if self.write_byte(addr << 1).await? {
            let _ = self.stop().await;
            return Err(());
        }

        // Send data bytes
        for byte in data {
            if self.write_byte(*byte).await? {
                let _ = self.stop().await;
                return Err(());
            }
        }

        self.stop().await?;
        Ok(())
    }

    /// Read data from an I2C device
    pub async fn read(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), ()> {
        self.start().await?;

        // Send address with read bit (1)
        if self.write_byte((addr << 1) | 1).await? {
            let _ = self.stop().await;
            return Err(());
        }

        // Read data bytes
        let len = buf.len();
        for (i, byte) in buf.iter_mut().enumerate() {
            // ACK all bytes except the last one
            *byte = self.read_byte(i < len - 1).await?;
        }

        self.stop().await?;
        Ok(())
    }

    /// Write then read (combined transaction)
    pub async fn write_read(&mut self, addr: u8, write_data: &[u8], read_buf: &mut [u8]) -> Result<(), ()> {
        self.start().await?;

        // Send address with write bit (0)
        if self.write_byte(addr << 1).await? {
            let _ = self.stop().await;
            return Err(());
        }

        // Send data bytes
        for byte in write_data {
            if self.write_byte(*byte).await? {
                let _ = self.stop().await;
                return Err(());
            }
        }

        // Repeated start
        self.start().await?;

        // Send address with read bit (1)
        if self.write_byte((addr << 1) | 1).await? {
            let _ = self.stop().await;
            return Err(());
        }

        // Read data bytes
        let len = read_buf.len();
        for (i, byte) in read_buf.iter_mut().enumerate() {
            // ACK all bytes except the last one
            *byte = self.read_byte(i < len - 1).await?;
        }

        self.stop().await?;
        Ok(())
    }
}

#[derive(defmt::Format)]
pub enum Command {
    Write { addr: u8, buf: Vec<u8, 256> },                   // 0x20
    Read { addr: u8, len: u8 },                              // 0x30
    WriteRead { addr: u8, buf: Vec<u8, 256>, read_len: u8 }, // 0x40
}

impl Command {
    pub fn from_bytes(buf: &[u8]) -> Result<Self, CommandError> {
        match buf {
            [0x20, addr, buf @ ..] => Ok(Self::Write {
                addr: *addr,
                buf: Vec::from_slice(buf).map_err(|_| CommandError::BufferOverflow)?,
            }),
            [0x30, addr, len] => Ok(Self::Read { addr: *addr, len: *len }),
            [0x40, addr, buf @ .., read_len] => Ok(Self::WriteRead {
                addr: *addr,
                buf: Vec::from_slice(buf).map_err(|_| CommandError::BufferOverflow)?,
                read_len: *read_len,
            }),
            _ => Err(CommandError::Invalid),
        }
    }
}

impl Command {
    pub async fn handle(&self, psu_i2c: &mut BitBangI2c<'_>) -> Result<Vec<u8, 256>, CommandError> {
        match self {
            Command::Write { addr, buf } => {
                psu_i2c
                    .write(*addr, buf)
                    .await
                    .map_err(|_| CommandError::Message("PSU I2C Write Error"))?;
                Ok(Vec::from_slice(&[buf.len() as u8]).unwrap())
            }

            Command::Read { addr, len } => {
                let mut buf = Vec::new();
                let _ = buf.resize_default(*len as usize);
                psu_i2c
                    .read(*addr, &mut buf)
                    .await
                    .map_err(|_| CommandError::Message("PSU I2C Read Error"))?;
                Ok(buf)
            }

            Command::WriteRead { addr, buf, read_len } => {
                let mut read_buf = Vec::new();
                let _ = read_buf.resize_default(*read_len as usize);
                psu_i2c
                    .write_read(*addr, buf, &mut read_buf)
                    .await
                    .map_err(|_| CommandError::Message("PSU I2C WriteRead Error"))?;
                Ok(read_buf)
            }
        }
    }
}
