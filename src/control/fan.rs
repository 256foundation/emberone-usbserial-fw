use embassy_rp::{gpio, pwm};
use embassy_time::{Duration, Instant, Timer};

use super::{CommandError, Controller, ControllerCommand};

pub struct Pins<'d> {
    pub pwm: pwm::Pwm<'d>,
    pub tach: gpio::Input<'d>,
}

#[derive(defmt::Format)]
pub enum Command {
    SetSpeed(u8),  // 0-100 percent
    GetTach,
}

impl Command {
    pub fn from_bytes(buf: &[u8]) -> Result<Self, CommandError> {
        match buf[0] {
            0x10 => {
                if buf.len() < 2 {
                    return Err(CommandError::Invalid);
                }
                let speed = buf[1];
                if speed > 100 {
                    return Err(CommandError::Invalid);
                }
                Ok(Command::SetSpeed(speed))
            }
            0x20 => Ok(Command::GetTach),
            _ => Err(CommandError::Invalid),
        }
    }
}

impl ControllerCommand for Command {
    async fn handle(&self, controller: &mut Controller) -> Result<heapless::Vec<u8, 256>, CommandError> {
        match self {
            Command::SetSpeed(speed) => {
                // PWM frequency is ~25kHz (typical for PC fans)
                // Convert percentage to duty cycle (top is 1000, so duty is 0-1000)
                // PWM is HIGH when counter < compare_a
                let duty = (1000u32 * (*speed as u32) / 100) as u16;
                
                // Update the PWM configuration with new compare value
                let mut config = pwm::Config::default();
                config.top = 1000;
                config.compare_a = duty;
                config.compare_b = 0;
                config.divider = 5.into();
                config.invert_a = true;  // Invert PWM for standard fan control (LOW = full speed)
                config.phase_correct = false;
                config.enable = true;
                controller.fan.pwm.set_config(&config);
                
                let mut res = heapless::Vec::new();
                res.push(0x00).unwrap(); // Success
                Ok(res)
            }
            Command::GetTach => {
                // Measure fan tachometer pulses
                // Most fans output 2 pulses per revolution
                let rpm = measure_fan_rpm(&mut controller.fan.tach).await?;
                
                let mut res = heapless::Vec::new();
                res.extend_from_slice(&rpm.to_le_bytes()).unwrap();
                Ok(res)
            }
        }
    }
}

/// Measure fan RPM by counting tachometer pulses over a period
/// Most PC fans output 2 pulses per revolution
async fn measure_fan_rpm(tach: &mut gpio::Input<'_>) -> Result<u16, CommandError> {
    let measurement_time = Duration::from_millis(250);
    let start = Instant::now();
    let mut pulse_count = 0u32;
    let mut last_state = tach.is_high();
    
    // Count falling edges for 250ms (fans typically pull low)
    while start.elapsed() < measurement_time {
        let current_state = tach.is_high();
        // Detect falling edge (high to low transition)
        if !current_state && last_state {
            pulse_count += 1;
        }
        last_state = current_state;
        Timer::after_micros(50).await; // Sample at 20kHz for better edge detection
    }
    
    // Calculate RPM: (pulses / 2) * (60 / 0.25) = pulses * 120
    // Most fans have 2 pulses per revolution
    let rpm = (pulse_count * 120) as u16;
    
    Ok(rpm)
}
